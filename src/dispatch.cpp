#include "smartride/dispatch.hpp"
#include "smartride/wal.hpp"
#include <cmath>
#include <stdexcept>
#include <utility>

namespace sr {
namespace {
bool finite_point(Point p) {
    return std::isfinite(p.x) && std::isfinite(p.y);
}
}

Dispatcher::Dispatcher(std::vector<DispatchDriver> drivers, unsigned workers)
    : Dispatcher(std::move(drivers), workers, std::string()) {}

Dispatcher::Dispatcher(std::vector<DispatchDriver> drivers, unsigned workers, const std::string& wal_path) {
    if (workers == 0) throw std::invalid_argument("dispatcher needs at least one worker");
    drivers_.reserve(drivers.size());
    states_.reset(new std::atomic<unsigned char>[drivers.size()]);
    index_.reserve(drivers.size());
    for (size_t i = 0; i < drivers.size(); ++i) {
        const auto& driver = drivers[i];
        if (driver.id == invalid || !finite_point(driver.position) ||
            !index_.emplace(driver.id, i).second)
            throw std::invalid_argument("invalid dispatch driver");
        if (driver.state != DriverState::Available && driver.state != DriverState::Reserved)
            throw std::invalid_argument("invalid driver state");
        drivers_.push_back(Slot{driver.id, driver.position});
        states_[i].store(static_cast<unsigned char>(driver.state), std::memory_order_relaxed);
    }
    if (!wal_path.empty()) {
        wal_.reset(new WriteAheadLog(wal_path));
        const WalReplay replay = wal_->replay(drivers);
        for (size_t i = 0; i < drivers_.size(); ++i) {
            const bool reserved = replay.reserved_request.count(drivers_[i].id) != 0;
            states_[i].store(static_cast<unsigned char>(reserved ? DriverState::Reserved : DriverState::Available),
                std::memory_order_relaxed);
        }
    }
    threads_.reserve(workers);
    try {
        for (unsigned i = 0; i < workers; ++i)
            threads_.emplace_back([this] { worker(); });
    } catch (...) {
        shutdown();
        throw;
    }
}

Dispatcher::~Dispatcher() { shutdown(); }

size_t Dispatcher::index_of(uint32_t driver_id) const {
    const auto it = index_.find(driver_id);
    if (it == index_.end()) throw std::invalid_argument("unknown driver");
    return it->second;
}

void Dispatcher::submit(RideRequest request) {
    if (request.id == invalid) throw std::invalid_argument("invalid request id");
    std::lock_guard<std::mutex> lock(mu_);
    if (stopping_) throw std::runtime_error("dispatcher shut down");
    if (!request_ids_.insert(request.id).second) {
        results_.push_back(DispatchResult{Assignment{request.id}, DispatchStatus::Duplicate});
        return;
    }
    queue_.push(request);
    ++outstanding_;
    cv_.notify_one();
}

std::vector<DispatchResult> Dispatcher::drain() {
    std::unique_lock<std::mutex> lock(mu_);
    cv_.wait(lock, [&] { return outstanding_ == 0; });
    std::vector<DispatchResult> out;
    out.swap(results_);
    return out;
}

void Dispatcher::shutdown() {
    if (shutdown_started_.exchange(true)) {
        while (!joined_.load()) std::this_thread::yield();
        return;
    }
    {
        std::lock_guard<std::mutex> lock(mu_);
        stopping_ = true;
    }
    cv_.notify_all();
    for (auto& thread : threads_)
        if (thread.joinable()) thread.join();
    joined_.store(true);
}

void Dispatcher::testing_fail_next_appends(int count) {
    if (!wal_) throw std::invalid_argument("dispatcher has no wal");
    wal_->testing_fail_next_appends(count);
}

void Dispatcher::testing_arm(WalTestPoint point) {
    if (!wal_) throw std::invalid_argument("dispatcher has no wal");
    wal_->testing_arm(point);
}

bool Dispatcher::release(uint32_t driver_id) {
    const size_t index = index_of(driver_id);
    std::unique_lock<std::mutex> commit(commit_mu_, std::defer_lock);
    if (wal_) commit.lock();
    auto& state = states_[index];
    if (wal_) {
        if (state.load(std::memory_order_acquire) != static_cast<unsigned char>(DriverState::Reserved))
            return false;
        // Durable release is recorded while the driver is still Reserved, so a
        // crash cannot acknowledge the release without the record.
        wal_->append_release(driver_id);
    }
    unsigned char expected = static_cast<unsigned char>(DriverState::Reserved);
    return state.compare_exchange_strong(expected,
        static_cast<unsigned char>(DriverState::Available),
        std::memory_order_acq_rel, std::memory_order_acquire);
}

DriverState Dispatcher::state_of(uint32_t driver_id) const {
    return static_cast<DriverState>(
        states_[index_of(driver_id)].load(std::memory_order_acquire));
}

DispatchResult Dispatcher::match(const RideRequest& request) {
    if (!finite_point(request.pickup))
        return DispatchResult{Assignment{request.id}, DispatchStatus::Rejected};
    for (;;) {
        std::vector<Driver> view;
        view.reserve(drivers_.size());
        for (size_t i = 0; i < drivers_.size(); ++i) {
            const DriverState state = static_cast<DriverState>(
                states_[i].load(std::memory_order_acquire));
            view.push_back(Driver{drivers_[i].id, drivers_[i].position,
                state == DriverState::Available});
        }
        uint32_t chosen = invalid;
        try {
            chosen = nearest_driver(view, request.pickup);
        } catch (const std::exception&) {
            return DispatchResult{Assignment{request.id}, DispatchStatus::Rejected};
        }
        if (chosen == invalid)
            return DispatchResult{Assignment{request.id}, DispatchStatus::Unmatched};
        const size_t index = index_.at(chosen);
        std::unique_lock<std::mutex> commit(commit_mu_, std::defer_lock);
        if (wal_) commit.lock();
        unsigned char expected = static_cast<unsigned char>(DriverState::Available);
        // Linearization point. Exactly one compare-and-swap can change this
        // driver from Available to Reserved. The caller that observes success
        // owns the reservation; every other caller fails and retries.
        // With a WAL, the commit lock is held until fsync returns, and Assigned
        // is reported only after that durability boundary.
        if (!states_[index].compare_exchange_strong(expected,
                static_cast<unsigned char>(DriverState::Reserved),
                std::memory_order_acq_rel, std::memory_order_acquire))
            continue;
        const double pickup = distance(drivers_[index].position, request.pickup);
        if (!std::isfinite(pickup)) {
            unsigned char reserved = static_cast<unsigned char>(DriverState::Reserved);
            states_[index].compare_exchange_strong(reserved,
                static_cast<unsigned char>(DriverState::Available),
                std::memory_order_acq_rel, std::memory_order_acquire);
            return DispatchResult{Assignment{request.id}, DispatchStatus::Rejected};
        }
        try {
            // append_assignment returns only after fsync. Do not roll the
            // reservation back after this point: the record is durable.
            if (wal_) wal_->append_assignment(request.id, chosen);
            return DispatchResult{Assignment{request.id, chosen, pickup}, DispatchStatus::Assigned};
        } catch (const WalFailure& failure) {
            if (!failure.bytes_written()) {
                unsigned char reserved = static_cast<unsigned char>(DriverState::Reserved);
                states_[index].compare_exchange_strong(reserved,
                    static_cast<unsigned char>(DriverState::Available),
                    std::memory_order_acq_rel, std::memory_order_acquire);
            }
            return DispatchResult{Assignment{request.id}, DispatchStatus::PersistenceFailed};
        } catch (...) {
            unsigned char reserved = static_cast<unsigned char>(DriverState::Reserved);
            states_[index].compare_exchange_strong(reserved,
                static_cast<unsigned char>(DriverState::Available),
                std::memory_order_acq_rel, std::memory_order_acquire);
            return DispatchResult{Assignment{request.id}, DispatchStatus::Rejected};
        }
    }
}

void Dispatcher::worker() {
    for (;;) {
        RideRequest request;
        {
            std::unique_lock<std::mutex> lock(mu_);
            cv_.wait(lock, [&] { return stopping_ || !queue_.empty(); });
            if (queue_.empty()) return;
            request = queue_.front();
            queue_.pop();
        }
        DispatchResult result{Assignment{request.id}, DispatchStatus::Rejected};
        try {
            result = match(request);
        } catch (const std::exception&) {
            result = DispatchResult{Assignment{request.id}, DispatchStatus::Rejected};
        } catch (...) {
            result = DispatchResult{Assignment{request.id}, DispatchStatus::Rejected};
        }
        {
            std::lock_guard<std::mutex> lock(mu_);
            results_.push_back(result);
            --outstanding_;
        }
        cv_.notify_all();
    }
}
}
