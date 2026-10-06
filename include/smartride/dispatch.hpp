#pragma once
#include "matching.hpp"
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <queue>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sr {
// Available -> Reserved is the only reservation transition.
// Reserved -> Available happens only through Dispatcher::release.
// No other transitions are valid.
enum class DriverState : unsigned char { Available = 0, Reserved = 1 };

enum class DispatchStatus { Assigned, Unmatched, Duplicate, Rejected };

struct DispatchResult {
    Assignment assignment;
    DispatchStatus status = DispatchStatus::Unmatched;
};

struct DispatchDriver {
    uint32_t id = invalid;
    Point position;
    DriverState state = DriverState::Available;
};

// Concurrent nearest-driver dispatch. Positions are fixed for the lifetime of
// the object. Request ids are unique for that lifetime; a repeated id is
// recorded as Duplicate and is not matched. shutdown() is idempotent and
// drains work already queued. Submit after shutdown throws runtime_error.
//
// Reservation linearization point: the successful compare-and-swap from
// Available to Reserved inside the worker. A failed exchange means another
// request already reserved that driver.
class Dispatcher {
public:
    Dispatcher(std::vector<DispatchDriver> drivers, unsigned workers);
    Dispatcher(const Dispatcher&) = delete;
    Dispatcher& operator=(const Dispatcher&) = delete;
    ~Dispatcher();

    void submit(RideRequest request);
    // Blocks until every submitted request has a result. Does not stop workers.
    std::vector<DispatchResult> drain();
    void shutdown();
    // Reserved -> Available. Returns false if the driver was not Reserved.
    bool release(uint32_t driver_id);
    DriverState state_of(uint32_t driver_id) const;

private:
    struct Slot {
        uint32_t id = invalid;
        Point position;
    };
    void worker();
    DispatchResult match(const RideRequest& request);
    size_t index_of(uint32_t driver_id) const;

    std::vector<Slot> drivers_;
    std::unique_ptr<std::atomic<unsigned char>[]> states_;
    std::unordered_map<uint32_t, size_t> index_;

    std::mutex mu_;
    std::condition_variable cv_;
    std::queue<RideRequest> queue_;
    std::vector<DispatchResult> results_;
    std::unordered_set<uint32_t> request_ids_;
    size_t outstanding_ = 0;
    bool stopping_ = false;
    std::atomic<bool> shutdown_started_{false};
    std::atomic<bool> joined_{false};
    std::vector<std::thread> threads_;
};
}
