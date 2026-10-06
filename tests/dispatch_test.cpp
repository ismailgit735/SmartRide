#include "smartride/dispatch.hpp"
#include <cmath>
#include <iostream>
#include <limits>
#include <random>
#include <stdexcept>
#include <unordered_map>
#include <unordered_set>

#define CHECK(x) do { if (!(x)) throw std::runtime_error("check failed: " #x); } while (false)

std::vector<sr::DispatchDriver> fleet(size_t n, sr::Point origin = {0, 0}, double step = 10) {
    std::vector<sr::DispatchDriver> drivers;
    drivers.reserve(n);
    for (size_t i = 0; i < n; ++i)
        drivers.push_back({uint32_t(i + 1), {origin.x + step * double(i), origin.y}, sr::DriverState::Available});
    return drivers;
}

void check_unique_reservations(const sr::Dispatcher& dispatcher,
                                const std::vector<sr::DispatchResult>& results) {
    std::unordered_set<uint32_t> used;
    size_t assigned = 0;
    size_t reserved = 0;
    for (uint32_t id = 1; id < 100000; ++id) {
        try {
            if (dispatcher.state_of(id) == sr::DriverState::Reserved) ++reserved;
        } catch (const std::invalid_argument&) {
            break;
        }
    }
    for (const auto& result : results) {
        if (result.status != sr::DispatchStatus::Assigned) {
            CHECK(result.assignment.driver_id == sr::invalid);
            continue;
        }
        CHECK(result.assignment.driver_id != sr::invalid);
        CHECK(std::isfinite(result.assignment.pickup_distance));
        CHECK(used.insert(result.assignment.driver_id).second);
        CHECK(dispatcher.state_of(result.assignment.driver_id) == sr::DriverState::Reserved);
        ++assigned;
    }
    CHECK(assigned == used.size());
    CHECK(assigned <= reserved);
}

void single_request() {
    sr::Dispatcher dispatcher(fleet(1), 1);
    dispatcher.submit({7, {0, 0}});
    auto results = dispatcher.drain();
    CHECK(results.size() == 1);
    CHECK(results[0].status == sr::DispatchStatus::Assigned);
    CHECK(results[0].assignment.driver_id == 1);
    CHECK(results[0].assignment.pickup_distance == 0);
    CHECK(dispatcher.state_of(1) == sr::DriverState::Reserved);
}

void several_requests() {
    sr::Dispatcher dispatcher(fleet(3), 2);
    dispatcher.submit({1, {0, 0}});
    dispatcher.submit({2, {10, 0}});
    dispatcher.submit({3, {20, 0}});
    auto results = dispatcher.drain();
    CHECK(results.size() == 3);
    check_unique_reservations(dispatcher, results);
    std::unordered_map<uint32_t, uint32_t> by_request;
    for (const auto& result : results) {
        CHECK(result.status == sr::DispatchStatus::Assigned);
        by_request[result.assignment.request_id] = result.assignment.driver_id;
    }
    CHECK(by_request[1] == 1);
    CHECK(by_request[2] == 2);
    CHECK(by_request[3] == 3);
}

void more_requests_than_drivers() {
    sr::Dispatcher dispatcher(fleet(2), 4);
    for (uint32_t i = 0; i < 5; ++i) dispatcher.submit({i, {double(i), 0}});
    auto results = dispatcher.drain();
    CHECK(results.size() == 5);
    size_t assigned = 0, unmatched = 0;
    for (const auto& result : results) {
        if (result.status == sr::DispatchStatus::Assigned) ++assigned;
        else { CHECK(result.status == sr::DispatchStatus::Unmatched); ++unmatched; }
    }
    CHECK(assigned == 2);
    CHECK(unmatched == 3);
    check_unique_reservations(dispatcher, results);
}

void one_driver_stampede() {
    sr::Dispatcher dispatcher(fleet(1), 8);
    for (uint32_t i = 0; i < 200; ++i) dispatcher.submit({i, {double(i), 1}});
    auto results = dispatcher.drain();
    CHECK(results.size() == 200);
    size_t assigned = 0;
    uint32_t winner = sr::invalid;
    for (const auto& result : results) {
        if (result.status == sr::DispatchStatus::Assigned) {
            ++assigned;
            winner = result.assignment.driver_id;
        } else {
            CHECK(result.status == sr::DispatchStatus::Unmatched);
        }
    }
    CHECK(assigned == 1);
    CHECK(winner == 1);
    check_unique_reservations(dispatcher, results);
}

void duplicate_ids() {
    sr::Dispatcher dispatcher(fleet(2), 2);
    dispatcher.submit({5, {0, 0}});
    dispatcher.submit({5, {10, 0}});
    auto results = dispatcher.drain();
    CHECK(results.size() == 2);
    size_t assigned = 0, duplicate = 0;
    for (const auto& result : results) {
        CHECK(result.assignment.request_id == 5);
        if (result.status == sr::DispatchStatus::Assigned) ++assigned;
        if (result.status == sr::DispatchStatus::Duplicate) {
            ++duplicate;
            CHECK(result.assignment.driver_id == sr::invalid);
        }
    }
    CHECK(assigned == 1);
    CHECK(duplicate == 1);
}

void empty_pool_and_unavailable() {
    sr::Dispatcher empty({}, 2);
    empty.submit({1, {0, 0}});
    auto none = empty.drain();
    CHECK(none.size() == 1);
    CHECK(none[0].status == sr::DispatchStatus::Unmatched);

    auto drivers = fleet(2);
    drivers[0].state = sr::DriverState::Reserved;
    sr::Dispatcher dispatcher(std::move(drivers), 2);
    dispatcher.submit({1, {0, 0}});
    dispatcher.submit({2, {10, 0}});
    auto results = dispatcher.drain();
    size_t assigned = 0;
    for (const auto& result : results) {
        if (result.status != sr::DispatchStatus::Assigned) continue;
        CHECK(result.assignment.driver_id == 2);
        ++assigned;
    }
    CHECK(assigned == 1);
    CHECK(dispatcher.state_of(1) == sr::DriverState::Reserved);
}

void rejected_request_keeps_working() {
    sr::Dispatcher dispatcher(fleet(1), 2);
    dispatcher.submit({1, {std::numeric_limits<double>::quiet_NaN(), 0}});
    dispatcher.submit({2, {0, 0}});
    auto results = dispatcher.drain();
    CHECK(results.size() == 2);
    bool rejected = false, assigned = false;
    for (const auto& result : results) {
        if (result.assignment.request_id == 1) {
            CHECK(result.status == sr::DispatchStatus::Rejected);
            CHECK(result.assignment.driver_id == sr::invalid);
            rejected = true;
        }
        if (result.assignment.request_id == 2) {
            CHECK(result.status == sr::DispatchStatus::Assigned);
            assigned = true;
        }
    }
    CHECK(rejected && assigned);
}

void shutdown_drains_and_rejects() {
    sr::Dispatcher dispatcher(fleet(4), 3);
    for (uint32_t i = 0; i < 20; ++i) dispatcher.submit({i, {double(i), 0}});
    dispatcher.shutdown();
    auto results = dispatcher.drain();
    CHECK(results.size() == 20);
    check_unique_reservations(dispatcher, results);
    bool threw = false;
    try { dispatcher.submit({100, {0, 0}}); }
    catch (const std::runtime_error&) { threw = true; }
    CHECK(threw);
    dispatcher.shutdown();
}

void repeated_batches() {
    sr::Dispatcher dispatcher(fleet(2), 4);
    dispatcher.submit({1, {0, 0}});
    dispatcher.submit({2, {10, 0}});
    auto first = dispatcher.drain();
    CHECK(first.size() == 2);
    check_unique_reservations(dispatcher, first);

    dispatcher.submit({3, {0, 0}});
    dispatcher.submit({4, {10, 0}});
    auto second = dispatcher.drain();
    CHECK(second.size() == 2);
    for (const auto& result : second) CHECK(result.status == sr::DispatchStatus::Unmatched);

    CHECK(dispatcher.release(1));
    CHECK(dispatcher.release(2));
    CHECK(!dispatcher.release(1));
    dispatcher.submit({5, {0, 0}});
    dispatcher.submit({6, {10, 0}});
    auto third = dispatcher.drain();
    CHECK(third.size() == 2);
    check_unique_reservations(dispatcher, third);
    for (const auto& result : third) CHECK(result.status == sr::DispatchStatus::Assigned);
}

void stress() {
    constexpr size_t drivers_n = 64;
    constexpr uint32_t requests_n = 4000;
    sr::Dispatcher dispatcher(fleet(drivers_n, {0, 0}, 1), 8);
    std::mt19937_64 rng(918273);
    for (uint32_t i = 0; i < requests_n; ++i) {
        sr::Point pickup{double(rng() % 80), double(rng() % 5)};
        dispatcher.submit({i, pickup});
    }
    auto results = dispatcher.drain();
    CHECK(results.size() == requests_n);
    std::unordered_set<uint32_t> winners;
    size_t assigned = 0;
    for (const auto& result : results) {
        CHECK(result.assignment.request_id < requests_n);
        if (result.status == sr::DispatchStatus::Assigned) {
            CHECK(winners.insert(result.assignment.driver_id).second);
            CHECK(dispatcher.state_of(result.assignment.driver_id) == sr::DriverState::Reserved);
            ++assigned;
        } else {
            CHECK(result.status == sr::DispatchStatus::Unmatched);
            CHECK(result.assignment.driver_id == sr::invalid);
        }
    }
    CHECK(assigned == winners.size());
    CHECK(assigned <= drivers_n);
    CHECK(assigned == drivers_n);
    size_t reserved = 0;
    for (uint32_t id = 1; id <= drivers_n; ++id)
        reserved += dispatcher.state_of(id) == sr::DriverState::Reserved;
    CHECK(reserved == assigned);
}

int main() {
    try {
        single_request();
        several_requests();
        more_requests_than_drivers();
        one_driver_stampede();
        duplicate_ids();
        empty_pool_and_unavailable();
        rejected_request_keeps_working();
        shutdown_drains_and_rejects();
        repeated_batches();
        stress();
        std::cout << "dispatch tests passed\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Dispatch test failure: " << e.what() << '\n';
        return 1;
    }
}
