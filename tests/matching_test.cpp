#include "smartride/matching.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <filesystem>
#include <limits>
#include <unordered_set>
#include <random>
#include <stdexcept>
#include <vector>

#define CHECK(x) do { if(!(x)) throw std::runtime_error("check failed: " #x); } while(false)

// Independent replay checks the greedy rule and reservation state, in addition
// to comparing the production brute-force oracle with SpatialGrid.
void compare_batch(const std::vector<sr::Driver>& initial,
                   const std::vector<sr::RideRequest>& requests, double cell = 0) {
    auto brute = initial, spatial = initial, replay = initial;
    auto expected = sr::greedy_batch_brute_force(brute, requests);
    auto actual = sr::greedy_batch_spatial(spatial, requests, cell);
    CHECK(actual.size() == requests.size());
    std::unordered_set<uint32_t> used;
    for (size_t i = 0; i < requests.size(); ++i) {
        CHECK(actual[i].request_id == requests[i].id);
        if (actual[i].driver_id != expected[i].driver_id ||
            actual[i].pickup_distance != expected[i].pickup_distance) {
            std::cerr << "Batch request index=" << i << " expected="
                      << expected[i].driver_id << " actual=" << actual[i].driver_id << '\n';
            CHECK(false);
        }
        uint32_t winner = sr::invalid;
        double best = sr::inf;
        for (const auto& d : replay) if (d.available) {
            double cost = sr::distance(d.position, requests[i].pickup);
            if (cost < best || (cost == best && d.id < winner)) {
                best = cost; winner = d.id;
            }
        }
        CHECK(actual[i].driver_id == winner);
        CHECK(actual[i].pickup_distance == best);
        if (winner != sr::invalid) {
            CHECK(used.insert(winner).second);
            for (auto& d : replay) if (d.id == winner) d.available = false;
        }
    }
    for (size_t i = 0; i < initial.size(); ++i) {
        CHECK(brute[i].available == spatial[i].available);
        CHECK(spatial[i].available == replay[i].available);
        CHECK(spatial[i].id == initial[i].id);
        CHECK(spatial[i].position.x == initial[i].position.x);
        CHECK(spatial[i].position.y == initial[i].position.y);
    }
}

void batch_tests() {
    using sr::Driver; using sr::RideRequest;
    compare_batch({}, {});
    compare_batch({}, {{9, {0, 0}}});
    compare_batch({{77, {1, 2}, true}}, {});
    compare_batch({{77, {1, 2}, false}}, {{9, {0, 0}}});
    std::vector<Driver> drivers = {{80, {0, 0}, true}, {7, {0, 0}, true},
        {99, {10, 0}, false}, {41, {10, 0}, true}};
    std::vector<RideRequest> requests = {{15, {0, 0}}, {3, {0, 0}},
        {200, {0, 0}}, {4, {10, 0}}, {6, {-100, 100}}};
    compare_batch(drivers, requests, 5);
    auto state = drivers;
    auto result = sr::greedy_batch_spatial(state, requests);
    CHECK(result[0].driver_id == 7);
    CHECK(result[1].driver_id == 80);
    CHECK(result[2].driver_id == 41);
    CHECK(result[3].driver_id == sr::invalid && std::isinf(result[3].pickup_distance));
    auto again = sr::greedy_batch_spatial(state, requests);
    for (const auto& a : again) CHECK(a.driver_id == sr::invalid);
    std::reverse(drivers.begin(), drivers.end());
    auto shuffled = sr::greedy_batch_spatial(drivers, requests);
    for (size_t i = 0; i < result.size(); ++i) CHECK(result[i].driver_id == shuffled[i].driver_id);

    // Supplied order, not request ID, determines who gets the nearest driver.
    std::vector<Driver> order_drivers = {{8, {0, 0}, true}, {2, {10, 0}, true}};
    std::vector<RideRequest> ordered = {{100, {1, 0}}, {1, {2, 0}}};
    auto first_state = order_drivers, second_state = order_drivers;
    auto first = sr::greedy_batch_spatial(first_state, ordered);
    std::reverse(ordered.begin(), ordered.end());
    auto second = sr::greedy_batch_spatial(second_state, ordered);
    CHECK(first[0].request_id == 100 && first[0].driver_id == 8);
    CHECK(second[0].request_id == 1 && second[0].driver_id == 8);
    // Incremental batches reserve drivers across calls.
    state = order_drivers;
    CHECK(sr::greedy_batch_spatial(state, {{0, {0, 0}}})[0].driver_id == 8);
    CHECK(sr::greedy_batch_spatial(state, {{1, {0, 0}}})[0].driver_id == 2);

    for (double cell : {0.0, 5.0, 10.0, 100.0}) {
        compare_batch({{90, {-10, 0}, true}, {2, {10, 0}, true},
                       {33, {0, 10}, true}, {44, {0, -10}, true}},
                      {{0, {0, 0}}, {1, {5, 5}}, {2, {-1000, 0}},
                       {3, {1000, 1000}}, {4, {0, 0}}}, cell);
    }
    auto rejected = [](std::vector<Driver> d, const std::vector<RideRequest>& r) {
        for (bool spatial : {false, true}) {
            auto copy = d;
            bool threw = false;
            try {
                if (spatial) sr::greedy_batch_spatial(copy, r);
                else sr::greedy_batch_brute_force(copy, r);
            } catch (const std::invalid_argument&) { threw = true; }
            CHECK(threw);
            for (size_t i = 0; i < d.size(); ++i) CHECK(copy[i].available == d[i].available);
        }
    };
    rejected({{1, {0, 0}, true}, {1, {1, 1}, false}}, {{1, {0, 0}}});
    rejected({{sr::invalid, {0, 0}, true}}, {});
    rejected(order_drivers, {{1, {0, 0}}, {1, {1, 1}}});
    rejected(order_drivers, {{1, {0, 0}}, {sr::invalid, {1, 1}}});
    for (double bad : {sr::inf, -sr::inf, std::numeric_limits<double>::quiet_NaN()}) {
        rejected({{1, {bad, 0}, true}}, {});
        rejected({{1, {0, bad}, false}}, {});
        rejected(order_drivers, {{1, {0, 0}}, {2, {bad, 0}}});
        rejected(order_drivers, {{1, {0, 0}}, {2, {0, bad}}});
    }
    for (double cell : {-1.0, sr::inf, std::numeric_limits<double>::quiet_NaN(), 1e-300}) {
        auto copy = order_drivers;
        bool threw = false;
        try { sr::greedy_batch_spatial(copy, ordered, cell); }
        catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
        CHECK(copy[0].available && copy[1].available);
    }
    // Far finite queries must be clamped before conversion to grid integers.
    compare_batch(order_drivers, {{0, {1e100, -1e100}}});
    {
        std::vector<Driver> extreme = {{1, {-1e200, -1e200}, true},
                                       {2, {1e200, 1e200}, true}};
        bool threw = false;
        try { sr::greedy_batch_spatial(extreme, {{0, {0, 0}}}); }
        catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
        CHECK(extreme[0].available && extreme[1].available);
    }
    for (uint64_t seed = 0; seed < 60; ++seed) {
        std::mt19937_64 rng(seed);
        std::vector<Driver> d;
        size_t n = rng() % 100;
        for (size_t i = 0; i < n; ++i)
            d.push_back({uint32_t(i * 13 + 7), {double(rng()%21)-10, double(rng()%21)-10}, rng()%3 != 0});
        std::shuffle(d.begin(), d.end(), rng);
        std::vector<RideRequest> r;
        for (size_t i = 0; i < 120; ++i)
            r.push_back({uint32_t(1000-i), {double(rng()%41)-20, double(rng()%41)-20}});
        for (double cell : {0.0, 2.0, 10.0}) {
            try { compare_batch(d, r, cell); }
            catch (...) { std::cerr << "Batch seed=" << seed << " cell=" << cell << '\n'; throw; }
        }
    }
    std::mt19937_64 rng(1234567);
    std::vector<Driver> large;
    for (size_t i = 0; i < 100000; ++i)
        large.push_back({uint32_t(i*3+1), {double(rng()%100000), double(rng()%100000)}, rng()%5 != 0});
    std::vector<RideRequest> large_requests;
    for (uint32_t i = 0; i < 100; ++i)
        large_requests.push_back({i, {double(rng()%120000)-10000, double(rng()%120000)-10000}});
    compare_batch(large, large_requests);
}

int main() {
    try {
        // 1. Empty driver set
        {
            std::vector<sr::Driver> empty;
            sr::SpatialGrid grid(empty);
            CHECK(grid.nearest_driver({0, 0}) == sr::invalid);
            CHECK(sr::nearest_driver(empty, {0, 0}) == sr::invalid);
        }

        // 2. Single driver
        {
            std::vector<sr::Driver> single = {{42, {15.0, 25.0}, true}};
            sr::SpatialGrid grid(single, 10.0);
            CHECK(grid.nearest_driver({15.0, 25.0}) == 42);
            CHECK(grid.nearest_driver({0.0, 0.0}) == 42);
            CHECK(grid.nearest_driver({1000.0, 1000.0}) == 42);
            single[0].available = false;
            CHECK(grid.nearest_driver({15.0, 25.0}) == sr::invalid);
        }

        // 3. All unavailable
        {
            std::vector<sr::Driver> d = {{0, {10, 10}, false}, {1, {20, 20}, false}};
            sr::SpatialGrid grid(d);
            CHECK(grid.nearest_driver({10, 10}) == sr::invalid);
            CHECK(sr::nearest_driver(d, {10, 10}) == sr::invalid);
        }

        // 4. Dynamic availability changes without grid re-indexing
        {
            std::vector<sr::Driver> d = {{0, {10, 0}, true}, {1, {20, 0}, true}};
            sr::SpatialGrid grid(d, 5.0);
            CHECK(grid.nearest_driver({11, 0}) == 0);
            d[0].available = false;
            CHECK(grid.nearest_driver({11, 0}) == 1);
            d[1].available = false;
            CHECK(grid.nearest_driver({11, 0}) == sr::invalid);
            d[0].available = true;
            CHECK(grid.nearest_driver({11, 0}) == 0);
        }

        // 5. Deterministic tie-breaking: lower ID wins on equal distance
        {
            std::vector<sr::Driver> d = {{5, {0, 10}, true}, {2, {0, -10}, true}, {8, {10, 0}, true}};
            sr::SpatialGrid grid(d, 5.0);
            CHECK(grid.nearest_driver({0, 0}) == 2);
            CHECK(sr::nearest_driver(d, {0, 0}) == 2);
        }

        // 6. Adjacent cell nearest driver (query cell has a driver, but neighbor cell is closer)
        {
            // Cell 0: driver 0 at (0, 0)
            // Cell 1: driver 1 at (5.1, 0)
            // Query at (4.9, 0): distance to driver 0 is 4.9; distance to driver 1 is 0.2
            std::vector<sr::Driver> d = {{0, {0, 0}, true}, {1, {5.1, 0}, true}};
            sr::SpatialGrid grid(d, 5.0);
            CHECK(grid.nearest_driver({4.9, 0}) == 1);
            CHECK(sr::nearest_driver(d, {4.9, 0}) == 1);
        }

        // 7. Out of bounds queries in all directions
        {
            std::vector<sr::Driver> d = {{0, {100, 100}, true}, {1, {200, 200}, true}};
            sr::SpatialGrid grid(d, 50.0);
            std::vector<sr::Point> out_points = {
                {-1000, -1000}, {-1000, 150}, {-1000, 3000},
                {150, -1000}, {150, 3000},
                {3000, -1000}, {3000, 150}, {3000, 3000}
            };
            for (auto p : out_points) {
                uint32_t expected = sr::nearest_driver(d, p);
                uint32_t actual = grid.nearest_driver(p);
                CHECK(expected == actual);
            }
        }

        // 8. Cell boundary and corner queries
        {
            std::vector<sr::Driver> d = {
                {0, {0.0, 0.0}, true},
                {1, {10.0, 0.0}, true},
                {2, {0.0, 10.0}, true},
                {3, {10.0, 10.0}, true}
            };
            sr::SpatialGrid grid(d, 10.0);
            std::vector<sr::Point> test_points = {
                {0.0, 0.0}, {10.0, 0.0}, {0.0, 10.0}, {10.0, 10.0},
                {5.0, 0.0}, {5.0, 5.0}, {5.0, 10.0}, {0.0, 5.0}, {10.0, 5.0}
            };
            for (auto p : test_points) {
                CHECK(grid.nearest_driver(p) == sr::nearest_driver(d, p));
            }
        }

        // 9. Randomized equivalence tests comparing SpatialGrid against brute force
        std::mt19937_64 rng(999111);
        for (size_t n : {10, 100, 1000, 10000, 100000}) {
            std::vector<sr::Driver> drivers;
            drivers.reserve(n);
            for (size_t i = 0; i < n; ++i) {
                double x = double(rng() % 100000);
                double y = double(rng() % 100000);
                bool avail = (rng() % 5 != 0); // 80% available
                drivers.push_back({static_cast<uint32_t>(i), {x, y}, avail});
            }
            sr::SpatialGrid grid(drivers);
            size_t query_count = (n <= 10000) ? 100 : 25;
            for (size_t q = 0; q < query_count; ++q) {
                double qx = -10000.0 + double(rng() % 120000);
                double qy = -10000.0 + double(rng() % 120000);
                uint32_t ref = sr::nearest_driver(drivers, {qx, qy});
                uint32_t got = grid.nearest_driver({qx, qy});
                CHECK(ref == got);
            }
        }

        // 10. Real Estonia road network driver simulation if graph file exists
        if (std::filesystem::exists("data/estonia.srg")) {
            sr::Graph g = sr::Graph::load("data/estonia.srg");
            auto drivers = sr::simulate_drivers(100000, g, 42);
            sr::SpatialGrid grid(drivers);
            CHECK(drivers.size() == 100000);
            for (size_t q = 0; q < 20; ++q) {
                sr::Point p = g.points[rng() % g.points.size()];
                uint32_t ref = sr::nearest_driver(drivers, p);
                uint32_t got = grid.nearest_driver(p);
                CHECK(ref == got);
            }
        } else {
            std::cout << "SKIP: optional Estonia fixture unavailable in working directory\n";
        }
        batch_tests();

        std::cout << "All matching unit tests and 100K-driver equivalence checks passed successfully!\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Matching test failure: " << e.what() << '\n';
        return 1;
    }
}
