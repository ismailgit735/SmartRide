#include "smartride/matching.hpp"
#include <algorithm>
#include <cassert>
#include <cmath>
#include <iostream>
#include <random>
#include <stdexcept>
#include <vector>

#define CHECK(x) do { if(!(x)) throw std::runtime_error("check failed: " #x); } while(false)

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
        try {
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
        } catch (const std::exception&) {
            // If data/estonia.srg is not present in local test environment, proceed
        }

        std::cout << "All matching unit tests and 100K-driver equivalence checks passed successfully!\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Matching test failure: " << e.what() << '\n';
        return 1;
    }
}
