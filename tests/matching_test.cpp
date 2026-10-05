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
#include <unordered_map>
#include <unordered_set>

#define CHECK(x) do { if(!(x)) throw std::runtime_error("check failed: " #x); } while(false)

template<class Call>
void rejects_invalid(Call call) {
    bool rejected = false;
    try { call(); } catch (const std::invalid_argument&) { rejected = true; }
    CHECK(rejected);
}

void numeric_boundary_tests() {
    using sr::Driver; using sr::RideRequest;
    const std::vector<Driver> ordinary = {{10, {0, 0}, true}, {20, {1, 1}, false}};
    for (double bad : {sr::inf, -sr::inf, std::numeric_limits<double>::quiet_NaN()}) {
        for (sr::Point p : {sr::Point{bad, 0}, sr::Point{0, bad}}) {
            for (bool empty : {false, true}) {
                auto drivers = empty ? std::vector<Driver>{} : ordinary;
                sr::SpatialGrid grid(drivers);
                rejects_invalid([&] { (void)grid.nearest_driver(p); });
                rejects_invalid([&] { (void)sr::nearest_driver(drivers, p); });
            }
            // Both the first point and an unavailable later point are checked.
            for (size_t i : {size_t(0), size_t(1)}) {
                auto drivers = ordinary;
                drivers[i].position = p;
                rejects_invalid([&] { sr::SpatialGrid grid(drivers); });
                rejects_invalid([&] { (void)sr::nearest_driver(drivers, {0, 0}); });
            }
        }
    }
    for (double bad : {-1.0, sr::inf, -sr::inf, std::numeric_limits<double>::quiet_NaN()}) {
        rejects_invalid([&] { sr::SpatialGrid grid(ordinary, bad); });
        rejects_invalid([&] { sr::SpatialGrid grid({}, bad); });
    }
    rejects_invalid([&] { sr::SpatialGrid grid(ordinary, 1e-300); });
    // Finite endpoints whose subtraction overflows must fail before conversion.
    const std::vector<Driver> wide = {{1, {-1e308, 0}, true}, {2, {1e308, 0}, true}};
    rejects_invalid([&] { sr::SpatialGrid grid(wide); });
    rejects_invalid([&] { sr::SpatialGrid grid(wide, 1e308); });

    // Each batch implementation must leave the entire state unchanged. The
    // greedy cases reserve driver 10 for the first request before the second
    // request encounters overflow, exercising rollback rather than just setup.
    auto reject_batch = [](const std::vector<Driver>& initial,
                           const std::vector<RideRequest>& requests) {
        for (int algorithm = 0; algorithm < 3; ++algorithm) {
            auto state = initial;
            rejects_invalid([&] {
                if (algorithm == 0) (void)sr::greedy_batch_brute_force(state, requests);
                if (algorithm == 1) (void)sr::greedy_batch_spatial(state, requests);
                if (algorithm == 2) (void)sr::hungarian_batch(state, requests);
            });
            for (size_t i = 0; i < state.size(); ++i) {
                CHECK(state[i].available == initial[i].available);
                CHECK(state[i].id == initial[i].id);
                CHECK(state[i].position.x == initial[i].position.x);
                CHECK(state[i].position.y == initial[i].position.y);
            }
        }
    };
    for (sr::Point far : {sr::Point{1e308, 0}, sr::Point{0, 1e308}}) {
        std::vector<Driver> drivers = {{10, far, true}, {20, far, true}, {30, far, false}};
        sr::Point opposite{-far.x, -far.y};
        reject_batch(drivers, {{1, far}, {2, opposite}});
        sr::SpatialGrid grid(drivers);
        rejects_invalid([&] { (void)grid.nearest_driver(opposite); });
        rejects_invalid([&] { (void)sr::nearest_driver(drivers, opposite); });
    }
    // Subtractions can be finite while hypot itself overflows.
    reject_batch({{1, {0, 0}, true}}, {{1, {1.3e308, 1.3e308}}});

    // All individual costs are finite, but the Hungarian dual objective can
    // overflow. It must reject, not loop or commit a partial assignment.
    std::vector<Driver> costly = {{1, {0, 0}, true}, {2, {0, 0}, true}};
    rejects_invalid([&] { (void)sr::hungarian_batch(costly,
        {{1, {1e308, 0}}, {2, {1e308, 0}}}); });
    CHECK(costly[0].available && costly[1].available);

    // Large but representable distances remain supported, including clamping
    // an overflowing cell quotient before integer conversion.
    for (int algorithm = 0; algorithm < 3; ++algorithm) {
        std::vector<Driver> state = {{7, {0, 0}, true}};
        const std::vector<RideRequest> request = {{9, {1e308, 0}}};
        auto result = algorithm == 0 ? sr::greedy_batch_brute_force(state, request)
                    : algorithm == 1 ? sr::greedy_batch_spatial(state, request, 1e-300)
                    : sr::hungarian_batch(state, request);
        CHECK(result[0].driver_id == 7 && result[0].pickup_distance == 1e308);
        CHECK(!state[0].available);
    }
    std::vector<Driver> unavailable = {{1, {1e308, 0}, false}};
    sr::SpatialGrid grid(unavailable);
    CHECK(grid.nearest_driver({-1e308, 0}) == sr::invalid);
    CHECK(sr::nearest_driver(unavailable, {-1e308, 0}) == sr::invalid);
    CHECK(sr::hungarian_batch(unavailable, {{1, {-1e308, 0}}})[0].driver_id == sr::invalid);
}

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

struct Oracle {
    double total = sr::inf;
    std::vector<uint32_t> driver_ids;
    int optima = 0;
};

void search_oracle(const std::vector<sr::Driver>& drivers,
                   const std::vector<size_t>& available,
                   const std::vector<sr::RideRequest>& requests,
                   std::vector<char>& used, std::vector<uint32_t>& cur,
                   size_t row, size_t matched, double sum, Oracle& best) {
    const size_t n = requests.size();
    const size_t m = available.size();
    const size_t need = std::min(n, m);
    if (matched > need || matched + (n - row) < need) return;
    if (row == n) {
        if (matched != need) return;
        if (sum < best.total) {
            best.total = sum;
            best.driver_ids = cur;
            best.optima = 1;
        } else if (sum == best.total) {
            ++best.optima;
        }
        return;
    }
    if (matched + (n - row) - 1 >= need) {
        const auto saved = cur[row];
        cur[row] = sr::invalid;
        search_oracle(drivers, available, requests, used, cur, row + 1, matched, sum, best);
        cur[row] = saved;
    }
    for (size_t j = 0; j < m; ++j) {
        if (used[j]) continue;
        used[j] = 1;
        cur[row] = drivers[available[j]].id;
        const double dist = sr::distance(drivers[available[j]].position, requests[row].pickup);
        search_oracle(drivers, available, requests, used, cur, row + 1, matched + 1, sum + dist, best);
        used[j] = 0;
    }
}

Oracle exhaustive_assignment(const std::vector<sr::Driver>& drivers,
                             const std::vector<sr::RideRequest>& requests) {
    std::vector<size_t> available;
    for (size_t i = 0; i < drivers.size(); ++i)
        if (drivers[i].available) available.push_back(i);
    Oracle best;
    std::vector<char> used(available.size(), 0);
    std::vector<uint32_t> cur(requests.size(), sr::invalid);
    search_oracle(drivers, available, requests, used, cur, 0, 0, 0.0, best);
    if (requests.empty()) {
        best.total = 0;
        best.optima = 1;
    }
    return best;
}

double assignment_total(const std::vector<sr::Assignment>& result) {
    double total = 0;
    std::unordered_set<uint32_t> used;
    for (const auto& a : result) {
        if (a.driver_id == sr::invalid) {
            CHECK(std::isinf(a.pickup_distance));
            continue;
        }
        CHECK(used.insert(a.driver_id).second);
        CHECK(std::isfinite(a.pickup_distance));
        total += a.pickup_distance;
    }
    return total;
}

void check_hungarian_result(const std::vector<sr::Driver>& initial,
                            std::vector<sr::Driver>& state,
                            const std::vector<sr::RideRequest>& requests,
                            const std::vector<sr::Assignment>& result) {
    CHECK(result.size() == requests.size());
    std::unordered_set<uint32_t> used;
    std::unordered_map<uint32_t, size_t> index;
    for (size_t i = 0; i < initial.size(); ++i) {
        index[initial[i].id] = i;
        CHECK(state[i].id == initial[i].id);
        CHECK(state[i].position.x == initial[i].position.x);
        CHECK(state[i].position.y == initial[i].position.y);
    }
    for (size_t i = 0; i < requests.size(); ++i) {
        CHECK(result[i].request_id == requests[i].id);
        if (result[i].driver_id == sr::invalid) {
            CHECK(std::isinf(result[i].pickup_distance));
            continue;
        }
        CHECK(used.insert(result[i].driver_id).second);
        auto it = index.find(result[i].driver_id);
        CHECK(it != index.end());
        CHECK(initial[it->second].available);
        CHECK(!state[it->second].available);
        CHECK(result[i].pickup_distance ==
              sr::distance(initial[it->second].position, requests[i].pickup));
    }
    for (size_t i = 0; i < initial.size(); ++i) {
        if (!initial[i].available) CHECK(!state[i].available);
        else if (used.count(initial[i].id)) CHECK(!state[i].available);
        else CHECK(state[i].available);
    }
}

void hungarian_tests() {
    using sr::Driver; using sr::RideRequest;
    auto run = [](std::vector<Driver> drivers, const std::vector<RideRequest>& requests,
                  sr::HungarianOptions options = {}) {
        auto initial = drivers;
        auto result = sr::hungarian_batch(drivers, requests, options);
        check_hungarian_result(initial, drivers, requests, result);
        return result;
    };

    CHECK(run({}, {}).empty());
    auto none = run({}, {{9, {0, 0}}});
    CHECK(none.size() == 1 && none[0].driver_id == sr::invalid && std::isinf(none[0].pickup_distance));
    CHECK(run({{77, {1, 2}, true}}, {}).empty());
    auto busy = run({{77, {1, 2}, false}}, {{9, {0, 0}}});
    CHECK(busy[0].driver_id == sr::invalid);

    // Greedy is suboptimal: request order grabs the globally more valuable driver.
    std::vector<Driver> subopt_d = {{10, {4, 0}, true}, {20, {20, 0}, true}};
    std::vector<RideRequest> subopt_r = {{0, {5, 0}}, {1, {0, 0}}};
    auto greedy_state = subopt_d, hungarian_state = subopt_d;
    auto greedy = sr::greedy_batch_brute_force(greedy_state, subopt_r);
    auto hungarian = sr::hungarian_batch(hungarian_state, subopt_r);
    CHECK(greedy[0].driver_id == 10 && greedy[1].driver_id == 20);
    CHECK(hungarian[0].driver_id == 20 && hungarian[1].driver_id == 10);
    CHECK(assignment_total(hungarian) < assignment_total(greedy));
    CHECK(assignment_total(hungarian) == 19.0);

    // Rectangular: more requests than drivers.
    auto short_supply = run({{5, {0, 0}, true}, {9, {10, 0}, false}},
                            {{1, {0, 0}}, {2, {10, 0}}, {3, {1, 0}}});
    CHECK(short_supply[0].driver_id == 5);
    CHECK(short_supply[1].driver_id == sr::invalid);
    CHECK(short_supply[2].driver_id == sr::invalid);

    // Rectangular: more drivers than requests.
    auto extra = run({{8, {100, 0}, true}, {3, {1, 0}, true}, {4, {2, 0}, true}},
                     {{10, {0, 0}}});
    CHECK(extra[0].driver_id == 3);

    // Shortage of available drivers.
    auto depleted = run({{1, {0, 0}, true}, {2, {1, 0}, true}},
                        {{8, {0, 0}}, {9, {1, 0}}, {10, {2, 0}}});
    CHECK(depleted[0].driver_id != sr::invalid);
    CHECK(depleted[1].driver_id != sr::invalid);
    CHECK(depleted[2].driver_id == sr::invalid);
    CHECK(depleted[0].driver_id != depleted[1].driver_id);

    // Unavailable drivers are excluded and do not consume the cap.
    std::vector<Driver> mixed;
    for (uint32_t i = 0; i < 513; ++i)
        mixed.push_back({i, {double(i), 0}, i == 512});
    auto only_last = run(mixed, {{0, {512, 0}}});
    CHECK(only_last[0].driver_id == 512);

    // Ties / coincident positions: canonical ID order, request rows.
    auto tied = run({{30, {0, 0}, true}, {10, {0, 0}, true}, {20, {0, 0}, true}},
                    {{2, {0, 0}}, {1, {0, 0}}});
    CHECK(tied[0].driver_id == 10 && tied[1].driver_id == 20);

    // Arbitrary IDs and shuffled storage.
    std::vector<Driver> arbitrary = {{1000, {0, 1}, true}, {2, {10, 0}, true}, {50, {0, 0}, true}};
    std::vector<RideRequest> arbitrary_r = {{9, {0, 0}}, {8, {10, 0}}};
    auto first = run(arbitrary, arbitrary_r);
    std::reverse(arbitrary.begin(), arbitrary.end());
    auto shuffled = run(arbitrary, arbitrary_r);
    CHECK(first[0].driver_id == shuffled[0].driver_id);
    CHECK(first[1].driver_id == shuffled[1].driver_id);
    CHECK(first[0].driver_id == 50 && first[1].driver_id == 2);

    auto copy_a = arbitrary, copy_b = arbitrary;
    auto once = sr::hungarian_batch(copy_a, arbitrary_r);
    auto twice = sr::hungarian_batch(copy_b, arbitrary_r);
    CHECK(once[0].driver_id == twice[0].driver_id && once[1].driver_id == twice[1].driver_id);

    auto rejected = [](std::vector<Driver> d, const std::vector<RideRequest>& r,
                       sr::HungarianOptions options = {}) {
        auto copy = d;
        bool threw = false;
        try { sr::hungarian_batch(copy, r, options); }
        catch (const std::invalid_argument&) { threw = true; }
        CHECK(threw);
        for (size_t i = 0; i < d.size(); ++i) CHECK(copy[i].available == d[i].available);
    };
    rejected({{1, {0, 0}, true}, {1, {1, 1}, false}}, {{1, {0, 0}}});
    rejected({{sr::invalid, {0, 0}, true}}, {});
    rejected({{1, {0, 0}, true}}, {{1, {0, 0}}, {1, {1, 1}}});
    rejected({{1, {0, 0}, true}}, {{sr::invalid, {1, 1}}});
    for (double bad : {sr::inf, -sr::inf, std::numeric_limits<double>::quiet_NaN()}) {
        rejected({{1, {bad, 0}, true}}, {});
        rejected({{1, {0, bad}, false}}, {});
        rejected({{1, {0, 0}, true}}, {{2, {bad, 0}}});
        rejected({{1, {0, 0}, true}}, {{2, {0, bad}}});
    }
    rejected({{1, {0, 0}, true}}, {}, {0, 512});
    rejected({{1, {0, 0}, true}}, {}, {64, 0});
    rejected({{1, {0, 0}, true}}, {}, {65, 512});
    rejected({{1, {0, 0}, true}}, {}, {64, 513});

    std::vector<RideRequest> too_many_requests;
    for (uint32_t i = 0; i < 65; ++i) too_many_requests.push_back({i, {0, 0}});
    rejected({{1, {0, 0}, true}}, too_many_requests);

    std::vector<Driver> too_many_available;
    for (uint32_t i = 0; i < 513; ++i) too_many_available.push_back({i, {0, 0}, true});
    rejected(too_many_available, {{0, {0, 0}}});

    std::vector<Driver> ok_cap;
    for (uint32_t i = 0; i < 512; ++i) ok_cap.push_back({i, {double(i), 0}, true});
    auto cap = run(ok_cap, {{0, {0, 0}}, {1, {10, 0}}});
    CHECK(cap[0].driver_id == 0 && cap[1].driver_id == 10);

    sr::HungarianOptions tight{2, 3};
    rejected({{1, {0, 0}, true}, {2, {1, 0}, true}, {3, {2, 0}, true}},
             {{0, {0, 0}}, {1, {1, 0}}, {2, {2, 0}}}, tight);
    auto tight_ok = run({{1, {0, 0}, true}, {2, {1, 0}, true}, {3, {2, 0}, false}},
                        {{0, {0, 0}}, {1, {1, 0}}}, tight);
    CHECK(tight_ok[0].driver_id != sr::invalid && tight_ok[1].driver_id != sr::invalid);

    for (uint64_t seed = 0; seed < 40; ++seed) {
        std::mt19937_64 rng(seed);
        std::vector<Driver> d;
        size_t n_drv = 1 + rng() % 6;
        for (size_t i = 0; i < n_drv; ++i)
            d.push_back({uint32_t(i * 17 + 3), {double(rng() % 21) - 10, double(rng() % 21) - 10}, rng() % 3 != 0});
        std::shuffle(d.begin(), d.end(), rng);
        std::vector<RideRequest> r;
        size_t n_req = rng() % 7;
        for (size_t i = 0; i < n_req; ++i)
            r.push_back({uint32_t(500 - i), {double(rng() % 21) - 10, double(rng() % 21) - 10}});
        auto initial = d;
        auto result = sr::hungarian_batch(d, r);
        check_hungarian_result(initial, d, r, result);
        auto oracle = exhaustive_assignment(initial, r);
        if (n_req == 0) continue;
        CHECK(assignment_total(result) == oracle.total);
        size_t matched = 0;
        for (const auto& a : result) if (a.driver_id != sr::invalid) ++matched;
        size_t avail = 0;
        for (const auto& drv : initial) if (drv.available) ++avail;
        CHECK(matched == std::min(n_req, avail));
        if (oracle.optima == 1) {
            for (size_t i = 0; i < r.size(); ++i)
                CHECK(result[i].driver_id == oracle.driver_ids[i]);
        }
        auto shuffled_d = initial;
        std::shuffle(shuffled_d.begin(), shuffled_d.end(), rng);
        auto shuffled_state = shuffled_d;
        auto again = sr::hungarian_batch(shuffled_state, r);
        for (size_t i = 0; i < r.size(); ++i) {
            CHECK(again[i].driver_id == result[i].driver_id);
            CHECK(again[i].pickup_distance == result[i].pickup_distance);
        }
        auto greedy_d = initial;
        auto greedy_r = sr::greedy_batch_brute_force(greedy_d, r);
        size_t greedy_matched = 0;
        for (const auto& a : greedy_r) if (a.driver_id != sr::invalid) ++greedy_matched;
        CHECK(matched == greedy_matched);
        CHECK(assignment_total(result) <= assignment_total(greedy_r));
    }

    for (uint64_t seed = 0; seed < 20; ++seed) {
        std::mt19937_64 rng(1000 + seed);
        std::vector<Driver> d;
        for (size_t i = 0; i < 24; ++i)
            d.push_back({uint32_t(i * 3 + 1), {double(rng() % 101), double(rng() % 101)}, rng() % 4 != 0});
        std::shuffle(d.begin(), d.end(), rng);
        std::vector<RideRequest> r;
        for (uint32_t i = 0; i < 16; ++i)
            r.push_back({i, {double(rng() % 101), double(rng() % 101)}});
        auto initial = d, greedy_d = d;
        auto result = sr::hungarian_batch(d, r);
        auto greedy_r = sr::greedy_batch_brute_force(greedy_d, r);
        check_hungarian_result(initial, d, r, result);
        CHECK(assignment_total(result) <= assignment_total(greedy_r));
        auto shuffled_d = initial;
        std::reverse(shuffled_d.begin(), shuffled_d.end());
        auto shuffled_res = sr::hungarian_batch(shuffled_d, r);
        for (size_t i = 0; i < r.size(); ++i)
            CHECK(shuffled_res[i].driver_id == result[i].driver_id);
    }
}

int main() {
    try {
        numeric_boundary_tests();
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

        // 10. Estonia fixture. CMake passes the source root so CTest's build-directory
        // working directory cannot hide a missing or misplaced map.
#ifdef SMARTRIDE_SOURCE_DIR
        const std::filesystem::path estonia = std::filesystem::path(SMARTRIDE_SOURCE_DIR) / "data" / "estonia.srg";
#else
        const std::filesystem::path estonia = std::filesystem::path("data") / "estonia.srg";
#endif
        if (!std::filesystem::is_regular_file(estonia))
            throw std::runtime_error("Estonia fixture not found: " + estonia.string());
        sr::Graph g = sr::Graph::load(estonia.string());
        auto drivers = sr::simulate_drivers(100000, g, 42);
        sr::SpatialGrid grid(drivers);
        CHECK(drivers.size() == 100000);
        CHECK(g.points.size() > 1000000);
        CHECK(g.edges.size() > 1000000);
        for (size_t q = 0; q < 20; ++q) {
            sr::Point p = g.points[rng() % g.points.size()];
            uint32_t ref = sr::nearest_driver(drivers, p);
            uint32_t got = grid.nearest_driver(p);
            CHECK(ref == got);
        }
        std::cout << "Estonia matching fixture passed: " << estonia.string()
                  << " nodes=" << g.points.size() << " directed_edges=" << g.edges.size() << '\n';
        batch_tests();
        hungarian_tests();

        std::cout << "All matching unit tests and 100K-driver equivalence checks passed successfully!\n";
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Matching test failure: " << e.what() << '\n';
        return 1;
    }
}
