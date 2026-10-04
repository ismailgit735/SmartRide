#include "smartride/matching.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>
#include <unordered_set>

using Clock = std::chrono::steady_clock;
using Results = std::vector<sr::Assignment>;
struct Measurement { Results assignments; double ms; };

Measurement measure_greedy(std::vector<sr::Driver>& drivers,
                           const std::vector<sr::RideRequest>& requests) {
    auto start = Clock::now();
    auto assignments = sr::greedy_batch_brute_force(drivers, requests);
    auto stop = Clock::now();
    return {std::move(assignments), std::chrono::duration<double, std::milli>(stop-start).count()};
}

Measurement measure_hungarian(std::vector<sr::Driver>& drivers,
                              const std::vector<sr::RideRequest>& requests) {
    auto start = Clock::now();
    auto assignments = sr::hungarian_batch(drivers, requests);
    auto stop = Clock::now();
    return {std::move(assignments), std::chrono::duration<double, std::milli>(stop-start).count()};
}

void verify(const Measurement& greedy, const Measurement& hungarian,
            const std::vector<sr::Driver>& greedy_state,
            const std::vector<sr::Driver>& hungarian_state) {
    if (greedy.assignments.size() != hungarian.assignments.size())
        throw std::runtime_error("assignment count mismatch");
    std::unordered_set<uint32_t> greedy_used, hungarian_used;
    double greedy_total = 0, hungarian_total = 0;
    size_t greedy_matched = 0, hungarian_matched = 0;
    for (size_t i = 0; i < greedy.assignments.size(); ++i) {
        const auto& g = greedy.assignments[i];
        const auto& h = hungarian.assignments[i];
        if (g.request_id != h.request_id)
            throw std::runtime_error("request order mismatch");
        if (g.driver_id != sr::invalid) {
            if (!greedy_used.insert(g.driver_id).second)
                throw std::runtime_error("greedy double assignment");
            if (!std::isfinite(g.pickup_distance))
                throw std::runtime_error("greedy nonfinite pickup");
            greedy_total += g.pickup_distance;
            ++greedy_matched;
        } else if (!std::isinf(g.pickup_distance)) {
            throw std::runtime_error("greedy unmatched distance");
        }
        if (h.driver_id != sr::invalid) {
            if (!hungarian_used.insert(h.driver_id).second)
                throw std::runtime_error("hungarian double assignment");
            if (!std::isfinite(h.pickup_distance))
                throw std::runtime_error("hungarian nonfinite pickup");
            hungarian_total += h.pickup_distance;
            ++hungarian_matched;
        } else if (!std::isinf(h.pickup_distance)) {
            throw std::runtime_error("hungarian unmatched distance");
        }
    }
    if (greedy_matched != hungarian_matched)
        throw std::runtime_error("cardinality mismatch");
    if (hungarian_total > greedy_total)
        throw std::runtime_error("hungarian total pickup exceeded greedy");
    size_t greedy_left = 0, hungarian_left = 0;
    for (size_t i = 0; i < greedy_state.size(); ++i) {
        greedy_left += greedy_state[i].available;
        hungarian_left += hungarian_state[i].available;
    }
    if (greedy_left != hungarian_left)
        throw std::runtime_error("availability cardinality mismatch");
}

double percentile(std::vector<double> values, double p) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size()-1, size_t(p*values.size()))];
}

void run(const sr::Graph& graph, size_t drivers, size_t batch_size) {
    const auto initial = sr::simulate_drivers(drivers, graph, 42);
    std::mt19937_64 rng(1234567);
    std::vector<sr::RideRequest> requests;
    for (uint32_t i = 0; i < 2000; ++i)
        requests.push_back({i, graph.points[rng()%graph.points.size()]});
    auto a = initial, b = initial;
    std::vector<sr::RideRequest> warm(requests.begin(),
        requests.begin()+std::min<size_t>(batch_size, requests.size()));
    auto warm_g = measure_greedy(a, warm);
    auto warm_h = measure_hungarian(b, warm);
    verify(warm_g, warm_h, a, b);
    a = initial; b = initial;
    std::vector<double> greedy_ms, hungarian_ms;
    size_t matched = 0, unmatched = 0, verified = 0;
    double greedy_distance = 0, hungarian_distance = 0;
    size_t batches = 0;
    for (size_t offset = 0; offset < requests.size(); offset += batch_size) {
        a = initial; b = initial;
        auto end = std::min(requests.size(), offset + batch_size);
        std::vector<sr::RideRequest> batch(requests.begin()+offset, requests.begin()+end);
        Measurement greedy, hungarian;
        if ((offset/batch_size)%2 == 0) {
            greedy = measure_greedy(a, batch); hungarian = measure_hungarian(b, batch);
        } else {
            hungarian = measure_hungarian(b, batch); greedy = measure_greedy(a, batch);
        }
        verify(greedy, hungarian, a, b);
        greedy_ms.push_back(greedy.ms); hungarian_ms.push_back(hungarian.ms);
        ++batches;
        verified += batch.size();
        for (size_t i = 0; i < greedy.assignments.size(); ++i) {
            if (greedy.assignments[i].driver_id == sr::invalid) ++unmatched;
            else { ++matched; greedy_distance += greedy.assignments[i].pickup_distance; }
            if (hungarian.assignments[i].driver_id != sr::invalid)
                hungarian_distance += hungarian.assignments[i].pickup_distance;
        }
    }
    double greedy_total = std::accumulate(greedy_ms.begin(), greedy_ms.end(), 0.0);
    double hungarian_total = std::accumulate(hungarian_ms.begin(), hungarian_ms.end(), 0.0);
    std::cout << std::fixed << std::setprecision(3)
        << "| " << batch_size << " | " << drivers << " | " << batches
        << " | " << percentile(greedy_ms,.50) << "/" << percentile(greedy_ms,.95) << "/" << percentile(greedy_ms,.99)
        << " | " << percentile(hungarian_ms,.50) << "/" << percentile(hungarian_ms,.95) << "/" << percentile(hungarian_ms,.99)
        << " | " << matched << "/" << unmatched
        << " | " << (matched ? greedy_distance/matched : 0)
        << " | " << (matched ? hungarian_distance/matched : 0)
        << " | " << greedy_total/hungarian_total
        << " | " << verified << "/" << verified
        << " |" << std::endl;
}

int main(int argc, char** argv) {
    try {
        const std::string path = argc > 1 ? argv[1] : "data/estonia.srg";
        auto graph = sr::Graph::load(path);
        if (graph.points.empty()) throw std::runtime_error("empty benchmark graph");
        std::cout << "# Hungarian vs greedy batch matching\n\n"
            << "Map: " << path << "; nodes=" << graph.points.size() << "; directed_edges=" << graph.edges.size()
            << "\nCompiler: " << __VERSION__ << "; C++=" << __cplusplus
            << "\nSeeds: driver=42, request=1234567; 2000 requests sliced into reset batches.\n"
            << "Requests and drivers are simulated on OSM nodes; pickup cost is Euclidean meters.\n"
            << "Availability is restored before each batch. Hungarian uses all available drivers in the snapshot.\n"
            << "Times include complete API setup/teardown; exclude input reset, request slicing, and verification.\n"
            << "Percentiles are batch latency in milliseconds. Alternate algorithm timing order per batch.\n"
            << "Cardinality and Hungarian total pickup <= greedy total are checked outside timing.\n"
            << "Initial engineering target: p95 <= 10 ms for 32 requests x 256 drivers in Release.\n\n"
            << "| Requests | Available drivers | Samples | Greedy p50/p95/p99 ms | Hungarian p50/p95/p99 ms | Matched/unmatched | Greedy mean pickup m | Hungarian mean pickup m | Greedy/Hungarian time | Verified |\n"
            << "|---|---|---|---|---|---|---|---|---|---|\n";
        for (size_t batch : {8, 16, 32, 64})
            for (size_t count : {32, 128, 256, 512})
                run(graph, count, batch);
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Hungarian benchmark failed: " << e.what() << '\n';
        return 1;
    }
}
