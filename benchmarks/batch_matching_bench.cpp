#include "smartride/matching.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <stdexcept>
#include <string>

using Clock = std::chrono::steady_clock;
using Results = std::vector<sr::Assignment>;
struct Measurement { Results assignments; double ms; };

Measurement measure(std::vector<sr::Driver>& drivers,
                    const std::vector<sr::RideRequest>& requests, bool spatial) {
    auto start = Clock::now();
    auto assignments = spatial ? sr::greedy_batch_spatial(drivers, requests)
                               : sr::greedy_batch_brute_force(drivers, requests);
    auto stop = Clock::now();
    return {std::move(assignments), std::chrono::duration<double, std::milli>(stop-start).count()};
}

void verify(const Measurement& brute, const Measurement& spatial,
            const std::vector<sr::Driver>& a, const std::vector<sr::Driver>& b,
            size_t offset) {
    if (brute.assignments.size() != spatial.assignments.size())
        throw std::runtime_error("assignment count mismatch");
    for (size_t i = 0; i < brute.assignments.size(); ++i) {
        const auto& x = brute.assignments[i]; const auto& y = spatial.assignments[i];
        if (x.request_id != y.request_id || x.driver_id != y.driver_id ||
            x.pickup_distance != y.pickup_distance) {
            throw std::runtime_error("oracle mismatch at request " + std::to_string(offset+i) +
                " brute=" + std::to_string(x.driver_id) + " spatial=" + std::to_string(y.driver_id));
        }
    }
    for (size_t i = 0; i < a.size(); ++i)
        if (a[i].available != b[i].available) throw std::runtime_error("availability mismatch");
}

double percentile(std::vector<double> values, double p) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size()-1, size_t(p*values.size()))];
}

void run(const sr::Graph& graph, size_t count, size_t batch_size, bool depletion) {
    const auto initial = sr::simulate_drivers(count, graph, 42);
    std::mt19937_64 rng(1234567);
    std::vector<sr::RideRequest> requests;
    for (uint32_t i = 0; i < 2000; ++i)
        requests.push_back({i, graph.points[rng()%graph.points.size()]});
    // Standalone grid construction diagnostic; complete API timings below also
    // include validation, ID lookup construction, result allocation, and teardown.
    auto start = Clock::now();
    sr::SpatialGrid diagnostic(initial);
    double grid_ms = std::chrono::duration<double, std::milli>(Clock::now()-start).count();
    auto a = initial, b = initial;
    std::vector<sr::RideRequest> warm(requests.begin(), requests.begin()+std::min<size_t>(100, batch_size));
    auto warm_a = measure(a, warm, false), warm_b = measure(b, warm, true);
    verify(warm_a, warm_b, a, b, 0);
    a = initial; b = initial;
    std::vector<double> brute_ms, spatial_ms;
    size_t matched = 0, unmatched = 0;
    double total_distance = 0;
    for (size_t offset = 0; offset < requests.size(); offset += batch_size) {
        if (!depletion) { a = initial; b = initial; }
        auto end = std::min(requests.size(), offset + batch_size);
        std::vector<sr::RideRequest> batch(requests.begin()+offset, requests.begin()+end);
        Measurement brute, spatial;
        // Alternate algorithm timing order to reduce systematic order bias.
        if ((offset/batch_size)%2 == 0) {
            brute = measure(a, batch, false); spatial = measure(b, batch, true);
        } else {
            spatial = measure(b, batch, true); brute = measure(a, batch, false);
        }
        verify(brute, spatial, a, b, offset);
        brute_ms.push_back(brute.ms); spatial_ms.push_back(spatial.ms);
        for (const auto& assignment : brute.assignments) {
            if (assignment.driver_id == sr::invalid) ++unmatched;
            else { ++matched; total_distance += assignment.pickup_distance; }
        }
    }
    double brute_total = std::accumulate(brute_ms.begin(), brute_ms.end(), 0.0);
    double spatial_total = std::accumulate(spatial_ms.begin(), spatial_ms.end(), 0.0);
    std::cout << std::fixed << std::setprecision(3)
        << "| " << count << " | " << (depletion ? "deplete" : "reset") << " | " << batch_size
        << " | " << brute_ms.size() << " | " << grid_ms
        << " | " << percentile(brute_ms,.50) << "/" << percentile(brute_ms,.95) << "/" << percentile(brute_ms,.99)
        << " | " << percentile(spatial_ms,.50) << "/" << percentile(spatial_ms,.95) << "/" << percentile(spatial_ms,.99)
        << " | " << 2000000.0/brute_total << " | " << 2000000.0/spatial_total
        << " | " << brute_total/spatial_total << " | " << matched << "/" << unmatched
        << " | " << total_distance << " | " << (matched ? total_distance/matched : 0)
        << " | 2000/2000 |" << std::endl;
}

int main(int argc, char** argv) {
    try {
#ifdef SMARTRIDE_SOURCE_DIR
        const std::string default_map = std::string(SMARTRIDE_SOURCE_DIR) + "/data/estonia.srg";
#else
        const std::string default_map = "data/estonia.srg";
#endif
        const std::string path = argc > 1 ? argv[1] : default_map;
        if (!std::filesystem::is_regular_file(path))
            throw std::runtime_error("road graph not found: " + path);
        auto graph = sr::Graph::load(path);
        if (graph.points.empty() || graph.edges.empty())
            throw std::runtime_error("road graph is empty: " + path);
        std::cout << "# Greedy batch matching: brute-force oracle vs SpatialGrid\n\n"
            << "Map: " << path << "; nodes=" << graph.points.size() << "; directed_edges=" << graph.edges.size()
            << "\nCompiler: " << __VERSION__ << "; C++=" << __cplusplus
            << "\nSeeds: driver=42, request=1234567; 2000 requests per row.\n"
            << "Requests and drivers are simulated on OSM nodes; pickup cost is Euclidean meters.\n"
            << "reset: restore availability before each batch; deplete: carry reservations across batches.\n"
            << "Times include complete API setup/teardown; exclude input reset, request slicing, and verification.\n"
            << "Grid build is one separate construction sample. Percentiles are batch latency in milliseconds.\n"
            << "Large batches have few samples: p95/p99 are descriptive, not stable tail estimates.\n"
            << "All assignment IDs, distances, and final availability checked outside timing.\n\n"
            << "| Drivers | Mode | Batch | Samples | Grid build ms | Brute p50/p95/p99 ms | Spatial p50/p95/p99 ms | Brute req/s | Spatial req/s | Speedup | Matched/unmatched | Total pickup m | Mean pickup m | Verified |\n"
            << "|---|---|---|---|---|---|---|---|---|---|---|---|---|---|\n";
        for (size_t count : {1000, 10000, 100000}) {
            for (size_t batch : {1, 10, 100, 1000}) run(graph, count, batch, false);
            run(graph, count, 1000, true);
        }
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Batch benchmark failed: " << e.what() << '\n';
        return 1;
    }
}
