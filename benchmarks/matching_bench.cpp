#include "smartride/matching.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

struct BenchmarkResult {
    size_t driver_count;
    size_t query_count;
    double grid_build_ms;
    double brute_p50_us;
    double brute_p95_us;
    double brute_p99_us;
    double brute_qps;
    double grid_p50_us;
    double grid_p95_us;
    double grid_p99_us;
    double grid_qps;
    double speedup_qps;
    double speedup_p50;
    bool verified_correct;
};

BenchmarkResult run_benchmark(const sr::Graph& g, size_t driver_count, size_t query_count, uint64_t driver_seed, uint64_t query_seed) {
    std::cout << "\n=== Running benchmark with " << driver_count << " drivers and " << query_count << " queries ===" << std::endl;

    // 1. Generate identical drivers
    auto drivers = sr::simulate_drivers(driver_count, g, driver_seed);

    // 2. Generate identical query points
    std::mt19937_64 q_rng(query_seed);
    std::vector<sr::Point> queries;
    queries.reserve(query_count);
    for (size_t i = 0; i < query_count; ++i) {
        queries.push_back(g.points[q_rng() % g.points.size()]);
    }

    // 3. Measure SpatialGrid construction time (averaged over 5 runs)
    double build_total_ms = 0;
    constexpr int build_iters = 5;
    for (int it = 0; it < build_iters; ++it) {
        auto t0 = std::chrono::high_resolution_clock::now();
        sr::SpatialGrid grid(drivers);
        auto t1 = std::chrono::high_resolution_clock::now();
        build_total_ms += std::chrono::duration<double, std::milli>(t1 - t0).count();
    }
    double grid_build_ms = build_total_ms / build_iters;
    sr::SpatialGrid grid(drivers);

    // 4. Warm-up phase
    for (size_t i = 0; i < std::min<size_t>(100, query_count); ++i) {
        volatile uint32_t r1 = sr::nearest_driver(drivers, queries[i]);
        volatile uint32_t r2 = grid.nearest_driver(queries[i]);
        (void)r1; (void)r2;
    }

    // 5. Measure Brute Force
    std::vector<double> brute_latencies_us(query_count);
    std::vector<uint32_t> brute_results(query_count);
    auto brute_start_total = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < query_count; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        brute_results[i] = sr::nearest_driver(drivers, queries[i]);
        auto t1 = std::chrono::high_resolution_clock::now();
        brute_latencies_us[i] = std::chrono::duration<double, std::micro>(t1 - t0).count();
    }
    auto brute_end_total = std::chrono::high_resolution_clock::now();
    double brute_total_sec = std::chrono::duration<double>(brute_end_total - brute_start_total).count();
    double brute_qps = static_cast<double>(query_count) / brute_total_sec;

    // 6. Measure SpatialGrid
    std::vector<double> grid_latencies_us(query_count);
    std::vector<uint32_t> grid_results(query_count);
    auto grid_start_total = std::chrono::high_resolution_clock::now();
    for (size_t i = 0; i < query_count; ++i) {
        auto t0 = std::chrono::high_resolution_clock::now();
        grid_results[i] = grid.nearest_driver(queries[i]);
        auto t1 = std::chrono::high_resolution_clock::now();
        grid_latencies_us[i] = std::chrono::duration<double, std::micro>(t1 - t0).count();
    }
    auto grid_end_total = std::chrono::high_resolution_clock::now();
    double grid_total_sec = std::chrono::duration<double>(grid_end_total - grid_start_total).count();
    double grid_qps = static_cast<double>(query_count) / grid_total_sec;

    // 7. Verify 100% equivalence on every single query
    bool verified_correct = true;
    for (size_t i = 0; i < query_count; ++i) {
        if (brute_results[i] != grid_results[i]) {
            std::cerr << "ERROR: Mismatch at query " << i
                      << ": brute=" << brute_results[i]
                      << " grid=" << grid_results[i] << std::endl;
            verified_correct = false;
            throw std::runtime_error("Benchmark equivalence verification failed!");
        }
    }

    // 8. Calculate latency percentiles
    std::sort(brute_latencies_us.begin(), brute_latencies_us.end());
    std::sort(grid_latencies_us.begin(), grid_latencies_us.end());

    auto p50_idx = static_cast<size_t>(0.50 * query_count);
    auto p95_idx = static_cast<size_t>(0.95 * query_count);
    auto p99_idx = static_cast<size_t>(0.99 * query_count);

    double brute_p50 = brute_latencies_us[p50_idx];
    double brute_p95 = brute_latencies_us[p95_idx];
    double brute_p99 = brute_latencies_us[p99_idx];

    double grid_p50 = grid_latencies_us[p50_idx];
    double grid_p95 = grid_latencies_us[p95_idx];
    double grid_p99 = grid_latencies_us[p99_idx];

    double speedup_qps = grid_qps / brute_qps;
    double speedup_p50 = (grid_p50 > 0) ? (brute_p50 / grid_p50) : 0.0;

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "  Grid construction time: " << grid_build_ms << " ms (" << grid.cols() << "x" << grid.rows() << " cells, cell_size=" << grid.cell_size() << " m)\n"
              << "  Brute-force: p50=" << brute_p50 << " us | p95=" << brute_p95 << " us | p99=" << brute_p99 << " us | " << brute_qps << " queries/sec\n"
              << "  SpatialGrid: p50=" << grid_p50 << " us | p95=" << grid_p95 << " us | p99=" << grid_p99 << " us | " << grid_qps << " queries/sec\n"
              << "  Speedup (QPS): " << speedup_qps << "x | Speedup (p50): " << speedup_p50 << "x\n"
              << "  Correctness: 100% identical (" << query_count << "/" << query_count << " matches)\n";

    return {
        driver_count,
        query_count,
        grid_build_ms,
        brute_p50,
        brute_p95,
        brute_p99,
        brute_qps,
        grid_p50,
        grid_p95,
        grid_p99,
        grid_qps,
        speedup_qps,
        speedup_p50,
        verified_correct
    };
}

int main(int argc, char** argv) {
    try {
#ifdef SMARTRIDE_SOURCE_DIR
        const std::string default_map = std::string(SMARTRIDE_SOURCE_DIR) + "/data/estonia.srg";
#else
        const std::string default_map = "data/estonia.srg";
#endif
        const std::string map_path = (argc > 1) ? argv[1] : default_map;
        if (!std::filesystem::is_regular_file(map_path))
            throw std::runtime_error("road graph not found: " + map_path);
        std::cout << "Loading road network from: " << map_path << "..." << std::endl;
        sr::Graph g = sr::Graph::load(map_path);
        if (g.points.empty() || g.edges.empty())
            throw std::runtime_error("road graph is empty: " + map_path);
        std::cout << "Loaded map: " << g.points.size() << " nodes, " << g.edges.size() << " directed edges.\n";

        std::vector<size_t> driver_scales = {1000, 10000, 100000};
        constexpr size_t query_count = 2000;
        constexpr uint64_t driver_seed = 42;
        constexpr uint64_t query_seed = 1234567;

        std::vector<BenchmarkResult> results;
        for (size_t n : driver_scales) {
            results.push_back(run_benchmark(g, n, query_count, driver_seed, query_seed));
        }

        // Print final summary table
        std::ostringstream out;
        out << "# SpatialGrid vs Brute-Force Nearest-Driver Benchmark\n\n";
        out << "Road network: " << g.points.size() << " nodes, " << g.edges.size() << " directed edges\n";
        out << "Workload: " << query_count << " random queries per scale; seeds: driver=42, query=1234567\n";
        out << "Timestamp: " << __DATE__ << " " << __TIME__ << "\n\n";
        out << "| Drivers | Grid Build (ms) | Brute p50 (us) | Brute p95 (us) | Brute p99 (us) | Brute (QPS) | Grid p50 (us) | Grid p95 (us) | Grid p99 (us) | Grid (QPS) | QPS Speedup | Correctness |\n";
        out << "|---------|-----------------|----------------|----------------|----------------|-------------|---------------|---------------|---------------|------------|-------------|-------------|\n";
        for (const auto& r : results) {
            out << "| " << std::setw(7) << r.driver_count
                << " | " << std::setw(15) << std::fixed << std::setprecision(2) << r.grid_build_ms
                << " | " << std::setw(14) << r.brute_p50_us
                << " | " << std::setw(14) << r.brute_p95_us
                << " | " << std::setw(14) << r.brute_p99_us
                << " | " << std::setw(11) << std::fixed << std::setprecision(0) << r.brute_qps
                << " | " << std::setw(13) << std::fixed << std::setprecision(2) << r.grid_p50_us
                << " | " << std::setw(13) << r.grid_p95_us
                << " | " << std::setw(13) << r.grid_p99_us
                << " | " << std::setw(10) << std::fixed << std::setprecision(0) << r.grid_qps
                << " | " << std::setw(10) << std::fixed << std::setprecision(1) << r.speedup_qps << "x"
                << " | " << (r.verified_correct ? "100% match" : "FAIL") << " |\n";
        }

        std::cout << "\n" << out.str() << std::endl;

        const std::string result_path = argc > 2 ? argv[2] : "results/matching-benchmark.txt";
        std::ofstream fout(result_path);
        if (!fout) throw std::runtime_error("could not write " + result_path);
        fout << out.str();
        if (!fout) throw std::runtime_error("failed while writing " + result_path);
        std::cout << "Saved raw benchmark results to " << result_path << '\n';

        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Benchmark failure: " << e.what() << '\n';
        return 1;
    }
}
