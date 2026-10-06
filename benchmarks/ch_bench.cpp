#include "smartride/routing.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <random>
#include <sstream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>
#if defined(__APPLE__)
#include <mach/mach.h>
#include <sys/resource.h>
#endif

using Clock = std::chrono::steady_clock;

constexpr uint64_t kPairSeed = 918273;
constexpr size_t kQueries = 100;
constexpr size_t kWarmup = 10;

double percentile(std::vector<double> values, double p) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size() - 1, size_t(p * values.size()))];
}

bool distances_agree(double reference, double candidate) {
    if (!std::isfinite(reference) && !std::isfinite(candidate)) return true;
    return std::abs(reference - candidate) < 1e-7 * std::max(1.0, reference);
}

struct ProcessMemory {
    bool available = false;
    uint64_t resident = 0;
    uint64_t peak = 0;
};

ProcessMemory process_memory() {
    ProcessMemory measured;
#if defined(__APPLE__)
    mach_task_basic_info info;
    mach_msg_type_number_t count = MACH_TASK_BASIC_INFO_COUNT;
    const kern_return_t status = task_info(mach_task_self(), MACH_TASK_BASIC_INFO,
        reinterpret_cast<task_info_t>(&info), &count);
    rusage usage{};
    if (status == KERN_SUCCESS && getrusage(RUSAGE_SELF, &usage) == 0) {
        measured.available = true;
        measured.resident = info.resident_size;
        measured.peak = static_cast<uint64_t>(usage.ru_maxrss);
    }
#else
    (void)measured;
#endif
    return measured;
}

int main(int argc, char** argv) {
    try {
#ifdef SMARTRIDE_SOURCE_DIR
        const std::string default_map = std::string(SMARTRIDE_SOURCE_DIR) + "/data/estonia.srg";
        const std::string result_path = std::string(SMARTRIDE_SOURCE_DIR) + "/results/ch-benchmark.txt";
#else
        const std::string default_map = "data/estonia.srg";
        const std::string result_path = "results/ch-benchmark.txt";
#endif
        const std::string path = argc > 1 ? argv[1] : default_map;
        if (!std::filesystem::is_regular_file(path))
            throw std::runtime_error("road graph not found: " + path);
        std::cerr << "Loading road network from: " << path << '\n';
        const sr::Graph graph = sr::Graph::load(path);
        if (graph.points.size() < 2 || graph.edges.empty())
            throw std::runtime_error("road graph is empty: " + path);

        std::mt19937_64 rng(kPairSeed);
        std::vector<std::pair<sr::Node, sr::Node>> pairs;
        pairs.reserve(kQueries);
        while (pairs.size() < kQueries) {
            sr::Node s = sr::Node(rng() % graph.points.size());
            sr::Node t = sr::Node(rng() % graph.points.size());
            if (s != t) pairs.push_back({s, t});
        }

        const ProcessMemory before = process_memory();
        std::cerr << "Building contraction hierarchy\n";
        const auto preprocess_start = Clock::now();
        const sr::ContractionHierarchy hierarchy(graph);
        const double preprocess_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - preprocess_start).count();
        const ProcessMemory after = process_memory();
        const size_t shortcuts = hierarchy.shortcut_count();
        std::cerr << "Preprocessing finished in " << preprocess_ms << " ms; shortcuts=" << shortcuts << '\n';

        std::cerr << "Checking " << pairs.size() << " pairs against Dijkstra before timing\n";
        size_t reachable = 0;
        for (size_t i = 0; i < pairs.size(); ++i) {
            const sr::Node s = pairs[i].first;
            const sr::Node t = pairs[i].second;
            const sr::Route reference = sr::dijkstra(graph, s, t);
            const sr::Route contracted = hierarchy.route(s, t);
            if (!sr::valid_route(graph, s, t, reference) ||
                !sr::valid_route(graph, s, t, contracted) ||
                !distances_agree(reference.distance, contracted.distance)) {
                throw std::runtime_error("routing mismatch at pair " + std::to_string(i) +
                    " s=" + std::to_string(s) + " t=" + std::to_string(t));
            }
            if (std::isfinite(reference.distance)) ++reachable;
            if ((i + 1) % 10 == 0)
                std::cerr << "  checked " << (i + 1) << "/" << pairs.size() << '\n';
        }

        const size_t warmup = std::min(kWarmup, pairs.size());
        for (size_t i = 0; i < warmup; ++i)
            (void)hierarchy.route(pairs[i].first, pairs[i].second);

        std::vector<double> query_ms;
        query_ms.reserve(pairs.size());
        size_t timed_reachable = 0;
        for (size_t i = 0; i < pairs.size(); ++i) {
            const auto start = Clock::now();
            const sr::Route route = hierarchy.route(pairs[i].first, pairs[i].second);
            query_ms.push_back(std::chrono::duration<double, std::milli>(Clock::now() - start).count());
            if (std::isfinite(route.distance)) ++timed_reachable;
            if ((i + 1) % 10 == 0)
                std::cerr << "  timed " << (i + 1) << "/" << pairs.size() << '\n';
        }
        if (query_ms.size() != pairs.size())
            throw std::runtime_error("incomplete timing sample");
        const double total = std::accumulate(query_ms.begin(), query_ms.end(), 0.0);
        const double mean = total / double(query_ms.size());

        std::ostringstream report;
        report << std::fixed << std::setprecision(3);
        report << "# Contraction hierarchy benchmark\n\n"
               << "Map: " << path << "; nodes=" << graph.points.size()
               << "; directed_edges=" << graph.edges.size() << "\n"
               << "Compiler: " << __VERSION__ << "; C++=" << __cplusplus << "\n"
               << "Seed: " << kPairSeed << "; distinct uniform node pairs via mt19937_64 modulo node count.\n"
               << "Queries: " << pairs.size() << "; reachable by Dijkstra: " << reachable << "\n"
               << "Preprocessing wall time: " << preprocess_ms << " ms.\n"
               << "Shortcuts: " << shortcuts << ".\n";
        if (before.available && after.available) {
            report << "Process resident bytes before preprocessing: " << before.resident << ".\n"
                   << "Process resident bytes after preprocessing: " << after.resident << ".\n"
                   << "Process peak RSS bytes after preprocessing: " << after.peak
                   << " (Darwin getrusage ru_maxrss; this platform reports bytes).\n"
                   << "These figures are process RSS, not an isolated ContractionHierarchy byte count.\n";
        } else {
            report << "Process memory: unavailable.\n";
        }
        report << "ContractionHierarchy does not expose its own memory use.\n"
               << "Correctness ran before timing. CH distances must match Dijkstra\n"
               << "within abs(delta) < 1e-7 * max(1, reference), and valid_route must accept every path.\n"
               << "Unreachable pairs agree when both distances are non-finite and the paths are empty.\n"
               << "Warmup is the first " << warmup << " CH queries and is excluded from the sample.\n"
               << "Latency is the CH query call only. Percentile index is floor(p * n).\n"
               << "Queries/sec uses the sum of per-query latencies.\n"
               << "Successful queries are those that passed the oracle. A mismatch aborts before this file is written.\n\n"
               << "| Algorithm | Queries | Successful | Reachable | p50 ms | p95 ms | p99 ms | Mean ms | Queries/sec |\n"
               << "|---|---|---|---|---|---|---|---|---|\n"
               << "| Contraction hierarchy | " << query_ms.size()
               << " | " << query_ms.size()
               << " | " << timed_reachable
               << " | " << percentile(query_ms, 0.50)
               << " | " << percentile(query_ms, 0.95)
               << " | " << percentile(query_ms, 0.99)
               << " | " << mean
               << " | " << (1000.0 * query_ms.size() / total)
               << " |\n";

        std::ofstream out(result_path);
        if (!out) throw std::runtime_error("could not write " + result_path);
        out << report.str();
        if (!out) throw std::runtime_error("failed while writing " + result_path);
        std::cout << report.str();
        std::cerr << "Wrote " << result_path << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "CH benchmark failed: " << e.what() << '\n';
        return 1;
    }
}
