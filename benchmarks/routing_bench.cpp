#include "smartride/routing.hpp"
#include <algorithm>
#include <chrono>
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

struct Sample {
    std::vector<double> ms;
    size_t reachable = 0;
};

int main(int argc, char** argv) {
    try {
#ifdef SMARTRIDE_SOURCE_DIR
        const std::string default_map = std::string(SMARTRIDE_SOURCE_DIR) + "/data/estonia.srg";
        const std::string default_result = std::string(SMARTRIDE_SOURCE_DIR) + "/results/routing-benchmark.txt";
#else
        const std::string default_map = "data/estonia.srg";
        const std::string default_result = "results/routing-benchmark.txt";
#endif
        const std::string path = argc > 1 ? argv[1] : default_map;
        const std::string result_path = argc > 2 ? argv[2] : default_result;
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

        auto router_start = Clock::now();
        const sr::Router router(graph);
        const double router_ms = std::chrono::duration<double, std::milli>(Clock::now() - router_start).count();

        std::cerr << "Checking " << pairs.size() << " pairs against Dijkstra before timing\n";
        size_t reachable = 0;
        for (size_t i = 0; i < pairs.size(); ++i) {
            const sr::Node s = pairs[i].first;
            const sr::Node t = pairs[i].second;
            const sr::Route reference = sr::dijkstra(graph, s, t);
            const sr::Route astar = router.astar(s, t);
            const sr::Route bidirectional = router.bidirectional_astar(s, t);
            if (!sr::valid_route(graph, s, t, reference) ||
                !sr::valid_route(graph, s, t, astar) ||
                !sr::valid_route(graph, s, t, bidirectional) ||
                !distances_agree(reference.distance, astar.distance) ||
                !distances_agree(reference.distance, bidirectional.distance)) {
                throw std::runtime_error("routing mismatch at pair " + std::to_string(i) +
                    " s=" + std::to_string(s) + " t=" + std::to_string(t));
            }
            if (std::isfinite(reference.distance)) ++reachable;
            if ((i + 1) % 10 == 0)
                std::cerr << "  checked " << (i + 1) << "/" << pairs.size() << '\n';
        }

        const size_t warmup = std::min(kWarmup, pairs.size());
        for (int algorithm = 0; algorithm < 3; ++algorithm)
            for (size_t i = 0; i < warmup; ++i) {
                if (algorithm == 0) (void)sr::dijkstra(graph, pairs[i].first, pairs[i].second);
                else if (algorithm == 1) (void)router.astar(pairs[i].first, pairs[i].second);
                else (void)router.bidirectional_astar(pairs[i].first, pairs[i].second);
            }

        std::vector<Sample> timed(3);
        for (size_t i = 0; i < pairs.size(); ++i) {
            const int first = int(i % 3);
            for (int step = 0; step < 3; ++step) {
                const int algorithm = (first + step) % 3;
                auto start = Clock::now();
                sr::Route route = algorithm == 0 ? sr::dijkstra(graph, pairs[i].first, pairs[i].second)
                    : algorithm == 1 ? router.astar(pairs[i].first, pairs[i].second)
                                     : router.bidirectional_astar(pairs[i].first, pairs[i].second);
                timed[algorithm].ms.push_back(
                    std::chrono::duration<double, std::milli>(Clock::now() - start).count());
                if (std::isfinite(route.distance)) ++timed[algorithm].reachable;
            }
            if ((i + 1) % 10 == 0)
                std::cerr << "  timed " << (i + 1) << "/" << pairs.size() << '\n';
        }
        const char* names[] = {"Dijkstra", "A*", "Bidirectional A*"};
        std::ostringstream report;
        report << std::fixed << std::setprecision(3);
        report << "# Routing benchmark: Dijkstra, A*, bidirectional A*\n\n"
               << "Map: " << path << "; nodes=" << graph.points.size()
               << "; directed_edges=" << graph.edges.size() << "\n"
               << "Compiler: " << __VERSION__ << "; C++=" << __cplusplus << "\n"
               << "Seed: " << kPairSeed << "; distinct uniform node pairs via mt19937_64 modulo node count.\n"
               << "Queries: " << pairs.size() << "; reachable by Dijkstra: " << reachable << "\n"
               << "Correctness ran before timing. A* and bidirectional A* distances must match Dijkstra\n"
               << "within abs(delta) < 1e-7 * max(1, reference), and valid_route must accept every path.\n"
               << "Router construction is excluded from query latency: " << router_ms << " ms.\n"
               << "Each algorithm is warmed with the first " << warmup
               << " pairs. Warmup is excluded from the sample.\n"
               << "Timed calls rotate which algorithm runs first. Latency is the query call only.\n"
               << "Percentile index is floor(p * n). Queries/sec uses the sum of per-query latencies.\n"
               << "Successful queries are those that passed the oracle. A mismatch aborts before this file is written.\n\n"
               << "| Algorithm | Queries | Successful | Reachable | p50 ms | p95 ms | p99 ms | Mean ms | Queries/sec |\n"
               << "|---|---|---|---|---|---|---|---|---|\n";
        for (int algorithm = 0; algorithm < 3; ++algorithm) {
            const auto& ms = timed[algorithm].ms;
            if (ms.size() != pairs.size())
                throw std::runtime_error("incomplete timing sample");
            const double total = std::accumulate(ms.begin(), ms.end(), 0.0);
            const double mean = total / double(ms.size());
            report << "| " << names[algorithm]
                   << " | " << ms.size()
                   << " | " << ms.size()
                   << " | " << timed[algorithm].reachable
                   << " | " << percentile(ms, 0.50)
                   << " | " << percentile(ms, 0.95)
                   << " | " << percentile(ms, 0.99)
                   << " | " << mean
                   << " | " << (1000.0 * ms.size() / total)
                   << " |\n";
        }

        std::ofstream out(result_path);
        if (!out) throw std::runtime_error("could not write " + result_path);
        out << report.str();
        if (!out) throw std::runtime_error("failed while writing " + result_path);
        std::cout << report.str();
        std::cerr << "Wrote " << result_path << '\n';
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Routing benchmark failed: " << e.what() << '\n';
        return 1;
    }
}
