#include "smartride/dispatch.hpp"
#include <algorithm>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

using Clock = std::chrono::steady_clock;

constexpr unsigned kDrivers = 64;
constexpr unsigned kRounds = 8;
constexpr unsigned kSerialSamples = 64;
constexpr unsigned kSerialWarmup = 8;
constexpr unsigned kContentionDrivers = 4;
constexpr unsigned kContentionRequests = 64;
constexpr unsigned kContentionWorkers = 8;

double percentile(std::vector<double> values, double p) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size() - 1, size_t(p * values.size()))];
}

std::vector<sr::DispatchDriver> fleet(unsigned n) {
    std::vector<sr::DispatchDriver> drivers;
    drivers.reserve(n);
    for (unsigned i = 0; i < n; ++i)
        drivers.push_back({i + 1, {double(i), 0.0}, sr::DriverState::Available});
    return drivers;
}

void require_unique_assignments(const std::vector<sr::DispatchResult>& results,
                                unsigned expected_assigned) {
    std::unordered_set<uint32_t> drivers;
    std::unordered_set<uint32_t> requests;
    unsigned assigned = 0;
    for (const auto& result : results) {
        if (!requests.insert(result.assignment.request_id).second)
            throw std::runtime_error("duplicate request result");
        if (result.status == sr::DispatchStatus::Assigned) {
            if (result.assignment.driver_id == sr::invalid)
                throw std::runtime_error("assigned result has no driver");
            if (!drivers.insert(result.assignment.driver_id).second)
                throw std::runtime_error("duplicate driver assignment");
            ++assigned;
        } else if (result.status != sr::DispatchStatus::Unmatched) {
            throw std::runtime_error("unexpected dispatch status");
        }
    }
    if (assigned != expected_assigned)
        throw std::runtime_error("assigned count mismatch");
}

struct RoundTiming {
    double ms = 0;
    unsigned assigned = 0;
};

RoundTiming time_round(sr::Dispatcher& dispatcher, uint32_t first_request, unsigned count) {
    const auto start = Clock::now();
    for (unsigned i = 0; i < count; ++i)
        dispatcher.submit({first_request + i, {double(i), 0.0}});
    auto results = dispatcher.drain();
    const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
    if (results.size() != count) throw std::runtime_error("missing dispatch result");
    require_unique_assignments(results, count);
    for (unsigned i = 0; i < count; ++i)
        if (!dispatcher.release(i + 1)) throw std::runtime_error("release failed");
    return {ms, count};
}

int main(int argc, char** argv) {
    try {
        if (argc < 2) throw std::runtime_error("usage: dispatch_bench <result-file>");
        const std::string result_path = argv[1];
        const unsigned worker_counts[] = {1, 4, 8};

        {
            sr::Dispatcher dispatcher(fleet(kContentionDrivers), kContentionWorkers);
            for (unsigned i = 0; i < kSerialWarmup; ++i)
                dispatcher.submit({i + 1, {0.0, 0.0}});
            auto warm = dispatcher.drain();
            require_unique_assignments(warm, kContentionDrivers);
        }

        sr::Dispatcher contention(fleet(kContentionDrivers), kContentionWorkers);
        const auto contention_start = Clock::now();
        for (unsigned i = 0; i < kContentionRequests; ++i)
            contention.submit({1000u + i, {0.0, 0.0}});
        auto contention_results = contention.drain();
        const double contention_ms =
            std::chrono::duration<double, std::milli>(Clock::now() - contention_start).count();
        if (contention_results.size() != kContentionRequests)
            throw std::runtime_error("contention result count mismatch");
        require_unique_assignments(contention_results, kContentionDrivers);

        std::vector<double> serial_ms;
        serial_ms.reserve(kSerialSamples);
        {
            sr::Dispatcher dispatcher(fleet(kDrivers), 1);
            for (unsigned i = 0; i < kSerialWarmup + kSerialSamples; ++i) {
                const auto start = Clock::now();
                dispatcher.submit({i + 1, {0.0, 0.0}});
                auto results = dispatcher.drain();
                const double ms = std::chrono::duration<double, std::milli>(Clock::now() - start).count();
                if (results.size() != 1 || results[0].status != sr::DispatchStatus::Assigned)
                    throw std::runtime_error("serial assignment failed");
                if (!dispatcher.release(results[0].assignment.driver_id))
                    throw std::runtime_error("serial release failed");
                if (i >= kSerialWarmup) serial_ms.push_back(ms);
            }
        }

        struct WorkerRow {
            unsigned workers;
            double ms;
            unsigned assigned;
        };
        std::vector<WorkerRow> rows;
        for (unsigned workers : worker_counts) {
            sr::Dispatcher dispatcher(fleet(kDrivers), workers);
            double total_ms = 0;
            unsigned assigned = 0;
            for (unsigned round = 0; round < kRounds; ++round) {
                const RoundTiming timed = time_round(
                    dispatcher, 1u + round * kDrivers, kDrivers);
                total_ms += timed.ms;
                assigned += timed.assigned;
            }
            rows.push_back({workers, total_ms, assigned});
        }

        const double serial_total = std::accumulate(serial_ms.begin(), serial_ms.end(), 0.0);
        std::ostringstream report;
        report << std::fixed << std::setprecision(3);
        report << "# Concurrent dispatch benchmark\n\n"
               << "No road graph and no WAL. Driver i is at (i, 0). "
               << "Pickup cost is Euclidean and is not a routed trip.\n"
               << "Compiler: " << __VERSION__ << "; C++=" << __cplusplus << "\n"
               << "Seed: none. Positions and request ids are deterministic.\n"
               << "A discarded 4-driver dispatcher drains " << kSerialWarmup
               << " requests once before the contention sample. That drain is not timed.\n"
               << "Serial latency excludes its first " << kSerialWarmup
               << " submit/drain/release cycles. Throughput rounds have no warmup.\n\n"
               << "Latency, serial column: one idle dispatcher, 1 worker, "
               << kSerialSamples << " samples.\n"
               << "Each sample is submit() through drain() for one request. "
               << "It includes queue wait, nearest-driver matching, and result publication.\n"
               << "It excludes release() and excludes WAL fsync. Percentile index is floor(p * n).\n"
               << "Throughput rows: " << kRounds << " rounds of " << kDrivers
               << " requests on " << kDrivers << " drivers.\n"
               << "Each request i in a round is submitted at (i, 0), colocated with driver i+1.\n"
               << "Round time is the first submit() through drain(). "
               << "It includes queue wait, matching, and result publication.\n"
               << "release() runs after the timed drain and is not included. No WAL.\n"
               << "Assignments/sec uses the sum of round times and counts only Assigned results.\n"
               << "A duplicate driver, unexpected status, or wrong assignment count aborts "
               << "before this file is written.\n\n"
               << "Contention: " << kContentionWorkers << " workers, " << kContentionDrivers
               << " drivers, " << kContentionRequests
               << " requests, all pickups at (0, 0), no release during the wave.\n"
               << "Contention wall time: " << contention_ms << " ms. Assigned: "
               << kContentionDrivers << ". Unmatched: "
               << (kContentionRequests - kContentionDrivers) << ".\n\n"
               << "Serial submit-to-drain: p50 " << percentile(serial_ms, 0.50)
               << " ms; p95 " << percentile(serial_ms, 0.95)
               << " ms; p99 " << percentile(serial_ms, 0.99)
               << " ms; mean " << (serial_total / double(serial_ms.size()))
               << " ms.\n\n"
               << "| Workers | Drivers | Rounds | Requests/round | Assigned | Round sum ms | Assignments/sec |\n"
               << "|---|---|---|---|---|---|---|\n";
        for (const auto& row : rows) {
            report << "| " << row.workers
                   << " | " << kDrivers
                   << " | " << kRounds
                   << " | " << kDrivers
                   << " | " << row.assigned
                   << " | " << row.ms
                   << " | " << (1000.0 * row.assigned / row.ms)
                   << " |\n";
        }

        std::ofstream out(result_path);
        if (!out) throw std::runtime_error("could not write " + result_path);
        out << report.str();
        if (!out) throw std::runtime_error("failed while writing " + result_path);
        std::cout << report.str();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "Dispatch benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
