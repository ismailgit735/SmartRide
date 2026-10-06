#include "smartride/dispatch.hpp"
#include "smartride/wal.hpp"
#include <algorithm>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <numeric>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

using Clock = std::chrono::steady_clock;

constexpr int kAppendRecords = 48;
constexpr int kAppendWarmup = 4;
constexpr int kDispatchRecords = 16;

double percentile(std::vector<double> values, double p) {
    std::sort(values.begin(), values.end());
    return values[std::min(values.size() - 1, size_t(p * values.size()))];
}

std::vector<sr::DispatchDriver> fleet(int n) {
    std::vector<sr::DispatchDriver> drivers;
    drivers.reserve(size_t(n));
    for (int i = 0; i < n; ++i)
        drivers.push_back({uint32_t(i + 1), {double(i), 0.0}, sr::DriverState::Available});
    return drivers;
}

struct TempDir {
    std::filesystem::path path;
    explicit TempDir(const std::filesystem::path& directory) : path(directory) {
        std::filesystem::remove_all(path);
        std::filesystem::create_directories(path);
    }
    ~TempDir() { std::filesystem::remove_all(path); }
    std::string wal() const { return (path / "rides.wal").string(); }
};

int main(int argc, char** argv) {
    try {
        if (argc < 2) throw std::runtime_error("usage: wal_bench <result-file>");
        const std::string result_path = argv[1];
        const std::filesystem::path result(result_path);
        TempDir dir(result.parent_path() / "wal-bench-tmp");

        std::vector<double> append_ms;
        append_ms.reserve(kAppendRecords);
        {
            sr::WriteAheadLog warmup(dir.wal() + ".warmup");
            for (int i = 0; i < kAppendWarmup; ++i)
                warmup.append_assignment(uint32_t(1000 + i), 1);
        }
        std::filesystem::remove(dir.wal() + ".warmup");
        {
            sr::WriteAheadLog log(dir.wal());
            for (int i = 0; i < kAppendRecords; ++i) {
                const auto start = Clock::now();
                log.append_assignment(uint32_t(i + 1), uint32_t(i + 1));
                append_ms.push_back(
                    std::chrono::duration<double, std::milli>(Clock::now() - start).count());
            }
        }
        {
            sr::WriteAheadLog log(dir.wal());
            const sr::WalReplay replay = log.replay(fleet(kAppendRecords));
            if (replay.applied != size_t(kAppendRecords) ||
                int(replay.reserved_request.size()) != kAppendRecords)
                throw std::runtime_error("wal append replay count mismatch");
            for (int i = 0; i < kAppendRecords; ++i) {
                const auto found = replay.reserved_request.find(uint32_t(i + 1));
                if (found == replay.reserved_request.end() || found->second != uint32_t(i + 1))
                    throw std::runtime_error("wal append replay mismatch");
            }
        }

        std::filesystem::remove_all(dir.path);
        std::filesystem::create_directories(dir.path);
        std::vector<double> dispatch_ms;
        dispatch_ms.reserve(kDispatchRecords);
        {
            sr::Dispatcher dispatcher(fleet(kDispatchRecords), 1, dir.wal());
            for (int i = 0; i < kDispatchRecords; ++i) {
                const auto start = Clock::now();
                dispatcher.submit({uint32_t(i + 1), {double(i), 0.0}});
                auto results = dispatcher.drain();
                dispatch_ms.push_back(
                    std::chrono::duration<double, std::milli>(Clock::now() - start).count());
                if (results.size() != 1 || results[0].status != sr::DispatchStatus::Assigned ||
                    results[0].assignment.driver_id != uint32_t(i + 1))
                    throw std::runtime_error("wal dispatch assignment mismatch");
            }
        }
        {
            sr::Dispatcher restored(fleet(kDispatchRecords), 1, dir.wal());
            for (int i = 0; i < kDispatchRecords; ++i)
                if (restored.state_of(uint32_t(i + 1)) != sr::DriverState::Reserved)
                    throw std::runtime_error("wal dispatch recovery mismatch");
        }

        const double append_total = std::accumulate(append_ms.begin(), append_ms.end(), 0.0);
        const double dispatch_total = std::accumulate(dispatch_ms.begin(), dispatch_ms.end(), 0.0);
        std::ostringstream report;
        report << std::fixed << std::setprecision(3);
        report << "# WAL durability benchmark\n\n"
               << "Each WriteAheadLog::append_assignment call writes one 20-byte record and fsyncs it.\n"
               << "Records are not grouped. fflush is not used.\n"
               << "Compiler: " << __VERSION__ << "; C++=" << __cplusplus << "\n"
               << "Seed: none. Record ids are sequential.\n"
               << "Append warmup: " << kAppendWarmup
               << " calls on a discarded log, excluded from the timed sample and from replay.\n"
               << "Timed append latency is one append_assignment call: the write plus its fsync.\n"
               << "It does not include dispatcher queueing or matching.\n"
               << "Dispatch latency is submit() through drain() on one worker with a WAL.\n"
               << "It includes queue wait, nearest-driver matching, one record write, and that record's fsync.\n"
               << "Percentile index is floor(p * n). A replay mismatch aborts before this file is written.\n\n"
               << "Direct appends: " << kAppendRecords << " timed records.\n"
               << "Sum of timed append calls: " << append_total << " ms; "
               << (1000.0 * append_ms.size() / append_total) << " records/sec.\n"
               << "Append p50/p95/p99/mean ms: "
               << percentile(append_ms, 0.50) << " / "
               << percentile(append_ms, 0.95) << " / "
               << percentile(append_ms, 0.99) << " / "
               << (append_total / double(append_ms.size())) << "\n\n"
               << "Dispatcher assignments: " << kDispatchRecords
               << " drivers, 1 worker, one request at a time.\n"
               << "Sum of submit-to-drain calls: " << dispatch_total << " ms; "
               << (1000.0 * dispatch_ms.size() / dispatch_total) << " assignments/sec.\n"
               << "Dispatch p50/p95/p99/mean ms: "
               << percentile(dispatch_ms, 0.50) << " / "
               << percentile(dispatch_ms, 0.95) << " / "
               << percentile(dispatch_ms, 0.99) << " / "
               << (dispatch_total / double(dispatch_ms.size())) << "\n"
               << "Individual fsync per record: yes.\n";

        std::ofstream out(result_path);
        if (!out) throw std::runtime_error("could not write " + result_path);
        out << report.str();
        if (!out) throw std::runtime_error("failed while writing " + result_path);
        std::cout << report.str();
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "WAL benchmark failed: " << error.what() << '\n';
        return 1;
    }
}
