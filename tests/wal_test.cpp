#include "smartride/dispatch.hpp"
#include "smartride/wal.hpp"
#include <csignal>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <sys/wait.h>
#include <unistd.h>
#include <vector>

#define CHECK(x) do { if (!(x)) throw std::runtime_error("check failed: " #x); } while (false)

std::string executable_path;

std::vector<sr::DispatchDriver> fleet(size_t n) {
    std::vector<sr::DispatchDriver> drivers;
    for (size_t i = 0; i < n; ++i)
        drivers.push_back({uint32_t(i + 1), {double(i), 0}, sr::DriverState::Available});
    return drivers;
}

struct TempDir {
    std::filesystem::path path;
    explicit TempDir(const std::string& name) {
        path = std::filesystem::temp_directory_path() / name;
        std::filesystem::remove_all(path);
        std::filesystem::create_directory(path);
    }
    ~TempDir() { std::filesystem::remove_all(path); }
    std::string wal() const { return (path / "rides.wal").string(); }
};

int crash_child(const char* directory) {
    sr::Dispatcher dispatcher(fleet(1), 1, std::string(directory) + "/rides.wal");
    dispatcher.submit({11, {0, 0}});
    auto results = dispatcher.drain();
    if (results.size() != 1 || results[0].status != sr::DispatchStatus::Assigned) return 2;
    if (results[0].assignment.driver_id != 1) return 3;
    kill(getpid(), SIGKILL);
    return 4;
}

void empty_log() {
    TempDir dir("smartride-wal-empty");
    sr::WriteAheadLog log(dir.wal());
    auto replay = log.replay(fleet(1));
    CHECK(replay.applied == 0);
    CHECK(!replay.truncated_tail);
    CHECK(replay.reserved_request.empty());
    auto again = log.replay(fleet(1));
    CHECK(again.reserved_request.empty());
}

void append_reopen_and_replay() {
    TempDir dir("smartride-wal-replay");
    {
        sr::WriteAheadLog log(dir.wal());
        log.append_assignment(4, 1);
        log.append_assignment(5, 2);
        log.append_assignment(4, 1);
    }
    sr::WriteAheadLog log(dir.wal());
    auto replay = log.replay(fleet(2));
    CHECK(replay.applied == 3);
    CHECK(replay.reserved_request.size() == 2);
    CHECK(replay.reserved_request.at(1) == 4);
    CHECK(replay.reserved_request.at(2) == 5);
    auto second = log.replay(fleet(2));
    CHECK(second.reserved_request == replay.reserved_request);
}

void duplicate_driver_is_rejected() {
    TempDir dir("smartride-wal-conflict");
    {
        sr::WriteAheadLog log(dir.wal());
        log.append_assignment(1, 1);
        log.append_assignment(2, 1);
    }
    sr::WriteAheadLog log(dir.wal());
    bool rejected = false;
    try { (void)log.replay(fleet(1)); }
    catch (const sr::WalFailure&) { rejected = true; }
    CHECK(rejected);
}

void corrupt_record_is_not_ignored() {
    TempDir dir("smartride-wal-corrupt");
    {
        sr::WriteAheadLog log(dir.wal());
        log.append_assignment(1, 1);
        log.append_assignment(2, 2);
    }
    {
        std::fstream file(dir.wal(), std::ios::in | std::ios::out | std::ios::binary);
        file.seekp(sr::WriteAheadLog::kRecordBytes + 8);
        char byte = 0;
        file.write(&byte, 1);
    }
    const auto size_before = std::filesystem::file_size(dir.wal());
    bool rejected = false;
    try { sr::WriteAheadLog log(dir.wal()); }
    catch (const sr::WalFailure& error) {
        rejected = std::string(error.what()).find("corrupt") != std::string::npos;
    }
    CHECK(rejected);
    CHECK(std::filesystem::file_size(dir.wal()) == size_before);
}

void truncated_tail_is_discarded() {
    TempDir dir("smartride-wal-torn");
    {
        sr::WriteAheadLog log(dir.wal());
        log.append_assignment(1, 1);
        log.append_assignment(2, 2);
    }
    {
        std::ofstream file(dir.wal(), std::ios::app | std::ios::binary);
        file.write("TORN", 4);
    }
    sr::WriteAheadLog log(dir.wal());
    CHECK(log.truncated_tail());
    auto replay = log.replay(fleet(2));
    CHECK(replay.truncated_tail);
    CHECK(replay.applied == 2);
    CHECK(replay.reserved_request.at(1) == 1);
    CHECK(replay.reserved_request.at(2) == 2);
    CHECK(std::filesystem::file_size(dir.wal()) == 2 * sr::WriteAheadLog::kRecordBytes);
}

void unknown_and_invalid_ids() {
    TempDir dir("smartride-wal-ids");
    sr::WriteAheadLog log(dir.wal());
    bool rejected = false;
    try { log.append_assignment(sr::invalid, 1); }
    catch (const sr::WalFailure&) { rejected = true; }
    CHECK(rejected);
    log.append_assignment(1, 99);
    bool unknown = false;
    try { (void)log.replay(fleet(1)); }
    catch (const sr::WalFailure& error) {
        unknown = std::string(error.what()).find("unknown") != std::string::npos;
    }
    CHECK(unknown);
}

void dispatcher_restart_and_release() {
    TempDir dir("smartride-wal-dispatch");
    {
        sr::Dispatcher dispatcher(fleet(2), 2, dir.wal());
        dispatcher.submit({1, {0, 0}});
        dispatcher.submit({2, {1, 0}});
        auto results = dispatcher.drain();
        CHECK(results.size() == 2);
        for (const auto& result : results) {
            CHECK(result.status == sr::DispatchStatus::Assigned);
            CHECK(result.assignment.driver_id == 1 || result.assignment.driver_id == 2);
        }
        CHECK(dispatcher.release(1));
        CHECK(dispatcher.state_of(1) == sr::DriverState::Available);
    }
    sr::Dispatcher restored(fleet(2), 2, dir.wal());
    CHECK(restored.state_of(1) == sr::DriverState::Available);
    CHECK(restored.state_of(2) == sr::DriverState::Reserved);
    restored.submit({3, {0, 0}});
    auto again = restored.drain();
    CHECK(again.size() == 1);
    CHECK(again[0].status == sr::DispatchStatus::Assigned);
    CHECK(again[0].assignment.driver_id == 1);
}

void failure_before_durability_is_not_acknowledged() {
    TempDir dir("smartride-wal-fail");
    {
        sr::Dispatcher dispatcher(fleet(1), 2, dir.wal());
        dispatcher.testing_fail_next_appends(1);
        dispatcher.submit({7, {0, 0}});
        auto failed = dispatcher.drain();
        CHECK(failed.size() == 1);
        CHECK(failed[0].status == sr::DispatchStatus::PersistenceFailed);
        CHECK(failed[0].assignment.driver_id == sr::invalid);
        CHECK(dispatcher.state_of(1) == sr::DriverState::Available);
        dispatcher.submit({8, {0, 0}});
        auto committed = dispatcher.drain();
        CHECK(committed.size() == 1);
        CHECK(committed[0].status == sr::DispatchStatus::Assigned);
        CHECK(committed[0].assignment.driver_id == 1);
    }
    {
        sr::Dispatcher restored(fleet(1), 1, dir.wal());
        CHECK(restored.state_of(1) == sr::DriverState::Reserved);
    }
    sr::WriteAheadLog log(dir.wal());
    auto replay = log.replay(fleet(1));
    CHECK(replay.reserved_request.size() == 1);
    CHECK(replay.reserved_request.at(1) == 8);
}

void release_replay_is_idempotent() {
    TempDir dir("smartride-wal-release-replay");
    {
        sr::WriteAheadLog log(dir.wal());
        log.append_assignment(4, 1);
        log.append_release(1);
        log.append_release(1);
        log.append_assignment(9, 1);
    }
    sr::WriteAheadLog log(dir.wal());
    auto replay = log.replay(fleet(1));
    CHECK(replay.reserved_request.size() == 1);
    CHECK(replay.reserved_request.at(1) == 9);
    auto second = log.replay(fleet(1));
    CHECK(second.reserved_request == replay.reserved_request);
    CHECK(second.applied == replay.applied);
}

void fsync_failure_is_not_assigned() {
    TempDir dir("smartride-wal-fsync-fail");
    {
        sr::Dispatcher dispatcher(fleet(1), 1, dir.wal());
        dispatcher.testing_arm(sr::WalTestPoint::FailFsync);
        dispatcher.submit({7, {0, 0}});
        auto failed = dispatcher.drain();
        CHECK(failed.size() == 1);
        CHECK(failed[0].status == sr::DispatchStatus::PersistenceFailed);
        CHECK(failed[0].status != sr::DispatchStatus::Assigned);
        CHECK(dispatcher.state_of(1) == sr::DriverState::Reserved);
    }
    sr::WriteAheadLog log(dir.wal());
    auto replay = log.replay(fleet(1));
    CHECK(replay.reserved_request.at(1) == 7);
    auto again = log.replay(fleet(1));
    CHECK(again.reserved_request == replay.reserved_request);
}

int boundary_child(const std::string& mode, const std::string& directory) {
    const std::string path = directory + "/rides.wal";
    sr::Dispatcher dispatcher(fleet(1), 1, path);
    if (mode == "release-after-fsync") {
        dispatcher.submit({11, {0, 0}});
        auto committed = dispatcher.drain();
        if (committed.size() != 1 || committed[0].status != sr::DispatchStatus::Assigned) return 2;
        dispatcher.testing_arm(sr::WalTestPoint::AfterFsync);
        dispatcher.release(1);
        return 3;
    }
    sr::WalTestPoint point = sr::WalTestPoint::None;
    if (mode == "before-write") point = sr::WalTestPoint::BeforeWrite;
    else if (mode == "after-write") point = sr::WalTestPoint::AfterWrite;
    else if (mode == "after-fsync") point = sr::WalTestPoint::AfterFsync;
    else return 2;
    dispatcher.testing_arm(point);
    dispatcher.submit({11, {0, 0}});
    (void)dispatcher.drain();
    return 4;
}

void expect_killed(const std::string& mode, const TempDir& dir) {
    const pid_t pid = fork();
    CHECK(pid >= 0);
    if (pid == 0) {
        execl(executable_path.c_str(), executable_path.c_str(), "--wal-boundary",
            mode.c_str(), dir.path.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    int status = 0;
    CHECK(waitpid(pid, &status, 0) == pid);
    CHECK(WIFSIGNALED(status));
    CHECK(WTERMSIG(status) == SIGKILL);
}

void crash_before_wal_bytes() {
    TempDir dir("smartride-wal-before-write");
    expect_killed("before-write", dir);
    CHECK(std::filesystem::file_size(dir.wal()) == 0);
    sr::WriteAheadLog log(dir.wal());
    auto replay = log.replay(fleet(1));
    CHECK(replay.reserved_request.empty());
    sr::Dispatcher restored(fleet(1), 1, dir.wal());
    CHECK(restored.state_of(1) == sr::DriverState::Available);
    restored.submit({12, {0, 0}});
    auto results = restored.drain();
    CHECK(results.size() == 1);
    CHECK(results[0].status == sr::DispatchStatus::Assigned);
    CHECK(results[0].assignment.driver_id == 1);
}

void crash_after_write_before_fsync() {
    TempDir dir("smartride-wal-after-write");
    expect_killed("after-write", dir);
    CHECK(std::filesystem::file_size(dir.wal()) == sr::WriteAheadLog::kRecordBytes);
    {
        sr::WriteAheadLog log(dir.wal());
        auto replay = log.replay(fleet(1));
        CHECK(replay.reserved_request.at(1) == 11);
        auto second = log.replay(fleet(1));
        CHECK(second.reserved_request == replay.reserved_request);
    }
    sr::Dispatcher restored(fleet(1), 1, dir.wal());
    CHECK(restored.state_of(1) == sr::DriverState::Reserved);
    restored.submit({12, {0, 0}});
    auto results = restored.drain();
    CHECK(results.size() == 1);
    CHECK(results[0].status == sr::DispatchStatus::Unmatched);
}

void crash_after_fsync_before_ack() {
    TempDir dir("smartride-wal-after-fsync");
    expect_killed("after-fsync", dir);
    CHECK(std::filesystem::file_size(dir.wal()) == sr::WriteAheadLog::kRecordBytes);
    sr::Dispatcher restored(fleet(1), 1, dir.wal());
    CHECK(restored.state_of(1) == sr::DriverState::Reserved);
    restored.submit({12, {0, 0}});
    auto results = restored.drain();
    CHECK(results.size() == 1);
    CHECK(results[0].status == sr::DispatchStatus::Unmatched);
}

void crash_after_durable_release_before_cas() {
    TempDir dir("smartride-wal-release-kill");
    expect_killed("release-after-fsync", dir);
    CHECK(std::filesystem::file_size(dir.wal()) == 2 * sr::WriteAheadLog::kRecordBytes);
    {
        sr::WriteAheadLog log(dir.wal());
        auto replay = log.replay(fleet(1));
        CHECK(replay.reserved_request.empty());
        auto second = log.replay(fleet(1));
        CHECK(second.reserved_request.empty());
    }
    sr::Dispatcher restored(fleet(1), 1, dir.wal());
    CHECK(restored.state_of(1) == sr::DriverState::Available);
    restored.submit({12, {0, 0}});
    auto results = restored.drain();
    CHECK(results.size() == 1);
    CHECK(results[0].status == sr::DispatchStatus::Assigned);
    CHECK(results[0].assignment.driver_id == 1);
}

void crash_after_acknowledgement() {
    TempDir dir("smartride-wal-crash");
    const pid_t pid = fork();
    CHECK(pid >= 0);
    if (pid == 0) {
        execl(executable_path.c_str(), executable_path.c_str(), "--wal-crash", dir.path.c_str(), static_cast<char*>(nullptr));
        _exit(127);
    }
    int status = 0;
    CHECK(waitpid(pid, &status, 0) == pid);
    CHECK(WIFSIGNALED(status));
    CHECK(WTERMSIG(status) == SIGKILL);
    {
        sr::WriteAheadLog log(dir.wal());
        auto replay = log.replay(fleet(1));
        CHECK(replay.reserved_request.at(1) == 11);
    }
    sr::Dispatcher restored(fleet(1), 1, dir.wal());
    CHECK(restored.state_of(1) == sr::DriverState::Reserved);
    restored.submit({12, {0, 0}});
    auto results = restored.drain();
    CHECK(results.size() == 1);
    CHECK(results[0].status == sr::DispatchStatus::Unmatched);
}

int main(int argc, char** argv) {
    executable_path = argv[0];
    if (argc == 3 && std::string(argv[1]) == "--wal-crash") return crash_child(argv[2]);
    if (argc == 4 && std::string(argv[1]) == "--wal-boundary")
        return boundary_child(argv[2], argv[3]);
    try {
        empty_log();
        append_reopen_and_replay();
        duplicate_driver_is_rejected();
        corrupt_record_is_not_ignored();
        truncated_tail_is_discarded();
        unknown_and_invalid_ids();
        dispatcher_restart_and_release();
        failure_before_durability_is_not_acknowledged();
        release_replay_is_idempotent();
        fsync_failure_is_not_assigned();
        crash_before_wal_bytes();
        crash_after_write_before_fsync();
        crash_after_fsync_before_ack();
        crash_after_durable_release_before_cas();
        crash_after_acknowledgement();
        std::cout << "wal tests passed\n";
        return 0;
    } catch (const std::exception& error) {
        std::cerr << "WAL test failure: " << error.what() << '\n';
        return 1;
    }
}
