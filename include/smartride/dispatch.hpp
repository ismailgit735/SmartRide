#pragma once
#include "matching.hpp"
#include <atomic>
#include <condition_variable>
#include <cstddef>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace sr {
// Available -> Reserved is the only reservation transition.
// Reserved -> Available happens only through Dispatcher::release.
// No other transitions are valid.
enum class DriverState : unsigned char { Available = 0, Reserved = 1 };

enum class DispatchStatus { Assigned, Unmatched, Duplicate, Rejected, PersistenceFailed };

class WriteAheadLog;

// Faults injected into the next WAL append. Kill points raise SIGKILL inside
// the logging process. FailFsync writes the full record and then throws.
enum class WalTestPoint : unsigned char {
    None = 0,
    BeforeWrite,
    AfterWrite,
    FailFsync,
    AfterFsync,
};

struct DispatchResult {
    Assignment assignment;
    DispatchStatus status = DispatchStatus::Unmatched;
};

struct DispatchDriver {
    uint32_t id = invalid;
    Point position;
    DriverState state = DriverState::Available;
};

// Concurrent nearest-driver dispatch. Positions are fixed for the lifetime of
// the object. Request ids are unique for that lifetime; a repeated id is
// recorded as Duplicate and is not matched. shutdown() is idempotent and
// drains work already queued. Submit after shutdown throws runtime_error.
//
// Reservation linearization point: the successful compare-and-swap from
// Available to Reserved inside the worker. A failed exchange means another
// request already reserved that driver.
//
// When a WAL path is supplied, Assigned is returned only after that reservation's
// record has been fsync'd. Release returns true only after its record has been
// fsync'd and the driver is Available. An append that fails before writing rolls
// the reservation back. An fsync failure after the record bytes were written
// leaves the driver Reserved and is not reported as success.
//
// commit_mu_ is acquired only when a WAL is present, and only before the WAL
// mutex. Queue mu_ is never held across that acquisition. Match, release, and
// rollback of a pre-durable reservation all run under commit_mu_, so a durable
// release cannot be undone by a later reservation in this process.
//
// If the process dies after a release record is fsync'd and before the
// in-memory transition to Available, replay follows the log: the driver is
// Available and is not reserved again by the earlier assignment.
class Dispatcher {
public:
    Dispatcher(std::vector<DispatchDriver> drivers, unsigned workers);
    // wal_path is created if needed. An existing log is replayed onto `drivers`
    // before workers start. Base driver state is the pre-log state.
    Dispatcher(std::vector<DispatchDriver> drivers, unsigned workers, const std::string& wal_path);
    Dispatcher(const Dispatcher&) = delete;
    Dispatcher& operator=(const Dispatcher&) = delete;
    ~Dispatcher();

    void submit(RideRequest request);
    // Blocks until every submitted request has a result. Does not stop workers.
    std::vector<DispatchResult> drain();
    void shutdown();
    // Reserved -> Available. Returns false if the driver was not Reserved.
    bool release(uint32_t driver_id);
    DriverState state_of(uint32_t driver_id) const;
    // The next `count` WAL appends throw before writing. Requires a WAL path.
    void testing_fail_next_appends(int count);
    void testing_arm(WalTestPoint point);

private:
    struct Slot {
        uint32_t id = invalid;
        Point position;
    };
    void worker();
    DispatchResult match(const RideRequest& request);
    size_t index_of(uint32_t driver_id) const;

    std::vector<Slot> drivers_;
    std::unique_ptr<std::atomic<unsigned char>[]> states_;
    std::unordered_map<uint32_t, size_t> index_;

    std::mutex mu_;
    std::condition_variable cv_;
    std::queue<RideRequest> queue_;
    std::vector<DispatchResult> results_;
    std::unordered_set<uint32_t> request_ids_;
    size_t outstanding_ = 0;
    bool stopping_ = false;
    std::atomic<bool> shutdown_started_{false};
    std::atomic<bool> joined_{false};
    std::vector<std::thread> threads_;
    std::unique_ptr<WriteAheadLog> wal_;
    // Serializes reservation and WAL append so release cannot pass an
    // assignment that has not reached fsync.
    std::mutex commit_mu_;
};
}
