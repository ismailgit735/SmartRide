#pragma once
#include "dispatch.hpp"
#include <cstddef>
#include <cstdint>
#include <mutex>
#include <stdexcept>
#include <string>
#include <unordered_map>

namespace sr {
// Durable assignment log. A record is committed only after POSIX fsync(fd)
// returns success. fflush is not used and is not a durability boundary.
//
// Fixed 20-byte little-endian frame:
//   magic u32      bytes 'S','R','W','1'
//   version u8     1
//   type u8        1 = assignment, 2 = release
//   length u16     8
//   request_id u32 assignment request; invalid for a release
//   driver_id u32
//   crc32 u32      IEEE CRC of the first 16 bytes
//
// A short trailing fragment is a torn final record: it is discarded with
// ftruncate and is not applied. A full 20-byte frame that fails magic,
// version, type, length, or CRC is corruption. Recovery throws and does not
// apply any records from that file.
//
// Crash boundaries, relative to append()'s write and fsync:
//   before write()     no frame exists; replay does not commit the operation
//   after write(),     the caller has not been acknowledged. A complete frame
//   before fsync       that is still in the file is replayed as committed.
//                      A power loss before fsync can remove it; that loss is
//                      not reproducible by killing the process.
//   after fsync        the frame is committed even if the caller never
//                      observes success. Replay is idempotent.
class WalFailure : public std::runtime_error {
public:
    explicit WalFailure(const std::string& message, bool bytes_written = false)
        : std::runtime_error(message), bytes_written_(bytes_written) {}
    // True when a full record was written and fsync then failed. The bytes may
    // still be in the file, so the in-memory reservation must not be undone.
    bool bytes_written() const { return bytes_written_; }
private:
    bool bytes_written_ = false;
};

struct WalReplay {
    // Drivers still reserved after applying the log to the supplied base state.
    // The mapped value is the committed request id, or `invalid` when the base
    // state reserved the driver and the log did not release it.
    // Pickup distance is not stored in the log.
    std::unordered_map<uint32_t, uint32_t> reserved_request;
    size_t applied = 0;
    bool truncated_tail = false;
};

class WriteAheadLog {
public:
    static constexpr uint32_t kMagic = 0x31575253u;
    static constexpr unsigned char kVersion = 1;
    static constexpr unsigned char kAssign = 1;
    static constexpr unsigned char kRelease = 2;
    static constexpr std::size_t kRecordBytes = 20;

    explicit WriteAheadLog(const std::string& path);
    WriteAheadLog(const WriteAheadLog&) = delete;
    WriteAheadLog& operator=(const WriteAheadLog&) = delete;
    ~WriteAheadLog();

    void append_assignment(uint32_t request_id, uint32_t driver_id);
    void append_release(uint32_t driver_id);
    // Throws before the next `count` appends write any bytes.
    void testing_fail_next_appends(int count);
    // One-shot fault injected on the next append. Kill points raise SIGKILL.
    // FailFsync writes the full record, then throws with bytes_written() set,
    // without treating the operation as fsync-complete.
    void testing_arm(WalTestPoint point);
    WalReplay replay(const std::vector<DispatchDriver>& base) const;
    bool truncated_tail() const { return truncated_tail_; }

private:
    void append(unsigned char type, uint32_t request_id, uint32_t driver_id);
    int fd_ = -1;
    int dir_fd_ = -1;
    std::string path_;
    mutable bool truncated_tail_ = false;
    int fail_remaining_ = 0;
    WalTestPoint armed_ = WalTestPoint::None;
    mutable std::mutex mu_;
};
}
