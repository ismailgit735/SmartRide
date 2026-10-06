#include "smartride/wal.hpp"
#include <algorithm>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <pthread.h>
#include <unistd.h>
#include <utility>
#include <vector>

namespace sr {
namespace {
void store_u16(unsigned char* p, uint16_t value) {
    p[0] = static_cast<unsigned char>(value);
    p[1] = static_cast<unsigned char>(value >> 8);
}
void store_u32(unsigned char* p, uint32_t value) {
    p[0] = static_cast<unsigned char>(value);
    p[1] = static_cast<unsigned char>(value >> 8);
    p[2] = static_cast<unsigned char>(value >> 16);
    p[3] = static_cast<unsigned char>(value >> 24);
}
uint16_t load_u16(const unsigned char* p) {
    return static_cast<uint16_t>(p[0] | (uint16_t(p[1]) << 8));
}
uint32_t load_u32(const unsigned char* p) {
    return uint32_t(p[0]) | (uint32_t(p[1]) << 8) | (uint32_t(p[2]) << 16) | (uint32_t(p[3]) << 24);
}
uint32_t crc32_ieee(const unsigned char* data, size_t n) {
    uint32_t crc = 0xffffffffu;
    for (size_t i = 0; i < n; ++i) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ (0xedb88320u & (0u - (crc & 1u)));
    }
    return ~crc;
}
[[noreturn]] void crash_process() {
    // Thread-directed SIGKILL stops this worker before a later write. If the
    // call returns, _exit still prevents that write from becoming reachable.
    pthread_kill(pthread_self(), SIGKILL);
    _exit(125);
}
void write_all(int fd, const unsigned char* data, size_t n) {
    size_t done = 0;
    while (done < n) {
        const ssize_t wrote = ::write(fd, data + done, n - done);
        if (wrote < 0) {
            if (errno == EINTR) continue;
            throw WalFailure(std::string("wal write failed: ") + std::strerror(errno));
        }
        if (wrote == 0) throw WalFailure("wal write returned zero");
        done += static_cast<size_t>(wrote);
    }
}
std::string parent_directory(const std::string& path) {
    const auto slash = path.find_last_of('/');
    if (slash == std::string::npos) return ".";
    if (slash == 0) return "/";
    return path.substr(0, slash);
}
struct Record {
    unsigned char type = 0;
    uint32_t request_id = invalid;
    uint32_t driver_id = invalid;
};
std::vector<Record> read_records(int fd, bool* truncated) {
    const off_t end = ::lseek(fd, 0, SEEK_END);
    if (end < 0) throw WalFailure(std::string("wal seek failed: ") + std::strerror(errno));
    const auto tail = static_cast<size_t>(end) % WriteAheadLog::kRecordBytes;
    const auto complete = static_cast<size_t>(end) / WriteAheadLog::kRecordBytes;
    *truncated = tail != 0;
    if (*truncated) {
        if (::ftruncate(fd, static_cast<off_t>(complete * WriteAheadLog::kRecordBytes)) != 0)
            throw WalFailure(std::string("wal truncate failed: ") + std::strerror(errno));
        if (::fsync(fd) != 0)
            throw WalFailure(std::string("wal truncate fsync failed: ") + std::strerror(errno));
    }
    if (::lseek(fd, 0, SEEK_SET) < 0)
        throw WalFailure(std::string("wal rewind failed: ") + std::strerror(errno));
    std::vector<Record> records;
    records.reserve(complete);
    for (size_t i = 0; i < complete; ++i) {
        unsigned char raw[WriteAheadLog::kRecordBytes];
        size_t got = 0;
        while (got < WriteAheadLog::kRecordBytes) {
            const ssize_t n = ::read(fd, raw + got, WriteAheadLog::kRecordBytes - got);
            if (n < 0) {
                if (errno == EINTR) continue;
                throw WalFailure(std::string("wal read failed: ") + std::strerror(errno));
            }
            if (n == 0) throw WalFailure("wal ended inside a framed record");
            got += static_cast<size_t>(n);
        }
        if (load_u32(raw) != WriteAheadLog::kMagic || raw[4] != WriteAheadLog::kVersion ||
            (raw[5] != WriteAheadLog::kAssign && raw[5] != WriteAheadLog::kRelease) ||
            load_u16(raw + 6) != 8 ||
            load_u32(raw + 16) != crc32_ieee(raw, 16))
            throw WalFailure("corrupt wal record");
        Record record;
        record.type = raw[5];
        record.request_id = load_u32(raw + 8);
        record.driver_id = load_u32(raw + 12);
        if (record.driver_id == invalid ||
            (record.type == WriteAheadLog::kAssign && record.request_id == invalid))
            throw WalFailure("wal record has an invalid id");
        records.push_back(record);
    }
    if (::lseek(fd, 0, SEEK_END) < 0)
        throw WalFailure(std::string("wal seek end failed: ") + std::strerror(errno));
    return records;
}
}

WriteAheadLog::WriteAheadLog(const std::string& path) : path_(path) {
    fd_ = ::open(path.c_str(), O_RDWR | O_CREAT, 0644);
    if (fd_ < 0) throw WalFailure(std::string("wal open failed: ") + std::strerror(errno));
    try {
        bool truncated = false;
        (void)read_records(fd_, &truncated);
        truncated_tail_ = truncated;
        const std::string parent = parent_directory(path);
        dir_fd_ = ::open(parent.c_str(), O_RDONLY);
        if (dir_fd_ < 0)
            throw WalFailure(std::string("wal directory open failed: ") + std::strerror(errno));
        if (::fsync(dir_fd_) != 0)
            throw WalFailure(std::string("wal directory fsync failed: ") + std::strerror(errno));
    } catch (...) {
        if (dir_fd_ >= 0) ::close(dir_fd_);
        ::close(fd_);
        fd_ = -1;
        throw;
    }
}

WriteAheadLog::~WriteAheadLog() {
    if (dir_fd_ >= 0) ::close(dir_fd_);
    if (fd_ >= 0) ::close(fd_);
}

void WriteAheadLog::testing_fail_next_appends(int count) {
    if (count < 0) throw std::invalid_argument("negative wal failure count");
    std::lock_guard<std::mutex> lock(mu_);
    fail_remaining_ = count;
}

void WriteAheadLog::testing_arm(WalTestPoint point) {
    std::lock_guard<std::mutex> lock(mu_);
    armed_ = point;
}

void WriteAheadLog::append_assignment(uint32_t request_id, uint32_t driver_id) {
    if (request_id == invalid || driver_id == invalid) throw WalFailure("wal assignment id is invalid");
    append(kAssign, request_id, driver_id);
}

void WriteAheadLog::append_release(uint32_t driver_id) {
    if (driver_id == invalid) throw WalFailure("wal release id is invalid");
    append(kRelease, invalid, driver_id);
}

void WriteAheadLog::append(unsigned char type, uint32_t request_id, uint32_t driver_id) {
    std::lock_guard<std::mutex> lock(mu_);
    if (fail_remaining_ > 0) {
        --fail_remaining_;
        throw WalFailure("wal append failed before durability");
    }
    const WalTestPoint point = armed_;
    armed_ = WalTestPoint::None;
    if (point == WalTestPoint::BeforeWrite) crash_process();
    unsigned char raw[kRecordBytes] = {};
    store_u32(raw, kMagic);
    raw[4] = kVersion;
    raw[5] = type;
    store_u16(raw + 6, 8);
    store_u32(raw + 8, request_id);
    store_u32(raw + 12, driver_id);
    store_u32(raw + 16, crc32_ieee(raw, 16));
    if (::lseek(fd_, 0, SEEK_END) < 0)
        throw WalFailure(std::string("wal seek failed: ") + std::strerror(errno));
    write_all(fd_, raw, kRecordBytes);
    if (point == WalTestPoint::AfterWrite) crash_process();
    if (point == WalTestPoint::FailFsync)
        throw WalFailure("wal fsync failed: injected", true);
    // Durability boundary. The assignment is not committed until this returns.
    if (::fsync(fd_) != 0)
        throw WalFailure(std::string("wal fsync failed: ") + std::strerror(errno), true);
    if (point == WalTestPoint::AfterFsync) crash_process();
}

WalReplay WriteAheadLog::replay(const std::vector<DispatchDriver>& base) const {
    std::lock_guard<std::mutex> lock(mu_);
    std::unordered_map<uint32_t, uint32_t> owner;
    std::unordered_map<uint32_t, uint32_t> request_owner;
    for (const auto& driver : base) {
        if (driver.id == invalid) throw WalFailure("wal replay base driver is invalid");
        if (driver.state == DriverState::Reserved)
            owner.emplace(driver.id, invalid);
    }
    bool truncated = false;
    const std::vector<Record> records = read_records(fd_, &truncated);
    if (truncated) truncated_tail_ = true;
    truncated = truncated_tail_;
    for (const auto& record : records) {
        if (!owner.count(record.driver_id) &&
            std::none_of(base.begin(), base.end(), [&](const DispatchDriver& driver) {
                return driver.id == record.driver_id;
            }))
            throw WalFailure("wal record names an unknown driver");
        if (record.type == kAssign) {
            const auto current = owner.find(record.driver_id);
            if (current != owner.end()) {
                if (current->second == record.request_id) continue;
                throw WalFailure("wal assigns an already reserved driver");
            }
            const auto previous = request_owner.find(record.request_id);
            if (previous != request_owner.end() && previous->second != record.driver_id)
                throw WalFailure("wal assigns one request to two drivers");
            owner.emplace(record.driver_id, record.request_id);
            request_owner[record.request_id] = record.driver_id;
        } else {
            const auto current = owner.find(record.driver_id);
            if (current != owner.end() && current->second != invalid)
                request_owner.erase(current->second);
            owner.erase(record.driver_id);
        }
    }
    WalReplay result;
    result.reserved_request = std::move(owner);
    result.applied = records.size();
    result.truncated_tail = truncated;
    return result;
}
}
