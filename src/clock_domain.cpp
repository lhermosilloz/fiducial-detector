#include "clock_domain.h"

#include <ctime>
#include <fstream>
#include <random>
#include <sstream>

namespace fd
{
namespace
{

std::string readFirstLine(const char* path)
{
    std::ifstream f(path);
    if (!f.is_open()) return {};
    std::string line;
    std::getline(f, line);
    // Trim trailing whitespace; boot_id comes with a newline.
    while (!line.empty() && (line.back() == '\n' || line.back() == '\r' || line.back() == ' '))
        line.pop_back();
    return line;
}

std::string computeClockDomain()
{
    // Preferred: the kernel's per-boot UUID. Regenerated on every boot, which
    // is the property needed here: a reboot invalidates the monotonic
    // timebase, so it must invalidate the clock domain too.
    std::string boot_id = readFirstLine("/proc/sys/kernel/random/boot_id");
    if (!boot_id.empty()) return "boot:" + boot_id;

    // Fallback: machine-id is per-install, not per-boot, so pair it with the
    // boot instant (now - uptime) to regain the per-boot property.
    std::string machine_id = readFirstLine("/etc/machine-id");
    if (!machine_id.empty())
    {
        struct timespec up{};
        clock_gettime(CLOCK_BOOTTIME, &up);
        struct timespec now{};
        clock_gettime(CLOCK_REALTIME, &now);
        const long long boot_epoch = static_cast<long long>(now.tv_sec) - up.tv_sec;
        std::ostringstream os;
        os << "machine:" << machine_id << ":" << boot_epoch;
        return os.str();
    }

    // Last resort: random per-process. A consumer can still detect a MISMATCH,
    // which is the failure this field exists to catch; it just cannot recognise
    // two processes on the same box as sharing a timebase.
    std::random_device rd;
    std::ostringstream os;
    os << "rand:" << std::hex << ((static_cast<uint64_t>(rd()) << 32) | rd());
    return os.str();
}

}  // namespace

const std::string& clockDomain()
{
    static const std::string domain = computeClockDomain();
    return domain;
}

uint64_t monotonicUs()
{
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

uint64_t realtimeUs()
{
    struct timespec ts{};
    clock_gettime(CLOCK_REALTIME, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000ULL
         + static_cast<uint64_t>(ts.tv_nsec) / 1000ULL;
}

int64_t realtimeMinusMonotonicUs()
{
    static const int64_t offset = []() {
        // Sample both as close together as possible; the residual error is
        // microseconds, far below the 40-80 ms end-to-end latency this stamp
        // exists to measure.
        const uint64_t m = monotonicUs();
        const uint64_t r = realtimeUs();
        return static_cast<int64_t>(r) - static_cast<int64_t>(m);
    }();
    return offset;
}

}  // namespace fd
