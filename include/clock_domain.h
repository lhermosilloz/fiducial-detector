#pragma once

// ---------------------------------------------------------------------------
// Clock identity for published timestamps.
//
// capture_timestamp_us is CLOCK_MONOTONIC, which is per-machine and per-boot.
// When the detector and its consumer run on different machines, timestamps from
// the two are not comparable, and a staleness check against them either passes
// everything or rejects everything.
//
// Every frame therefore carries the producing machine's boot id. Same-box
// deployments can ignore it; cross-box consumers can detect a mismatch and
// refuse, or measure an offset using the CLOCK_REALTIME stamp published
// alongside.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>

namespace fd
{

/// Stable per-boot identifier for this machine. Reads
/// /proc/sys/kernel/random/boot_id, falling back to /etc/machine-id plus the
/// boot time, and finally to a process-lifetime random string. Computed once.
const std::string& clockDomain();

/// CLOCK_MONOTONIC, microseconds. The timebase for capture_timestamp_us.
uint64_t monotonicUs();

/// CLOCK_REALTIME, microseconds.
uint64_t realtimeUs();

/// realtime - monotonic, in microseconds, sampled once at startup. Converts a
/// monotonic capture stamp to wall clock without a second syscall per frame.
int64_t realtimeMinusMonotonicUs();

}  // namespace fd
