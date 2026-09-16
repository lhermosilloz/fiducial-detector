#pragma once

// ---------------------------------------------------------------------------
// YAML configuration: a `streams:` list, one process, one socket.
//
// Shape and file naming mirror jetson-vision-service's app_config. Missing keys
// are filled with documented defaults and written back, so a partially written
// config becomes a complete one.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>
#include <vector>

#include "aruco_compat.h"

namespace fd
{

// ---------------------------------------------------------------------------
// Per-stream camera intrinsics.
//
// Intrinsics are lens-specific and must be re-measured after any lens change or
// sensor-mode change. A 10% error in fx/fy produces a 10% range error on every
// detection, and the consumer ranks landing candidates on range.
// Use tools/calibrate_camera.py.
// ---------------------------------------------------------------------------
struct CameraConfig
{
    double fx = 0.0, fy = 0.0;   // pixels; 0 = unset, stream refuses to start
    double cx = 0.0, cy = 0.0;

    // "pinhole" (5-8 coeffs, cv::solvePnP) or "fisheye" (4 coeffs, cv::fisheye)
    std::string distortion_model = "pinhole";
    std::vector<double> dist_coeffs{0.0, 0.0, 0.0, 0.0, 0.0};

    // Full-frame undistortion before detection. Off by default: detecting on
    // the raw frame and letting solvePnP consume the distortion coefficients
    // avoids rectifying every pixel to place four corners. Enable only if a
    // wide lens bows tag edges enough to hurt detection rate; it costs a remap
    // per frame.
    //
    // When enabled, the pose solve switches to the rectified intrinsics with
    // zero distortion.
    bool undistort = false;

    /// Resolution these intrinsics were calibrated at. When the negotiated
    /// frame size differs, intrinsics are rescaled and a warning is logged.
    int calib_width  = 0;   // 0 = assume intrinsics match the runtime frame
    int calib_height = 0;

    bool valid() const { return fx > 0.0 && fy > 0.0 && cx > 0.0 && cy > 0.0; }
};

// ---------------------------------------------------------------------------
// Per-tag size override. Supports nested tag sets: a large tag for approach and
// a small co-located tag for close range, differing only by size_m.
// ---------------------------------------------------------------------------
struct TagSize
{
    int    id      = 0;
    double size_m  = 0.0;
};

struct TagsConfig
{
    // Measured with the convention the family expects. For AprilTag families
    // that is the outer edge of the black border, not the white quiet zone.
    double default_size_m = 0.30;
    std::vector<TagSize> sizes;

    // Empty accepts every id the dictionary decodes. Populating this is the
    // cheapest available false-positive filter.
    std::vector<int> accept_ids;

    double sizeFor(int id) const
    {
        for (const auto& s : sizes)
            if (s.id == id) return s.size_m;
        return default_size_m;
    }

    bool accepts(int id) const
    {
        if (accept_ids.empty()) return true;
        for (int a : accept_ids)
            if (a == id) return true;
        return false;
    }
};

struct DetectorConfig
{
    // apriltag_36h11 by default: better false-positive rejection than the 4x4
    // ArUco dictionary, and a false landing target is the expensive failure.
    // Set aruco_4x4_50 when testing against SITL assets, which use DICT_4X4_50.
    std::string family = "apriltag_36h11";

    ArucoParams params;

    // Reprojection gate. A pose whose corners do not reproject onto the
    // detected corners is a bad pose regardless of the decode. The value is
    // also published, because the consumer ranks candidates on it.
    // <= 0 disables the gate; the field is still populated.
    double max_reproj_error_px = 6.0;

    // Sanity gate on the solved range. An implausible range indicates a
    // mis-decode or a wrong size_m rather than a real detection.
    double min_range_m = 0.05;
    double max_range_m = 100.0;
};

// ---------------------------------------------------------------------------
// One camera stream.
// ---------------------------------------------------------------------------
struct StreamConfig
{
    /// Key the consumer demultiplexes on and uses to look up per-camera
    /// extrinsics. Must be unique across streams and stable across restarts.
    std::string camera_id = "down";

    // The only platform-varying value in this service.
    //
    // Jetson: nvarguscamerasrc ... ! nvvidconv ! video/x-raw,format=NV12 ! appsink
    // Pi:     libcamerasrc ! video/x-raw,format=NV12 ! appsink
    // USB:    v4l2src device=/dev/video0 ! video/x-raw,format=GRAY8 ! appsink
    //
    // Must end in an appsink. Set `drop=true max-buffers=2` on every pipeline:
    // the newest frame is wanted, not a queue.
    std::string gst;

    // Expected geometry; informational until the pipeline negotiates caps, at
    // which point the negotiated values win and a mismatch is logged.
    int frame_width  = 1280;
    int frame_height = 720;
    int fps          = 30;

    /// Per-stream watchdog. No buffer for this many seconds tears down and
    /// rebuilds this pipeline only; the process and all other streams stay up.
    double watchdog_timeout_sec = 3.0;

    /// What end-of-stream means for THIS source. A live camera never legitimately
    /// reaches EOS, so the default treats it as a wedge and rebuilds. A file
    /// source reaches EOS every time it finishes, where rebuilding means looping
    /// forever, which suits a soak test but not a one-shot replay.
    ///
    /// This sits next to `gst:` deliberately: the same config entry that says
    /// "this is a file" is the one that says "and stop when it ends".
    bool exit_on_eos = false;

    /// Cap on detector rate. 0 detects on every frame delivered. Use this where
    /// ingest outruns the detector: skipping on a schedule is preferable to
    /// falling progressively behind.
    double detect_fps = 0.0;

    CameraConfig   camera;
    DetectorConfig detector;
    TagsConfig     tags;
};

// ---------------------------------------------------------------------------
// Global sections.
// ---------------------------------------------------------------------------
struct UdpConfig
{
    bool        enabled   = true;
    std::string dest_ip   = "127.0.0.1";
    // 5601 is jetson-vision-service; this service uses 5602.
    uint16_t    dest_port = 5602;

    /// Per-camera liveness rate. Without heartbeats, a crashed detector and an
    /// empty field of view are indistinguishable to the consumer.
    double heartbeat_hz = 10.0;
};

struct LoggingConfig
{
    int  stats_interval_sec = 5;
    bool verbose            = false;   // per-detection lines; noisy, bench only
};

// ---------------------------------------------------------------------------
struct AppConfig
{
    std::vector<StreamConfig> streams;
    UdpConfig     udp;
    LoggingConfig logging;

    /// Parse `path`, filling every absent key with its documented default.
    /// Sets `healed` when anything was missing, so the caller can write back.
    /// Throws std::runtime_error on a YAML syntax error or a structurally
    /// unusable document (no streams, duplicate camera_id).
    static AppConfig loadFromFile(const std::string& path, bool* healed = nullptr);

    /// Re-emit the fully-populated config to `path`. yaml-cpp cannot preserve
    /// comments, so this writes a canonical document with a generated header.
    /// Called only when defaults were filled in.
    void writeToFile(const std::string& path) const;

    /// Semantic checks that parsing alone cannot make: usable intrinsics, an
    /// appsink in the pipeline, a known dictionary, sane sizes. Appends
    /// human-readable problems to `errors`. Returns true when clean.
    bool validate(std::vector<std::string>& errors) const;

    /// One-line-per-stream summary for startup logs and `-c`.
    std::string summary() const;
};

}  // namespace fd
