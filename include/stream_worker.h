#pragma once

// ---------------------------------------------------------------------------
// One camera, one thread, the whole pipeline:
//
//     frame -> (optional rectify) -> detect -> pose -> publish
//
// One instance per `streams:` entry, all in one process sharing one UDP
// destination. Instances are independent: a wedged pipeline on one stream does
// not affect the others.
// ---------------------------------------------------------------------------

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "app_config.h"
#include "detector.h"
#include "gst_capture.h"
#include "udp_publisher.h"

namespace fd
{

class StreamWorker
{
public:
    StreamWorker(StreamConfig stream_cfg,
                 UdpConfig    udp_cfg,
                 LoggingConfig log_cfg,
                 std::atomic<bool>& stop_flag);

    StreamWorker(const StreamWorker&)            = delete;
    StreamWorker& operator=(const StreamWorker&) = delete;

    /// Construct the detector and socket. Returns false with `err` filled if
    /// this stream can never work (bad family, unusable intrinsics). A camera
    /// that is merely absent is retried by run() instead.
    bool init(std::string& err);

    /// Blocking main loop. Returns when the stop flag is set.
    void run();

    const std::string& cameraId() const { return cfg_.camera_id; }

private:
    /// Build the GStreamer pipeline, retrying with backoff. Returns false if
    /// the stop flag was set while retrying.
    bool openCapture();

    /// Heartbeat cadence, independent of frame delivery.
    void maybeHeartbeat(uint64_t now_us);

    void logStats(uint64_t now_us);

    StreamConfig  cfg_;
    UdpConfig     udp_cfg_;
    LoggingConfig log_cfg_;
    std::atomic<bool>& stop_;

    GstCapture capture_;
    Detector   detector_;
    std::unique_ptr<UdpPublisher> udp_;

    std::vector<Detection> detections_;

    int32_t  frame_id_ = 0;
    uint32_t last_w_   = 0;
    uint32_t last_h_   = 0;

    uint64_t last_publish_us_   = 0;   // any publish, detections or heartbeat
    uint64_t last_detect_us_    = 0;   // detector rate limiter
    uint64_t last_stats_us_     = 0;
    uint64_t last_rebuild_us_   = 0;
    double   rebuild_backoff_s_ = 0.0;
    uint64_t heartbeat_period_us_ = 0;
    uint64_t detect_period_us_    = 0;

    // Stats window
    uint64_t frames_in_      = 0;
    uint64_t frames_detected_= 0;
    uint64_t frames_skipped_ = 0;
    uint64_t heartbeats_     = 0;
    uint64_t rebuilds_       = 0;
    double   detect_ms_sum_  = 0.0;

    std::string tag_;
};

}  // namespace fd
