#include "stream_worker.h"

#include "clock_domain.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <thread>

namespace fd
{
namespace
{
constexpr int    kReadTimeoutMs      = 200;   // polling granularity for the watchdog
constexpr double kReopenBackoffMinS  = 1.0;
constexpr double kReopenBackoffMaxS  = 10.0;

/// A rebuild sooner than this after the previous one means the pipeline is
/// failing immediately on start rather than merely wedging. Retrying those at
/// full speed achieves nothing and loads the underlying camera stack.
constexpr double kRapidRebuildWindowS = 5.0;
}  // namespace

StreamWorker::StreamWorker(StreamConfig stream_cfg,
                           UdpConfig    udp_cfg,
                           LoggingConfig log_cfg,
                           std::atomic<bool>& stop_flag)
    : cfg_(std::move(stream_cfg))
    , udp_cfg_(std::move(udp_cfg))
    , log_cfg_(std::move(log_cfg))
    , stop_(stop_flag)
    , tag_("[" + cfg_.camera_id + "]")
{
    heartbeat_period_us_ = (udp_cfg_.heartbeat_hz > 0.0)
        ? static_cast<uint64_t>(1e6 / udp_cfg_.heartbeat_hz)
        : 0;
    detect_period_us_ = (cfg_.detect_fps > 0.0)
        ? static_cast<uint64_t>(1e6 / cfg_.detect_fps)
        : 0;

    last_w_ = static_cast<uint32_t>(cfg_.frame_width);
    last_h_ = static_cast<uint32_t>(cfg_.frame_height);
}

// ---------------------------------------------------------------------------

bool StreamWorker::init(std::string& err)
{
    if (!detector_.init(cfg_, err))
    {
        err = cfg_.camera_id + ": " + err;
        return false;
    }

    if (udp_cfg_.enabled)
    {
        UdpPublisher::Config pc;
        pc.dest_ip   = udp_cfg_.dest_ip;
        pc.dest_port = udp_cfg_.dest_port;
        pc.camera_id = cfg_.camera_id;
        udp_ = std::make_unique<UdpPublisher>(pc);
        if (!udp_->init(err))
        {
            err = cfg_.camera_id + ": " + err;
            return false;
        }
    }
    return true;
}

// ---------------------------------------------------------------------------

bool StreamWorker::openCapture()
{
    double backoff = kReopenBackoffMinS;
    int attempt = 0;

    while (!stop_.load())
    {
        ++attempt;
        std::string err;
        if (capture_.open(cfg_.gst, err))
        {
            std::cout << tag_ << " pipeline PLAYING"
                      << (attempt > 1 ? " (attempt " + std::to_string(attempt) + ")" : "")
                      << "\n";
            return true;
        }

        std::cerr << tag_ << " pipeline failed to start: " << err << "\n";
        if (attempt == 1)
            std::cerr << tag_ << "   pipeline: " << cfg_.gst << "\n"
                      << tag_ << "   verify the camera enumerates: tools/check_camera.sh\n";

        // Retry indefinitely rather than exiting: a camera that is not ready at
        // boot is normal on both platforms (Argus daemon startup, libcamera
        // enumeration).
        const auto deadline = std::chrono::steady_clock::now()
                            + std::chrono::milliseconds(static_cast<int>(backoff * 1000));
        while (!stop_.load() && std::chrono::steady_clock::now() < deadline)
            std::this_thread::sleep_for(std::chrono::milliseconds(100));

        backoff = std::min(backoff * 2.0, kReopenBackoffMaxS);
    }
    return false;
}

// ---------------------------------------------------------------------------

void StreamWorker::maybeHeartbeat(uint64_t now_us)
{
    if (!udp_ || heartbeat_period_us_ == 0) return;
    if (now_us - last_publish_us_ < heartbeat_period_us_) return;

    ++frame_id_;
    udp_->sendHeartbeat(now_us, frame_id_, last_w_, last_h_);
    last_publish_us_ = now_us;
    ++heartbeats_;
}

// ---------------------------------------------------------------------------

void StreamWorker::logStats(uint64_t now_us)
{
    if (log_cfg_.stats_interval_sec <= 0) return;
    const uint64_t interval_us = static_cast<uint64_t>(log_cfg_.stats_interval_sec) * 1000000ULL;
    if (now_us - last_stats_us_ < interval_us) return;

    const double secs = static_cast<double>(now_us - last_stats_us_) / 1e6;
    const Detector::Stats ds = detector_.takeStats();
    const double avg_ms = frames_detected_ ? detect_ms_sum_ / frames_detected_ : 0.0;

    std::ostringstream os;
    os << tag_ << std::fixed << std::setprecision(1)
       << " in=" << (frames_in_ / secs) << "fps"
       << " det=" << (frames_detected_ / secs) << "fps"
       << " skip=" << frames_skipped_
       << " tags=" << ds.accepted
       << " hb=" << heartbeats_
       << " avg_detect=" << std::setprecision(2) << avg_ms << "ms";

    if (ds.rejected_reproj || ds.rejected_range || ds.rejected_id || ds.rejected_pnp)
        os << "  rejected[reproj=" << ds.rejected_reproj
           << " range=" << ds.rejected_range
           << " id=" << ds.rejected_id
           << " pnp=" << ds.rejected_pnp << "]";

    if (rebuilds_) os << "  pipeline_rebuilds=" << rebuilds_;
    if (udp_ && udp_->sendErrors()) os << "  send_errors=" << udp_->sendErrors();

    std::cout << os.str() << "\n";

    frames_in_ = frames_detected_ = frames_skipped_ = heartbeats_ = 0;
    detect_ms_sum_ = 0.0;
    last_stats_us_ = now_us;
}

// ---------------------------------------------------------------------------

void StreamWorker::run()
{
    std::cout << tag_ << " starting: family=" << cfg_.detector.family
              << " size=" << cfg_.tags.default_size_m << "m"
              << " watchdog=" << cfg_.watchdog_timeout_sec << "s\n"
              << tag_ << " gst: " << cfg_.gst << "\n";

    last_stats_us_   = monotonicUs();
    last_publish_us_ = monotonicUs();
    last_rebuild_us_ = monotonicUs();

    if (!openCapture()) return;

    uint64_t last_frame_us = monotonicUs();
    bool logged_first_frame = false;

    while (!stop_.load())
    {
        GstCapture::Frame frame;
        const bool got = capture_.read(frame, kReadTimeoutMs);
        const uint64_t now = monotonicUs();

        if (got)
        {
            last_frame_us = now;
            ++frames_in_;
            last_w_ = static_cast<uint32_t>(frame.width);
            last_h_ = static_cast<uint32_t>(frame.height);

            if (!logged_first_frame)
            {
                std::cout << tag_ << " first frame " << frame.width << "x" << frame.height
                          << "  capture_timestamp="
                          << (frame.capture_time_exact ? "exact (PTS + base_time)"
                                                       : "ARRIVAL TIME (includes ingest latency)")
                          << "  detector=" << ArucoDetectorCompat::backend() << "\n";
                if (cfg_.frame_width != frame.width || cfg_.frame_height != frame.height)
                    std::cerr << tag_ << " negotiated " << frame.width << "x" << frame.height
                              << " but config says " << cfg_.frame_width << "x"
                              << cfg_.frame_height << "; the negotiated size wins. Fix the "
                                 "config so the intrinsics rescale check is meaningful\n";
                logged_first_frame = true;
            }

            // Backpressure: when the detector cannot keep up, skip the frame
            // rather than queueing it. The pipeline's drop=true max-buffers=2
            // does the same upstream. A stale pose is worse than a missing one.
            const bool due = (detect_period_us_ == 0)
                          || (now - last_detect_us_ >= detect_period_us_);

            if (!due)
            {
                ++frames_skipped_;
            }
            else
            {
                last_detect_us_ = now;

                const auto t0 = std::chrono::steady_clock::now();
                detector_.detect(frame.gray, detections_);
                const auto t1 = std::chrono::steady_clock::now();
                const float detect_ms =
                    std::chrono::duration<float, std::milli>(t1 - t0).count();

                ++frames_detected_;
                detect_ms_sum_ += detect_ms;

                if (!detections_.empty())
                {
                    ++frame_id_;
                    if (udp_)
                        udp_->send(frame.capture_mono_us, frame_id_, last_w_, last_h_,
                                   detections_, detect_ms);
                    last_publish_us_ = now;

                    if (log_cfg_.verbose)
                        for (const auto& d : detections_)
                            std::cout << tag_ << " id=" << d.tag_id
                                      << " t=(" << std::fixed << std::setprecision(3)
                                      << d.t_tag_wrt_cam[0] << ", " << d.t_tag_wrt_cam[1]
                                      << ", " << d.t_tag_wrt_cam[2] << ")m"
                                      << " range=" << d.t_tag_wrt_cam[2] << "m"
                                      << " reproj=" << std::setprecision(2)
                                      << d.reproj_error_px << "px\n";
                }
            }

            maybeHeartbeat(now);
            logStats(now);
            continue;
        }

        // ---- no frame ----------------------------------------------------
        //
        // Per-stream watchdog. Tears down and rebuilds this pipeline only; the
        // process stays up and all other streams keep running.

        const double idle_s = static_cast<double>(now - last_frame_us) / 1e6;
        const bool wedged = idle_s > cfg_.watchdog_timeout_sec;

        if (capture_.eos() && cfg_.exit_on_eos)
        {
            std::cout << tag_ << " end of stream; exit_on_eos is set, stopping this stream\n";
            break;
        }

        if (wedged || capture_.eos() || capture_.error())
        {
            const char* why = capture_.error() ? "pipeline error"
                            : capture_.eos()   ? "end of stream"
                                               : "watchdog timeout";
            std::cerr << tag_ << " rebuilding pipeline: " << why
                      << " (no frame for " << std::fixed << std::setprecision(1) << idle_s << "s)";
            if (!capture_.lastError().empty())
                std::cerr << ": " << capture_.lastError();
            std::cerr << "\n";

            capture_.close();
            ++rebuilds_;

            // Back off when rebuilds are consecutive. A wedged camera rebuilds
            // once and recovers; one failing on start would otherwise retry at
            // the read-timeout rate indefinitely.
            const double since_last_rebuild =
                static_cast<double>(now - last_rebuild_us_) / 1e6;
            last_rebuild_us_ = now;

            if (since_last_rebuild < kRapidRebuildWindowS)
            {
                rebuild_backoff_s_ = std::min(
                    rebuild_backoff_s_ > 0.0 ? rebuild_backoff_s_ * 2.0 : kReopenBackoffMinS,
                    kReopenBackoffMaxS);
                std::cerr << tag_ << " rebuilds are back-to-back (" << std::fixed
                          << std::setprecision(1) << since_last_rebuild
                          << "s apart); waiting " << rebuild_backoff_s_
                          << "s before retrying. Check that the source exists and is not "
                             "held by another process.\n";

                const auto deadline = std::chrono::steady_clock::now()
                    + std::chrono::milliseconds(static_cast<int>(rebuild_backoff_s_ * 1000));
                while (!stop_.load() && std::chrono::steady_clock::now() < deadline)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
                if (stop_.load()) break;
            }
            else
            {
                // Ran for a while before failing again: treat as a fresh
                // incident rather than escalating.
                rebuild_backoff_s_ = 0.0;
            }

            if (!openCapture()) return;
            last_frame_us = monotonicUs();
            logged_first_frame = false;
            continue;
        }

        // Idle but not yet wedged. Deliberately does not heartbeat: a heartbeat
        // asserts this camera is alive, so emitting one while no frames are
        // arriving would report the camera as healthy right up to the rebuild.
        logStats(now);
    }

    capture_.close();
    std::cout << tag_ << " stopped\n";
}

}  // namespace fd
