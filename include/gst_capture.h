#pragma once

// ---------------------------------------------------------------------------
// Camera ingest: a GStreamer appsink driven by a pipeline string from config.
//
// This class knows nothing about Jetson, Pi, Argus, libcamera or V4L2. It is
// handed a pipeline string and negotiates whatever caps result, so the
// per-platform difference is a config line rather than a compile-time branch:
//
//   Jetson:  nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),... !
//            nvvidconv ! video/x-raw,format=NV12 ! appsink name=sink drop=true max-buffers=2
//   Pi:      libcamerasrc ! video/x-raw,format=NV12,... !
//            appsink name=sink drop=true max-buffers=2
//   USB:     v4l2src device=/dev/video0 ! video/x-raw,format=GRAY8,... !
//            appsink name=sink drop=true max-buffers=2
//   File:    filesrc location=clip.mp4 ! decodebin ! videoconvert !
//            video/x-raw,format=GRAY8 ! appsink name=sink drop=true max-buffers=2
//
// The file source makes the detector fully testable without hardware: it takes
// no telemetry, so replaying a clip produces identical output every run.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <string>

#include <gst/gst.h>
#include <opencv2/core.hpp>

namespace fd
{

class GstCapture
{
public:
    struct Frame
    {
        /// Single-channel 8-bit image handed to the detector.
        ///
        /// Aliases the mapped GStreamer buffer when the negotiated format is
        /// luma-first (GRAY8, NV12, I420, ...), which is why the shipped
        /// pipelines negotiate NV12 and use plane 0 instead of inserting a
        /// videoconvert. Valid only until the next read() or close().
        cv::Mat gray;

        /// Sensor exposure time on CLOCK_MONOTONIC, derived as pipeline
        /// base_time + GST_BUFFER_PTS. The raw PTS is pipeline-relative and is
        /// not a system timestamp.
        uint64_t capture_mono_us = 0;

        /// False when the source gave no usable PTS or the pipeline clock is
        /// not monotonic. capture_mono_us is then the frame arrival time, which
        /// includes ingest latency. Logged once per stream, not per frame.
        bool capture_time_exact = false;

        int width  = 0;
        int height = 0;
    };

    GstCapture() = default;
    ~GstCapture();

    GstCapture(const GstCapture&)            = delete;
    GstCapture& operator=(const GstCapture&) = delete;

    /// Build and start the pipeline. On failure returns false and fills `err`.
    bool open(const std::string& pipeline_str, std::string& err);

    /// Tear the pipeline down. Safe to call on a closed capture.
    void close();

    bool isOpen() const { return pipeline_ != nullptr; }

    /// Pull the next frame, blocking up to `timeout_ms`.
    /// Returns false on timeout, end-of-stream, or pipeline error; the caller
    /// distinguishes them with eos() and error().
    bool read(Frame& out, int timeout_ms);

    /// True once the pipeline posted EOS or an error on the bus.
    bool eos() const   { return eos_; }
    bool error() const { return error_; }

    /// Last bus error message, for logs.
    const std::string& lastError() const { return last_error_; }

    /// One-time GStreamer init. Idempotent, safe from any thread.
    static void initGst(int* argc = nullptr, char*** argv = nullptr);

private:
    void drainBus();

    GstElement* pipeline_ = nullptr;
    GstElement* appsink_  = nullptr;

    /// Held so Frame::gray can alias it until the next read.
    GstSample* held_sample_ = nullptr;
    bool       held_mapped_ = false;
    GstMapInfo held_map_{};
    cv::Mat    convert_buf_;   // only used on formats needing a real conversion

    bool        eos_   = false;
    bool        error_ = false;
    std::string last_error_;

    bool clock_is_monotonic_ = false;
    bool warned_clock_       = false;
    bool warned_format_      = false;
    bool warned_future_      = false;
};

}  // namespace fd
