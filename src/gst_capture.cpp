#include "gst_capture.h"

#include "clock_domain.h"

#include <gst/app/gstappsink.h>
#include <gst/video/video.h>

#include <atomic>
#include <cstring>
#include <iostream>
#include <mutex>

#include <opencv2/imgproc.hpp>

namespace fd
{
namespace
{

std::once_flag g_gst_init_once;

/// A capture stamp may sit marginally ahead of "now" because the two clocks are
/// sampled at slightly different moments; that error is microseconds. Beyond
/// this the source is not running in real time, and the stamp is a media
/// timestamp rather than a capture time.
///
/// Kept well under one frame period: a stamp a whole frame in the future cannot
/// be a capture time. Too generous a slack lets a fast file replay publish
/// capture stamps later than their own publish stamps.
constexpr uint64_t kFutureStampSlackUs = 10000;  // 10 ms

/// Find the appsink. Prefers name=sink (what the shipped configs use); falls
/// back to scanning the bin so a hand-written pipeline that forgot to name it
/// still works.
GstElement* findAppSink(GstElement* pipeline)
{
    if (GstElement* named = gst_bin_get_by_name(GST_BIN(pipeline), "sink"))
    {
        if (GST_IS_APP_SINK(named)) return named;
        gst_object_unref(named);
    }

    GstIterator* it = gst_bin_iterate_recurse(GST_BIN(pipeline));
    GValue item = G_VALUE_INIT;
    GstElement* found = nullptr;
    bool done = false;
    while (!done)
    {
        switch (gst_iterator_next(it, &item))
        {
        case GST_ITERATOR_OK:
        {
            GstElement* e = GST_ELEMENT(g_value_get_object(&item));
            if (GST_IS_APP_SINK(e))
            {
                found = GST_ELEMENT(gst_object_ref(e));
                done = true;
            }
            g_value_reset(&item);
            break;
        }
        case GST_ITERATOR_RESYNC: gst_iterator_resync(it); break;
        default: done = true; break;
        }
    }
    g_value_unset(&item);
    gst_iterator_free(it);
    return found;
}

}  // namespace

// ---------------------------------------------------------------------------

void GstCapture::initGst(int* argc, char*** argv)
{
    std::call_once(g_gst_init_once, [&]() { gst_init(argc, argv); });
}

GstCapture::~GstCapture()
{
    close();
}

// ---------------------------------------------------------------------------

bool GstCapture::open(const std::string& pipeline_str, std::string& err)
{
    close();
    initGst();

    GError* gerr = nullptr;
    pipeline_ = gst_parse_launch(pipeline_str.c_str(), &gerr);
    if (!pipeline_ || gerr)
    {
        err = gerr ? gerr->message : "gst_parse_launch returned null";
        if (gerr) g_error_free(gerr);
        if (pipeline_) { gst_object_unref(pipeline_); pipeline_ = nullptr; }
        return false;
    }

    appsink_ = findAppSink(pipeline_);
    if (!appsink_)
    {
        err = "pipeline contains no appsink element (add: ! appsink name=sink drop=true max-buffers=2)";
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
        return false;
    }

    // Pull-based rather than signal-based, so this service drives the loop and
    // applies its own backpressure policy. drop/max-buffers from the config
    // string are not overridden: validate() already requires drop=true, and
    // rewriting the pipeline would hide a misconfiguration rather than report
    // it.
    g_object_set(G_OBJECT(appsink_), "emit-signals", FALSE, "sync", FALSE, nullptr);

    eos_ = false;
    error_ = false;
    last_error_.clear();

    const GstStateChangeReturn ret = gst_element_set_state(pipeline_, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        drainBus();
        err = last_error_.empty() ? "pipeline failed to reach PLAYING" : last_error_;
        close();
        return false;
    }

    // A live source reports NO_PREROLL; both that and ASYNC are fine, the first
    // read() blocks until a buffer actually arrives.
    return true;
}

// ---------------------------------------------------------------------------

void GstCapture::close()
{
    if (held_mapped_ && held_sample_)
    {
        GstBuffer* buf = gst_sample_get_buffer(held_sample_);
        if (buf) gst_buffer_unmap(buf, &held_map_);
        held_mapped_ = false;
    }
    if (held_sample_)
    {
        gst_sample_unref(held_sample_);
        held_sample_ = nullptr;
    }
    if (pipeline_)
    {
        gst_element_set_state(pipeline_, GST_STATE_NULL);
        if (appsink_) { gst_object_unref(appsink_); appsink_ = nullptr; }
        gst_object_unref(pipeline_);
        pipeline_ = nullptr;
    }
    convert_buf_.release();
}

// ---------------------------------------------------------------------------

void GstCapture::drainBus()
{
    if (!pipeline_) return;
    GstBus* bus = gst_element_get_bus(pipeline_);
    if (!bus) return;

    while (GstMessage* msg = gst_bus_pop_filtered(
               bus, static_cast<GstMessageType>(GST_MESSAGE_ERROR | GST_MESSAGE_EOS)))
    {
        if (GST_MESSAGE_TYPE(msg) == GST_MESSAGE_ERROR)
        {
            GError* gerr = nullptr;
            gchar* dbg = nullptr;
            gst_message_parse_error(msg, &gerr, &dbg);
            last_error_ = gerr ? gerr->message : "unknown pipeline error";
            if (dbg) { last_error_ += " | "; last_error_ += dbg; g_free(dbg); }
            if (gerr) g_error_free(gerr);
            error_ = true;
        }
        else
        {
            eos_ = true;
        }
        gst_message_unref(msg);
    }
    gst_object_unref(bus);
}

// ---------------------------------------------------------------------------

bool GstCapture::read(Frame& out, int timeout_ms)
{
    if (!pipeline_ || !appsink_) return false;

    // Release the previous frame's mapping. Frame::gray aliased it and the
    // caller has had its turn with it.
    if (held_mapped_ && held_sample_)
    {
        GstBuffer* prev = gst_sample_get_buffer(held_sample_);
        if (prev) gst_buffer_unmap(prev, &held_map_);
        held_mapped_ = false;
    }
    if (held_sample_)
    {
        gst_sample_unref(held_sample_);
        held_sample_ = nullptr;
    }

    GstSample* sample = gst_app_sink_try_pull_sample(
        GST_APP_SINK(appsink_), static_cast<GstClockTime>(timeout_ms) * GST_MSECOND);

    if (!sample)
    {
        // No buffer within the timeout. Could be a wedged pipeline (what the
        // watchdog is for), EOS, or a posted error. Check the bus so the
        // caller can tell them apart.
        drainBus();
        if (gst_app_sink_is_eos(GST_APP_SINK(appsink_))) eos_ = true;
        return false;
    }

    GstBuffer* buf = gst_sample_get_buffer(sample);
    GstCaps* caps  = gst_sample_get_caps(sample);
    if (!buf || !caps)
    {
        gst_sample_unref(sample);
        return false;
    }

    GstVideoInfo info;
    gst_video_info_init(&info);
    if (!gst_video_info_from_caps(&info, caps))
    {
        gst_sample_unref(sample);
        last_error_ = "could not parse video caps from appsink";
        error_ = true;
        return false;
    }

    const int w = GST_VIDEO_INFO_WIDTH(&info);
    const int h = GST_VIDEO_INFO_HEIGHT(&info);
    if (w <= 0 || h <= 0)
    {
        gst_sample_unref(sample);
        return false;
    }

    if (!gst_buffer_map(buf, &held_map_, GST_MAP_READ))
    {
        gst_sample_unref(sample);
        return false;
    }
    held_sample_ = sample;
    held_mapped_ = true;

    // ---- pixels ----------------------------------------------------------
    //
    // Luma-first formats provide the grayscale image directly in plane 0 at the
    // plane's own stride, with no conversion or copy; this is why the shipped
    // pipelines negotiate NV12. Any other format costs a conversion, which is
    // logged once.

    const GstVideoFormat fmt = GST_VIDEO_INFO_FORMAT(&info);
    const guint8* base = held_map_.data;
    const int plane0_stride = GST_VIDEO_INFO_PLANE_STRIDE(&info, 0);
    const size_t plane0_off = GST_VIDEO_INFO_PLANE_OFFSET(&info, 0);

    bool zero_copy = false;
    switch (fmt)
    {
    case GST_VIDEO_FORMAT_GRAY8:
    case GST_VIDEO_FORMAT_NV12:
    case GST_VIDEO_FORMAT_NV21:
    case GST_VIDEO_FORMAT_I420:
    case GST_VIDEO_FORMAT_YV12:
    case GST_VIDEO_FORMAT_Y41B:
    case GST_VIDEO_FORMAT_Y42B:
    case GST_VIDEO_FORMAT_Y444:
        zero_copy = true;
        break;
    default:
        zero_copy = false;
        break;
    }

    if (zero_copy)
    {
        out.gray = cv::Mat(h, w, CV_8UC1,
                           const_cast<guint8*>(base + plane0_off),
                           static_cast<size_t>(plane0_stride));
    }
    else
    {
        int cvt = -1;
        int src_type = CV_8UC3;
        switch (fmt)
        {
        case GST_VIDEO_FORMAT_BGR:  cvt = cv::COLOR_BGR2GRAY;   src_type = CV_8UC3; break;
        case GST_VIDEO_FORMAT_RGB:  cvt = cv::COLOR_RGB2GRAY;   src_type = CV_8UC3; break;
        case GST_VIDEO_FORMAT_BGRA:
        case GST_VIDEO_FORMAT_BGRx: cvt = cv::COLOR_BGRA2GRAY;  src_type = CV_8UC4; break;
        case GST_VIDEO_FORMAT_RGBA:
        case GST_VIDEO_FORMAT_RGBx: cvt = cv::COLOR_RGBA2GRAY;  src_type = CV_8UC4; break;
        case GST_VIDEO_FORMAT_YUY2: cvt = cv::COLOR_YUV2GRAY_YUY2; src_type = CV_8UC2; break;
        case GST_VIDEO_FORMAT_UYVY: cvt = cv::COLOR_YUV2GRAY_UYVY; src_type = CV_8UC2; break;
        default: break;
        }

        if (cvt < 0)
        {
            if (!warned_format_)
            {
                std::cerr << "[capture] unsupported appsink format '"
                          << gst_video_format_to_string(fmt)
                          << "'. Negotiate NV12 (preferred, zero-copy) or GRAY8 in the "
                             "gst: pipeline string.\n";
                warned_format_ = true;
            }
            return false;
        }

        if (!warned_format_)
        {
            std::cerr << "[capture] appsink format '" << gst_video_format_to_string(fmt)
                      << "' needs a per-frame colour conversion. Negotiating NV12 and using "
                         "plane 0 avoids it.\n";
            warned_format_ = true;
        }

        const cv::Mat src(h, w, src_type,
                          const_cast<guint8*>(base + plane0_off),
                          static_cast<size_t>(plane0_stride));
        cv::cvtColor(src, convert_buf_, cvt);
        out.gray = convert_buf_;
    }

    // ---- timestamp -------------------------------------------------------
    //
    // GST_BUFFER_PTS is running time: pipeline-relative, starting near zero.
    // Absolute capture time is pipeline base_time + PTS, on the pipeline clock.
    // GstSystemClock on Linux defaults to GST_CLOCK_TYPE_MONOTONIC, which is
    // the timebase the proto specifies, but a pipeline can be given a different
    // clock, so this checks rather than assumes.

    out.capture_time_exact = false;
    const GstClockTime pts = GST_BUFFER_PTS(buf);

    if (GST_CLOCK_TIME_IS_VALID(pts))
    {
        if (GstClock* clock = gst_element_get_clock(pipeline_))
        {
            if (GST_IS_SYSTEM_CLOCK(clock))
            {
                GstClockType ct = GST_CLOCK_TYPE_MONOTONIC;
                g_object_get(clock, "clock-type", &ct, nullptr);
                clock_is_monotonic_ = (ct == GST_CLOCK_TYPE_MONOTONIC);
            }
            gst_object_unref(clock);
        }

        const GstClockTime base = gst_element_get_base_time(pipeline_);
        if (clock_is_monotonic_ && GST_CLOCK_TIME_IS_VALID(base))
        {
            const uint64_t stamp = static_cast<uint64_t>((base + pts) / GST_USECOND);

            // A live camera's PTS tracks the clock, so base + PTS lands at or
            // just before now. A stamp in the future means the media timeline
            // is ahead of wall time, which is what a non-live source played
            // with sync=false does. That is a valid media timestamp but a
            // meaningless capture time, and publishing it would produce poses
            // that appear to come from the future.
            const uint64_t now = monotonicUs();
            if (stamp <= now + kFutureStampSlackUs)
            {
                out.capture_mono_us = stamp;
                out.capture_time_exact = true;
            }
            else if (!warned_future_)
            {
                std::cerr << "[capture] buffer PTS is "
                          << (stamp - now) / 1000 << " ms in the future; the source is not "
                             "running in real time (file replay with sync=false does this). "
                             "capture_timestamp_us falls back to arrival time; treat replay "
                             "latency figures as meaningless.\n";
                warned_future_ = true;
            }
        }
    }

    if (!out.capture_time_exact)
    {
        // Arrival time, which includes the ingest latency the exact stamp
        // excludes. Logged once rather than publishing a degraded value
        // silently under the same field name.
        out.capture_mono_us = monotonicUs();
        if (!warned_clock_)
        {
            std::cerr << "[capture] no usable buffer PTS on a monotonic pipeline clock; "
                         "capture_timestamp_us falls back to frame arrival time and "
                         "carries the ingest latency.\n";
            warned_clock_ = true;
        }
    }

    out.width  = w;
    out.height = h;
    return true;
}

}  // namespace fd
