#include "app_config.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <fstream>
#include <set>
#include <sstream>
#include <stdexcept>

namespace fd
{
namespace
{

// Every read goes through this: an absent key yields its documented default,
// and the omission is recorded so the caller can write the completed file back.
struct Healer
{
    bool healed = false;

    template <typename T>
    T get(const YAML::Node& n, const std::string& key, T def)
    {
        if (!n || !n.IsMap() || !n[key])
        {
            healed = true;
            return def;
        }
        try
        {
            return n[key].as<T>();
        }
        catch (const YAML::Exception&)
        {
            throw std::runtime_error("[config] key '" + key + "' has the wrong type");
        }
    }

    /// Record an absent section so a config missing `detector:` entirely is
    /// still completed with every default spelled out.
    const YAML::Node section(const YAML::Node& n, const std::string& key)
    {
        if (!n || !n.IsMap() || !n[key])
        {
            healed = true;
            return YAML::Node(YAML::NodeType::Undefined);
        }
        return n[key];
    }
};

std::vector<double> parseDoubleList(const YAML::Node& n, const std::vector<double>& def)
{
    if (!n || !n.IsSequence()) return def;
    std::vector<double> out;
    out.reserve(n.size());
    for (const auto& v : n) out.push_back(v.as<double>());
    return out;
}

ArucoParams parseArucoParams(Healer& h, const YAML::Node& n)
{
    ArucoParams p;
    p.adaptive_thresh_win_size_min  = h.get(n, "adaptive_thresh_win_size_min",  p.adaptive_thresh_win_size_min);
    p.adaptive_thresh_win_size_max  = h.get(n, "adaptive_thresh_win_size_max",  p.adaptive_thresh_win_size_max);
    p.adaptive_thresh_win_size_step = h.get(n, "adaptive_thresh_win_size_step", p.adaptive_thresh_win_size_step);
    p.min_marker_perimeter_rate     = h.get(n, "min_marker_perimeter_rate",     p.min_marker_perimeter_rate);
    p.max_marker_perimeter_rate     = h.get(n, "max_marker_perimeter_rate",     p.max_marker_perimeter_rate);
    p.polygonal_approx_accuracy_rate= h.get(n, "polygonal_approx_accuracy_rate",p.polygonal_approx_accuracy_rate);
    p.corner_refine                 = h.get<std::string>(n, "corner_refine",    p.corner_refine);
    p.corner_refine_win_size        = h.get(n, "corner_refine_win_size",        p.corner_refine_win_size);
    p.corner_refine_max_iterations  = h.get(n, "corner_refine_max_iterations",  p.corner_refine_max_iterations);
    p.corner_refine_min_accuracy    = h.get(n, "corner_refine_min_accuracy",    p.corner_refine_min_accuracy);
    p.error_correction_rate         = h.get(n, "error_correction_rate",         p.error_correction_rate);
    p.apriltag_quad_decimate        = h.get(n, "apriltag_quad_decimate",        p.apriltag_quad_decimate);
    p.detect_inverted_marker        = h.get(n, "detect_inverted_marker",        p.detect_inverted_marker);
    return p;
}

CameraConfig parseCamera(Healer& h, const YAML::Node& n)
{
    CameraConfig c;
    c.fx = h.get(n, "fx", c.fx);
    c.fy = h.get(n, "fy", c.fy);
    c.cx = h.get(n, "cx", c.cx);
    c.cy = h.get(n, "cy", c.cy);
    c.distortion_model = h.get<std::string>(n, "distortion_model", c.distortion_model);
    if (n && n.IsMap() && n["dist_coeffs"])
        c.dist_coeffs = parseDoubleList(n["dist_coeffs"], c.dist_coeffs);
    else
        h.healed = true;
    c.undistort    = h.get(n, "undistort",    c.undistort);
    c.calib_width  = h.get(n, "calib_width",  c.calib_width);
    c.calib_height = h.get(n, "calib_height", c.calib_height);
    return c;
}

DetectorConfig parseDetector(Healer& h, const YAML::Node& n)
{
    DetectorConfig d;
    d.family              = h.get<std::string>(n, "family", d.family);
    d.max_reproj_error_px = h.get(n, "max_reproj_error_px", d.max_reproj_error_px);
    d.min_range_m         = h.get(n, "min_range_m",         d.min_range_m);
    d.max_range_m         = h.get(n, "max_range_m",         d.max_range_m);
    d.params              = parseArucoParams(h, h.section(n, "params"));
    return d;
}

TagsConfig parseTags(Healer& h, const YAML::Node& n)
{
    TagsConfig t;
    t.default_size_m = h.get(n, "default_size_m", t.default_size_m);

    if (n && n.IsMap() && n["sizes"] && n["sizes"].IsSequence())
    {
        for (const auto& e : n["sizes"])
        {
            TagSize ts;
            ts.id     = e["id"] ? e["id"].as<int>() : -1;
            ts.size_m = e["size_m"] ? e["size_m"].as<double>() : t.default_size_m;
            if (ts.id < 0)
                throw std::runtime_error("[config] tags.sizes entry missing 'id'");
            t.sizes.push_back(ts);
        }
    }
    if (n && n.IsMap() && n["accept_ids"] && n["accept_ids"].IsSequence())
        for (const auto& e : n["accept_ids"]) t.accept_ids.push_back(e.as<int>());

    return t;
}

StreamConfig parseStream(Healer& h, const YAML::Node& n)
{
    StreamConfig s;
    s.camera_id    = h.get<std::string>(n, "camera_id", s.camera_id);
    s.gst          = h.get<std::string>(n, "gst",       s.gst);
    s.frame_width  = h.get(n, "frame_width",  s.frame_width);
    s.frame_height = h.get(n, "frame_height", s.frame_height);
    s.fps          = h.get(n, "fps",          s.fps);
    s.watchdog_timeout_sec = h.get(n, "watchdog_timeout_sec", s.watchdog_timeout_sec);
    s.detect_fps   = h.get(n, "detect_fps",   s.detect_fps);
    s.exit_on_eos  = h.get(n, "exit_on_eos",  s.exit_on_eos);

    s.camera   = parseCamera(h,   h.section(n, "camera"));
    s.detector = parseDetector(h, h.section(n, "detector"));
    s.tags     = parseTags(h,     h.section(n, "tags"));
    return s;
}

}  // namespace

// ---------------------------------------------------------------------------

AppConfig AppConfig::loadFromFile(const std::string& path, bool* healed)
{
    YAML::Node root;
    try
    {
        root = YAML::LoadFile(path);
    }
    catch (const YAML::BadFile&)
    {
        throw std::runtime_error("[config] cannot open " + path);
    }
    catch (const YAML::Exception& e)
    {
        throw std::runtime_error("[config] YAML parse error in " + path + ": " + e.what());
    }

    Healer h;
    AppConfig cfg;

    if (!root["streams"] || !root["streams"].IsSequence() || root["streams"].size() == 0)
        throw std::runtime_error("[config] a non-empty 'streams:' list is required in " + path);

    for (const auto& sn : root["streams"]) cfg.streams.push_back(parseStream(h, sn));

    // camera_id is the consumer's demultiplex and extrinsics-lookup key.
    // Duplicates would merge two cameras into one extrinsics entry, producing
    // wrong poses rather than a visible error.
    std::set<std::string> seen;
    for (const auto& s : cfg.streams)
        if (!seen.insert(s.camera_id).second)
            throw std::runtime_error("[config] duplicate camera_id '" + s.camera_id + "'");

    const YAML::Node udp = h.section(root, "udp");
    cfg.udp.enabled      = h.get(udp, "enabled",   cfg.udp.enabled);
    cfg.udp.dest_ip      = h.get<std::string>(udp, "dest_ip", cfg.udp.dest_ip);
    cfg.udp.dest_port    = static_cast<uint16_t>(h.get<int>(udp, "dest_port", cfg.udp.dest_port));
    cfg.udp.heartbeat_hz = h.get(udp, "heartbeat_hz", cfg.udp.heartbeat_hz);

    const YAML::Node log = h.section(root, "logging");
    cfg.logging.stats_interval_sec = h.get(log, "stats_interval_sec", cfg.logging.stats_interval_sec);
    cfg.logging.verbose            = h.get(log, "verbose",            cfg.logging.verbose);

    if (healed) *healed = h.healed;
    return cfg;
}

// ---------------------------------------------------------------------------

void AppConfig::writeToFile(const std::string& path) const
{
    YAML::Emitter e;
    e << YAML::BeginMap;

    e << YAML::Key << "streams" << YAML::Value << YAML::BeginSeq;
    for (const auto& s : streams)
    {
        e << YAML::BeginMap;
        e << YAML::Key << "camera_id"            << YAML::Value << s.camera_id;
        e << YAML::Key << "gst"                  << YAML::Value << YAML::Literal << s.gst;
        e << YAML::Key << "frame_width"          << YAML::Value << s.frame_width;
        e << YAML::Key << "frame_height"         << YAML::Value << s.frame_height;
        e << YAML::Key << "fps"                  << YAML::Value << s.fps;
        e << YAML::Key << "watchdog_timeout_sec" << YAML::Value << s.watchdog_timeout_sec;
        e << YAML::Key << "detect_fps"           << YAML::Value << s.detect_fps;
        e << YAML::Key << "exit_on_eos"          << YAML::Value << s.exit_on_eos;

        e << YAML::Key << "camera" << YAML::Value << YAML::BeginMap;
        e << YAML::Key << "fx" << YAML::Value << s.camera.fx;
        e << YAML::Key << "fy" << YAML::Value << s.camera.fy;
        e << YAML::Key << "cx" << YAML::Value << s.camera.cx;
        e << YAML::Key << "cy" << YAML::Value << s.camera.cy;
        e << YAML::Key << "distortion_model" << YAML::Value << s.camera.distortion_model;
        e << YAML::Key << "dist_coeffs" << YAML::Value << YAML::Flow << s.camera.dist_coeffs;
        e << YAML::Key << "undistort"    << YAML::Value << s.camera.undistort;
        e << YAML::Key << "calib_width"  << YAML::Value << s.camera.calib_width;
        e << YAML::Key << "calib_height" << YAML::Value << s.camera.calib_height;
        e << YAML::EndMap;

        e << YAML::Key << "detector" << YAML::Value << YAML::BeginMap;
        e << YAML::Key << "family"              << YAML::Value << s.detector.family;
        e << YAML::Key << "max_reproj_error_px" << YAML::Value << s.detector.max_reproj_error_px;
        e << YAML::Key << "min_range_m"         << YAML::Value << s.detector.min_range_m;
        e << YAML::Key << "max_range_m"         << YAML::Value << s.detector.max_range_m;
        const ArucoParams& p = s.detector.params;
        e << YAML::Key << "params" << YAML::Value << YAML::BeginMap;
        e << YAML::Key << "adaptive_thresh_win_size_min"  << YAML::Value << p.adaptive_thresh_win_size_min;
        e << YAML::Key << "adaptive_thresh_win_size_max"  << YAML::Value << p.adaptive_thresh_win_size_max;
        e << YAML::Key << "adaptive_thresh_win_size_step" << YAML::Value << p.adaptive_thresh_win_size_step;
        e << YAML::Key << "min_marker_perimeter_rate"     << YAML::Value << p.min_marker_perimeter_rate;
        e << YAML::Key << "max_marker_perimeter_rate"     << YAML::Value << p.max_marker_perimeter_rate;
        e << YAML::Key << "polygonal_approx_accuracy_rate"<< YAML::Value << p.polygonal_approx_accuracy_rate;
        e << YAML::Key << "corner_refine"                 << YAML::Value << p.corner_refine;
        e << YAML::Key << "corner_refine_win_size"        << YAML::Value << p.corner_refine_win_size;
        e << YAML::Key << "corner_refine_max_iterations"  << YAML::Value << p.corner_refine_max_iterations;
        e << YAML::Key << "corner_refine_min_accuracy"    << YAML::Value << p.corner_refine_min_accuracy;
        e << YAML::Key << "error_correction_rate"         << YAML::Value << p.error_correction_rate;
        e << YAML::Key << "apriltag_quad_decimate"        << YAML::Value << p.apriltag_quad_decimate;
        e << YAML::Key << "detect_inverted_marker"        << YAML::Value << p.detect_inverted_marker;
        e << YAML::EndMap;
        e << YAML::EndMap;

        e << YAML::Key << "tags" << YAML::Value << YAML::BeginMap;
        e << YAML::Key << "default_size_m" << YAML::Value << s.tags.default_size_m;
        e << YAML::Key << "sizes" << YAML::Value << YAML::BeginSeq;
        for (const auto& ts : s.tags.sizes)
        {
            e << YAML::Flow << YAML::BeginMap;
            e << YAML::Key << "id" << YAML::Value << ts.id;
            e << YAML::Key << "size_m" << YAML::Value << ts.size_m;
            e << YAML::EndMap;
        }
        e << YAML::EndSeq;
        e << YAML::Key << "accept_ids" << YAML::Value << YAML::Flow << s.tags.accept_ids;
        e << YAML::EndMap;

        e << YAML::EndMap;
    }
    e << YAML::EndSeq;

    e << YAML::Key << "udp" << YAML::Value << YAML::BeginMap;
    e << YAML::Key << "enabled"      << YAML::Value << udp.enabled;
    e << YAML::Key << "dest_ip"      << YAML::Value << udp.dest_ip;
    e << YAML::Key << "dest_port"    << YAML::Value << static_cast<int>(udp.dest_port);
    e << YAML::Key << "heartbeat_hz" << YAML::Value << udp.heartbeat_hz;
    e << YAML::EndMap;

    e << YAML::Key << "logging" << YAML::Value << YAML::BeginMap;
    e << YAML::Key << "stats_interval_sec" << YAML::Value << logging.stats_interval_sec;
    e << YAML::Key << "verbose"            << YAML::Value << logging.verbose;
    e << YAML::EndMap;

    e << YAML::EndMap;

    // Write to a temp file and rename, so an interrupted write cannot leave a
    // truncated config on the board.
    const std::string tmp = path + ".tmp";
    {
        std::ofstream out(tmp, std::ios::trunc);
        if (!out) throw std::runtime_error("[config] cannot write " + tmp);
        out << "# fiducial-detector-service config\n"
               "#\n"
               "# Rewritten by the service: absent keys were filled with their defaults.\n"
               "# Comments from the previous file are not preserved by this round-trip.\n"
               "# Validate without starting:  fiducial-detector-service -c <file>\n"
               "#\n"
               "# `gst:` is the only platform-varying value in this service.\n"
               "#   Jetson: nvarguscamerasrc ... ! nvvidconv ! video/x-raw,format=NV12 ! appsink\n"
               "#   Pi:     libcamerasrc ! video/x-raw,format=NV12 ! appsink\n"
               "\n"
            << e.c_str() << "\n";
        if (!out) throw std::runtime_error("[config] write failed: " + tmp);
    }
    if (std::rename(tmp.c_str(), path.c_str()) != 0)
        throw std::runtime_error("[config] cannot replace " + path);
}

// ---------------------------------------------------------------------------

bool AppConfig::validate(std::vector<std::string>& errors) const
{
    const size_t before = errors.size();

    if (streams.empty()) errors.push_back("no streams configured");

    for (const auto& s : streams)
    {
        const std::string tag = "stream '" + s.camera_id + "': ";

        if (s.camera_id.empty()) errors.push_back("a stream has an empty camera_id");

        if (s.gst.empty())
            errors.push_back(tag + "'gst' pipeline is empty; this is the per-platform "
                                   "pipeline string (nvarguscamerasrc / libcamerasrc / v4l2src)");
        else if (s.gst.find("appsink") == std::string::npos)
            errors.push_back(tag + "'gst' pipeline must end in an appsink element");
        else if (!s.exit_on_eos && s.gst.find("drop=true") == std::string::npos)
            // Required for a live source, where the newest frame is wanted and a
            // backlog is worse than a gap. A finite source (exit_on_eos) is
            // replay: every frame should be processed, so dropping is wrong
            // there and the check does not apply.
            errors.push_back(tag + "'gst' appsink should set drop=true max-buffers=2 "
                                   "so the newest frame is used rather than a queued one");

        if (!s.camera.valid())
            errors.push_back(tag + "camera intrinsics (fx, fy, cx, cy) are unset; "
                                   "calibrate with tools/calibrate_camera.py");

        if (s.camera.distortion_model != "pinhole" && s.camera.distortion_model != "fisheye")
            errors.push_back(tag + "camera.distortion_model must be 'pinhole' or 'fisheye', got '"
                             + s.camera.distortion_model + "'");

        if (s.camera.distortion_model == "fisheye" && s.camera.dist_coeffs.size() != 4)
            errors.push_back(tag + "fisheye model needs exactly 4 distortion coefficients, got "
                             + std::to_string(s.camera.dist_coeffs.size()));

        if (s.camera.distortion_model == "pinhole"
            && s.camera.dist_coeffs.size() != 4 && s.camera.dist_coeffs.size() != 5
            && s.camera.dist_coeffs.size() != 8)
            errors.push_back(tag + "pinhole model needs 4, 5, or 8 distortion coefficients, got "
                             + std::to_string(s.camera.dist_coeffs.size()));

        if (dictionaryIdFromFamily(s.detector.family) < 0)
            errors.push_back(tag + "unknown detector.family '" + s.detector.family
                             + "' (try apriltag_36h11 or aruco_4x4_50)");

        if (s.detector.family == "apriltag_16h5")
            errors.push_back(tag + "detector.family 'apriltag_16h5' has a high "
                                   "false-positive rate; prefer apriltag_36h11");

        if (s.tags.default_size_m <= 0.0)
            errors.push_back(tag + "tags.default_size_m must be > 0; a size error "
                                   "produces a proportional range error");

        for (const auto& ts : s.tags.sizes)
            if (ts.size_m <= 0.0)
                errors.push_back(tag + "tags.sizes id " + std::to_string(ts.id) + " has size_m <= 0");

        if (s.watchdog_timeout_sec <= 0.0)
            errors.push_back(tag + "watchdog_timeout_sec must be > 0; otherwise a wedged "
                                   "pipeline is never rebuilt");
    }

    if (udp.enabled && udp.dest_port == 0)
        errors.push_back("udp.dest_port is 0");
    if (udp.enabled && udp.dest_port == 5601)
        errors.push_back("udp.dest_port 5601 belongs to jetson-vision-service; this service uses 5602");
    if (udp.enabled && udp.heartbeat_hz <= 0.0)
        errors.push_back("udp.heartbeat_hz must be > 0; heartbeats are what distinguish "
                         "'no tag in view' from 'detector dead'");

    return errors.size() == before;
}

// ---------------------------------------------------------------------------

std::string AppConfig::summary() const
{
    std::ostringstream os;
    os << streams.size() << " stream(s), udp "
       << (udp.enabled ? (udp.dest_ip + ":" + std::to_string(udp.dest_port)) : std::string("disabled"))
       << ", heartbeat " << udp.heartbeat_hz << " Hz\n";
    for (const auto& s : streams)
    {
        os << "  [" << s.camera_id << "] " << s.detector.family
           << "  size=" << s.tags.default_size_m << "m"
           << "  fx=" << s.camera.fx << " fy=" << s.camera.fy
           << " cx=" << s.camera.cx << " cy=" << s.camera.cy
           << "  " << s.camera.distortion_model
           << (s.camera.undistort ? " (undistort on)" : "")
           << "  watchdog=" << s.watchdog_timeout_sec << "s\n";
    }
    return os.str();
}

}  // namespace fd
