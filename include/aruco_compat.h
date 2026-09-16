#pragma once

// ---------------------------------------------------------------------------
// OpenCV ArUco API compatibility.
//
// OpenCV moved ArUco from opencv_contrib into core objdetect in 4.7.0, changing
// the API shape:
//
//   4.0 - 4.6   <opencv2/aruco.hpp>, Ptr<Dictionary>,
//               Ptr<DetectorParameters>::create(), free detectMarkers()
//   >= 4.7      <opencv2/objdetect/aruco_detector.hpp>, Dictionary by value,
//               DetectorParameters struct, cv::aruco::ArucoDetector
//
// Raspberry Pi OS Bookworm ships 4.6; JetPack 6 ships 4.8+. Both are supported,
// and this is the only place in the codebase that may include an OpenCV ArUco
// header or test a version macro.
//
// OpenCV's ArUco detector reads AprilTag dictionaries (DICT_APRILTAG_36h11 and
// friends) directly, so no vendored AprilTag library is required.
// ---------------------------------------------------------------------------

#include <opencv2/core.hpp>
#include <string>
#include <vector>

#if CV_VERSION_MAJOR > 4 || (CV_VERSION_MAJOR == 4 && CV_VERSION_MINOR >= 7)
  #define FD_ARUCO_MODERN 1
  #include <opencv2/objdetect/aruco_detector.hpp>
#else
  #define FD_ARUCO_MODERN 0
  #include <opencv2/aruco.hpp>
#endif

namespace fd
{

/// Tunables we actually expose to YAML. Names match the config keys.
struct ArucoParams
{
    int   adaptive_thresh_win_size_min = 3;
    int   adaptive_thresh_win_size_max = 23;
    int   adaptive_thresh_win_size_step = 10;
    double min_marker_perimeter_rate   = 0.03;
    double max_marker_perimeter_rate   = 4.0;
    double polygonal_approx_accuracy_rate = 0.03;

    // "none" | "subpix" | "contour" | "apriltag"
    std::string corner_refine = "subpix";
    int    corner_refine_win_size  = 5;
    int    corner_refine_max_iterations = 30;
    double corner_refine_min_accuracy   = 0.1;

    // Caps how many bit errors the decoder will repair. Lower is stricter and
    // yields fewer false positives. OpenCV's ArUco does not expose AprilTag's
    // decision margin; this is the nearest equivalent, paired with
    // max_reproj_error_px in DetectorConfig.
    double error_correction_rate = 0.6;

    // Only consulted when corner_refine == "apriltag".
    double apriltag_quad_decimate = 0.0;

    bool detect_inverted_marker = false;
};

/// Map a family string from config to an OpenCV predefined-dictionary id.
/// Returns -1 if unknown.
inline int dictionaryIdFromFamily(const std::string& family)
{
    // ArUco
    if (family == "aruco_4x4_50")    return cv::aruco::DICT_4X4_50;
    if (family == "aruco_4x4_100")   return cv::aruco::DICT_4X4_100;
    if (family == "aruco_4x4_250")   return cv::aruco::DICT_4X4_250;
    if (family == "aruco_4x4_1000")  return cv::aruco::DICT_4X4_1000;
    if (family == "aruco_5x5_50")    return cv::aruco::DICT_5X5_50;
    if (family == "aruco_5x5_100")   return cv::aruco::DICT_5X5_100;
    if (family == "aruco_5x5_250")   return cv::aruco::DICT_5X5_250;
    if (family == "aruco_5x5_1000")  return cv::aruco::DICT_5X5_1000;
    if (family == "aruco_6x6_50")    return cv::aruco::DICT_6X6_50;
    if (family == "aruco_6x6_100")   return cv::aruco::DICT_6X6_100;
    if (family == "aruco_6x6_250")   return cv::aruco::DICT_6X6_250;
    if (family == "aruco_6x6_1000")  return cv::aruco::DICT_6X6_1000;
    if (family == "aruco_7x7_50")    return cv::aruco::DICT_7X7_50;
    if (family == "aruco_7x7_100")   return cv::aruco::DICT_7X7_100;
    if (family == "aruco_7x7_250")   return cv::aruco::DICT_7X7_250;
    if (family == "aruco_7x7_1000")  return cv::aruco::DICT_7X7_1000;
    if (family == "aruco_original")  return cv::aruco::DICT_ARUCO_ORIGINAL;
    // AprilTag, read by the same detector with no extra library.
    if (family == "apriltag_16h5")   return cv::aruco::DICT_APRILTAG_16h5;
    if (family == "apriltag_25h9")   return cv::aruco::DICT_APRILTAG_25h9;
    if (family == "apriltag_36h10")  return cv::aruco::DICT_APRILTAG_36h10;
    if (family == "apriltag_36h11")  return cv::aruco::DICT_APRILTAG_36h11;
    return -1;
}

inline int cornerRefineMethodFromString(const std::string& s)
{
    if (s == "none")     return cv::aruco::CORNER_REFINE_NONE;
    if (s == "contour")  return cv::aruco::CORNER_REFINE_CONTOUR;
    if (s == "apriltag") return cv::aruco::CORNER_REFINE_APRILTAG;
    return cv::aruco::CORNER_REFINE_SUBPIX;
}

// ---------------------------------------------------------------------------
// One detector instance owning its own dictionary. Non-copyable and held by
// value inside each stream's Detector, so per-stream families cannot be shared
// by accident.
// ---------------------------------------------------------------------------
class ArucoDetectorCompat
{
public:
    ArucoDetectorCompat() = default;
    ArucoDetectorCompat(const ArucoDetectorCompat&) = delete;
    ArucoDetectorCompat& operator=(const ArucoDetectorCompat&) = delete;

    /// Returns false if the family string is not a known dictionary.
    bool init(const std::string& family, const ArucoParams& p)
    {
        const int dict_id = dictionaryIdFromFamily(family);
        if (dict_id < 0) return false;

#if FD_ARUCO_MODERN
        cv::aruco::DetectorParameters params;
#else
        cv::Ptr<cv::aruco::DetectorParameters> params_ptr = cv::aruco::DetectorParameters::create();
        cv::aruco::DetectorParameters& params = *params_ptr;
#endif
        params.adaptiveThreshWinSizeMin  = p.adaptive_thresh_win_size_min;
        params.adaptiveThreshWinSizeMax  = p.adaptive_thresh_win_size_max;
        params.adaptiveThreshWinSizeStep = p.adaptive_thresh_win_size_step;
        params.minMarkerPerimeterRate    = p.min_marker_perimeter_rate;
        params.maxMarkerPerimeterRate    = p.max_marker_perimeter_rate;
        params.polygonalApproxAccuracyRate = p.polygonal_approx_accuracy_rate;
        params.errorCorrectionRate       = p.error_correction_rate;
        params.detectInvertedMarker      = p.detect_inverted_marker;

        // cornerRefinementMethod is `int` up to OpenCV 4.11 and an enum after;
        // decltype keeps this assignment valid either way.
        params.cornerRefinementMethod =
            static_cast<decltype(params.cornerRefinementMethod)>(
                cornerRefineMethodFromString(p.corner_refine));
        params.cornerRefinementWinSize     = p.corner_refine_win_size;
        params.cornerRefinementMaxIterations = p.corner_refine_max_iterations;
        params.cornerRefinementMinAccuracy   = p.corner_refine_min_accuracy;

        if (p.apriltag_quad_decimate > 0.0)
            params.aprilTagQuadDecimate = static_cast<float>(p.apriltag_quad_decimate);

#if FD_ARUCO_MODERN
        detector_ = cv::aruco::ArucoDetector(cv::aruco::getPredefinedDictionary(dict_id), params);
#else
        dictionary_ = cv::aruco::getPredefinedDictionary(dict_id);
        params_     = params_ptr;
#endif
        family_ = family;
        return true;
    }

    void detect(const cv::Mat& gray,
                std::vector<std::vector<cv::Point2f>>& corners,
                std::vector<int>& ids) const
    {
#if FD_ARUCO_MODERN
        detector_.detectMarkers(gray, corners, ids);
#else
        cv::aruco::detectMarkers(gray, dictionary_, corners, ids, params_);
#endif
    }

    const std::string& family() const { return family_; }

    /// Backend name, logged at startup to record which API path a board took.
    static const char* backend()
    {
#if FD_ARUCO_MODERN
        return "cv::aruco::ArucoDetector (OpenCV >= 4.7)";
#else
        return "cv::aruco::detectMarkers (OpenCV contrib < 4.7)";
#endif
    }

private:
    std::string family_;
#if FD_ARUCO_MODERN
    mutable cv::aruco::ArucoDetector detector_;
#else
    cv::Ptr<cv::aruco::Dictionary>         dictionary_;
    cv::Ptr<cv::aruco::DetectorParameters> params_;
#endif
};

}  // namespace fd
