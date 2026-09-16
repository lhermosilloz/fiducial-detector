#pragma once

// ---------------------------------------------------------------------------
// Fiducial detection and pose estimation, per stream.
//
// Detects on the raw frame and solves pose with cv::SOLVEPNP_IPPE_SQUARE,
// passing the distortion coefficients to solvePnP. This matches the object-point
// convention used by the SITL ArUco source, and avoids rectifying every pixel
// just to place four corners.
//
// Planar-marker pose ambiguity mostly affects rotation; translation is
// considerably more stable, and the consumer uses translation only.
//
// Pose is expressed in the camera optical frame. This class has no notion of a
// body frame, a world frame, or a vehicle.
// ---------------------------------------------------------------------------

#include <array>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

#include "app_config.h"
#include "aruco_compat.h"

namespace fd
{

struct Detection
{
    int         tag_id = 0;
    std::string family;
    float       size_m = 0.0f;

    /// Camera optical frame: +x right, +y down, +z forward.
    /// [2] is the range the consumer ranks candidates on.
    cv::Vec3d   t_tag_wrt_cam{0.0, 0.0, 0.0};

    /// Row-major tag->camera rotation. Published for future heading alignment;
    /// no consumer uses it today and it is not validated for that purpose.
    cv::Matx33d r_tag_to_cam = cv::Matx33d::eye();

    float reproj_error_px = 0.0f;

    /// Clockwise from top-left, in captured-frame coordinates.
    std::array<cv::Point2f, 4> corners{};
};

class Detector
{
public:
    Detector() = default;

    Detector(const Detector&)            = delete;
    Detector& operator=(const Detector&) = delete;

    /// Build the dictionary and detector parameters for this stream.
    /// Each stream owns its own dictionary, so per-camera families are safe.
    bool init(const StreamConfig& cfg, std::string& err);

    /// Detect and solve pose for every accepted marker in `gray`.
    /// Clears and fills `out`. Safe to call repeatedly; allocations are reused.
    void detect(const cv::Mat& gray, std::vector<Detection>& out);

    /// Counters since the last call, for the periodic stats line. Reset by
    /// takeStats().
    struct Stats
    {
        uint64_t decoded  = 0;   ///< markers the dictionary decoded
        uint64_t rejected_id     = 0;  ///< dropped by tags.accept_ids
        uint64_t rejected_pnp    = 0;  ///< solvePnP failed outright
        uint64_t rejected_reproj = 0;  ///< dropped by max_reproj_error_px
        uint64_t rejected_range  = 0;  ///< outside [min_range_m, max_range_m]
        uint64_t accepted = 0;
    };
    Stats takeStats();

    /// Intrinsics used for the pose solve, after any calibration-resolution
    /// rescale and any rectification. Logged at startup.
    const cv::Matx33d& effectiveK() const { return K_solve_; }
    bool  rescaled()  const { return rescaled_; }

private:
    /// Built on the first frame, once the negotiated resolution is known.
    /// Rescales intrinsics if they were calibrated at a different resolution,
    /// and builds the rectification maps if undistort is enabled.
    void lazyInit(const cv::Mat& gray);

    StreamConfig cfg_;
    ArucoDetectorCompat aruco_;

    bool  initialized_ = false;
    bool  rescaled_    = false;
    int   frame_w_ = 0, frame_h_ = 0;

    cv::Matx33d K_raw_   = cv::Matx33d::eye();  // as configured, rescaled to runtime size
    cv::Mat     dist_raw_;

    /// Intrinsics passed to solvePnP: K_raw_ normally, or the rectified
    /// intrinsics with zero distortion when undistort is enabled.
    cv::Matx33d K_solve_ = cv::Matx33d::eye();
    cv::Mat     dist_solve_;

    bool    undistort_ = false;
    cv::Mat map1_, map2_;
    cv::Mat undistorted_;

    // Reused across calls.
    std::vector<std::vector<cv::Point2f>> corners_;
    std::vector<int>                      ids_;
    std::vector<cv::Point3f>              obj_pts_;
    std::vector<cv::Point2f>              reproj_;

    Stats stats_;
};

}  // namespace fd
