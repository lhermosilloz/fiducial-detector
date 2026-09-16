#include "detector.h"

#include <cmath>
#include <iostream>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace fd
{

bool Detector::init(const StreamConfig& cfg, std::string& err)
{
    cfg_ = cfg;

    if (!aruco_.init(cfg_.detector.family, cfg_.detector.params))
    {
        err = "unknown detector.family '" + cfg_.detector.family + "'";
        return false;
    }

    if (!cfg_.camera.valid())
    {
        err = "camera intrinsics are unset";
        return false;
    }

    K_raw_ = cv::Matx33d(cfg_.camera.fx, 0.0,            cfg_.camera.cx,
                         0.0,            cfg_.camera.fy, cfg_.camera.cy,
                         0.0,            0.0,            1.0);

    dist_raw_ = cv::Mat(1, static_cast<int>(cfg_.camera.dist_coeffs.size()), CV_64F);
    for (size_t i = 0; i < cfg_.camera.dist_coeffs.size(); ++i)
        dist_raw_.at<double>(0, static_cast<int>(i)) = cfg_.camera.dist_coeffs[i];

    undistort_ = cfg_.camera.undistort;

    // Real values are settled in lazyInit once the frame size is known.
    K_solve_    = K_raw_;
    dist_solve_ = dist_raw_.clone();
    initialized_ = false;
    return true;
}

// ---------------------------------------------------------------------------

void Detector::lazyInit(const cv::Mat& gray)
{
    frame_w_ = gray.cols;
    frame_h_ = gray.rows;

    // Intrinsics scale linearly with resolution. Correct for a mismatch and log
    // it, since it silently biases every range otherwise.
    if (cfg_.camera.calib_width > 0 && cfg_.camera.calib_height > 0
        && (cfg_.camera.calib_width != frame_w_ || cfg_.camera.calib_height != frame_h_))
    {
        const double sx = static_cast<double>(frame_w_) / cfg_.camera.calib_width;
        const double sy = static_cast<double>(frame_h_) / cfg_.camera.calib_height;
        K_raw_(0, 0) *= sx;  K_raw_(0, 2) *= sx;
        K_raw_(1, 1) *= sy;  K_raw_(1, 2) *= sy;
        rescaled_ = true;
        std::cerr << "[detect/" << cfg_.camera_id << "] intrinsics calibrated at "
                  << cfg_.camera.calib_width << "x" << cfg_.camera.calib_height
                  << " but stream negotiated " << frame_w_ << "x" << frame_h_
                  << "; rescaled by (" << sx << ", " << sy << "). Recalibrate at the "
                     "runtime resolution if range accuracy matters.\n";
    }

    const bool fisheye = (cfg_.camera.distortion_model == "fisheye");

    if (!undistort_)
    {
        // Default path: detect on the raw frame and let solvePnP consume the
        // distortion coefficients.
        K_solve_    = K_raw_;
        dist_solve_ = dist_raw_.clone();
        initialized_ = true;
        return;
    }

    // Optional full-frame rectification, for a wide lens whose edge distortion
    // interferes with quad fitting. Built here because this is the first point
    // at which the resolution is known.
    const cv::Size sz(frame_w_, frame_h_);
    cv::Mat K_new;

    if (fisheye)
    {
        cv::Mat K_in;
        cv::Mat(K_raw_).convertTo(K_in, CV_64F);
        cv::fisheye::estimateNewCameraMatrixForUndistortRectify(
            K_in, dist_raw_, sz, cv::Mat::eye(3, 3, CV_64F), K_new, 0.0, sz, 1.0);
        cv::fisheye::initUndistortRectifyMap(
            K_in, dist_raw_, cv::Mat::eye(3, 3, CV_64F), K_new, sz, CV_16SC2, map1_, map2_);
    }
    else
    {
        cv::Mat K_in;
        cv::Mat(K_raw_).convertTo(K_in, CV_64F);
        // alpha = 0 crops to the all-valid region; a larger alpha would leave
        // black wedges for the detector to search.
        K_new = cv::getOptimalNewCameraMatrix(K_in, dist_raw_, sz, 0.0, sz);
        cv::initUndistortRectifyMap(
            K_in, dist_raw_, cv::Mat(), K_new, sz, CV_16SC2, map1_, map2_);
    }

    // The pose solve must use the rectified focal lengths. Distortion is zero
    // by construction after the remap.
    K_solve_ = cv::Matx33d(K_new.at<double>(0, 0), 0.0, K_new.at<double>(0, 2),
                           0.0, K_new.at<double>(1, 1), K_new.at<double>(1, 2),
                           0.0, 0.0, 1.0);
    dist_solve_ = cv::Mat::zeros(1, 5, CV_64F);

    std::cerr << "[detect/" << cfg_.camera_id << "] undistort ON ("
              << cfg_.camera.distortion_model << ") rectified fx="
              << K_solve_(0, 0) << " fy=" << K_solve_(1, 1)
              << " cx=" << K_solve_(0, 2) << " cy=" << K_solve_(1, 2) << "\n";

    initialized_ = true;
}

// ---------------------------------------------------------------------------

void Detector::detect(const cv::Mat& gray, std::vector<Detection>& out)
{
    out.clear();
    if (gray.empty()) return;

    if (!initialized_ || gray.cols != frame_w_ || gray.rows != frame_h_)
        lazyInit(gray);

    const cv::Mat* img = &gray;
    if (undistort_ && !map1_.empty())
    {
        cv::remap(gray, undistorted_, map1_, map2_, cv::INTER_LINEAR);
        img = &undistorted_;
    }

    corners_.clear();
    ids_.clear();
    aruco_.detect(*img, corners_, ids_);
    if (ids_.empty()) return;

    stats_.decoded += ids_.size();

    for (size_t i = 0; i < ids_.size(); ++i)
    {
        const int id = ids_[i];

        if (!cfg_.tags.accepts(id))
        {
            ++stats_.rejected_id;
            continue;
        }
        if (corners_[i].size() != 4) continue;

        const double size_m = cfg_.tags.sizeFor(id);
        if (size_m <= 0.0) continue;

        // Marker-frame corners, clockwise from top-left, matching the order
        // detectMarkers returns and the convention the SITL ArUco source uses.
        // For AprilTag families size_m is the outer edge of the black border,
        // not the white quiet zone.
        const float h = static_cast<float>(size_m) / 2.0f;
        obj_pts_ = {{-h, h, 0.0f}, {h, h, 0.0f}, {h, -h, 0.0f}, {-h, -h, 0.0f}};

        cv::Vec3d rvec, tvec;
        cv::Mat K;
        cv::Mat(K_solve_).convertTo(K, CV_64F);

        if (!cv::solvePnP(obj_pts_, corners_[i], K, dist_solve_, rvec, tvec,
                          false, cv::SOLVEPNP_IPPE_SQUARE))
        {
            ++stats_.rejected_pnp;
            continue;
        }

        // Range sanity: an implausible range indicates a mis-decode or a wrong
        // size_m rather than a real detection.
        const double range = tvec[2];
        if (!std::isfinite(range)
            || range < cfg_.detector.min_range_m
            || range > cfg_.detector.max_range_m)
        {
            ++stats_.rejected_range;
            continue;
        }

        // Reprojection error: project the solved pose back and compare against
        // the detected corners. Published whether or not it passes the gate,
        // because the consumer ranks candidates on it.
        reproj_.clear();
        cv::projectPoints(obj_pts_, rvec, tvec, K, dist_solve_, reproj_);

        double err_sum = 0.0;
        for (int c = 0; c < 4; ++c)
            err_sum += cv::norm(reproj_[c] - corners_[i][c]);
        const double reproj_err = err_sum / 4.0;

        if (cfg_.detector.max_reproj_error_px > 0.0
            && reproj_err > cfg_.detector.max_reproj_error_px)
        {
            ++stats_.rejected_reproj;
            continue;
        }

        Detection d;
        d.tag_id = id;
        d.family = cfg_.detector.family;
        d.size_m = static_cast<float>(size_m);
        d.t_tag_wrt_cam = tvec;
        d.reproj_error_px = static_cast<float>(reproj_err);

        cv::Mat R;
        cv::Rodrigues(rvec, R);
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 3; ++c)
                d.r_tag_to_cam(r, c) = R.at<double>(r, c);

        for (int c = 0; c < 4; ++c) d.corners[c] = corners_[i][c];

        ++stats_.accepted;
        out.push_back(d);
    }
}

// ---------------------------------------------------------------------------

Detector::Stats Detector::takeStats()
{
    const Stats s = stats_;
    stats_ = Stats{};
    return s;
}

}  // namespace fd
