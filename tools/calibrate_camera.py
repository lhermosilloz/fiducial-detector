#!/usr/bin/env python3
"""Measure camera intrinsics and emit the `camera:` block for a stream config.

Everything in this service that produces a NUMBER depends on these values. A 10%
error in fx/fy is a 10% error in every range, and range_m is what Nexus ranks
landing candidates on. The placeholders shipped in config/*.yaml are computed from
a nominal FOV and are not a calibration — they exist so the service starts, not so
it is correct.

Headless by design: no GUI, no keypresses. It auto-captures over SSH, which is how
you will actually be using the board.

Usage
-----
  # Jetson, live, through the same pipeline shape the service uses
  ./calibrate_camera.py --gst "nvarguscamerasrc sensor-id=0 sensor-mode=1 !
      video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1 !
      nvvidconv ! video/x-raw,width=1280,height=720,format=BGRx !
      videoconvert ! video/x-raw,format=BGR ! appsink drop=true max-buffers=2"

  # Pi
  ./calibrate_camera.py --gst "libcamerasrc !
      video/x-raw,width=1280,height=720,format=NV12 ! videoconvert !
      video/x-raw,format=BGR ! appsink drop=true max-buffers=2"

  # From images you already shot
  ./calibrate_camera.py --images ./calib/*.jpg

  # Just tell me the field of view of each sensor mode (no board needed)
  ./calibrate_camera.py --fov-only --fx 1283 --width 1280

The board
---------
Default is a 9x6 INNER-CORNER chessboard (a 10x7 grid of squares) with 25 mm
squares. Print it on rigid, flat stock — a curled sheet calibrates the curl into
your intrinsics, and taped-to-a-wall paper is the single most common reason a
calibration comes out plausible but wrong. Measure a printed square with calipers
and pass the real number to --square-mm; printer scaling is routinely off by a
few percent, and that percentage lands directly in your range.
"""

import argparse
import glob
import math
import os
import sys
import time

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs OpenCV python bindings:  sudo apt-get install python3-opencv")


def report_fov(fx, fy, width, height):
    hfov = 2.0 * math.atan(width / (2.0 * fx))
    vfov = 2.0 * math.atan(height / (2.0 * fy))
    dfov = 2.0 * math.atan(math.hypot(width, height) / (2.0 * math.hypot(fx, fy) / math.sqrt(2)))
    print(f"  horizontal FOV : {math.degrees(hfov):.1f} deg")
    print(f"  vertical   FOV : {math.degrees(vfov):.1f} deg")
    print(f"  diagonal   FOV : {math.degrees(dfov):.1f} deg")
    return math.degrees(hfov)


def open_capture(gst):
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        sys.exit("cannot open the GStreamer pipeline.\n"
                 "  - check it ends in an appsink producing BGR\n"
                 "  - check OpenCV was built with GStreamer: "
                 "python3 -c \"import cv2; print(cv2.getBuildInformation())\" | grep -i gstreamer")
    return cap


def find_board(gray, pattern):
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_FAST_CHECK
    ok, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not ok:
        return False, None
    corners = cv2.cornerSubPix(
        gray, corners, (11, 11), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
    return True, corners


def grid_cell(corners, w, h, n=3):
    c = corners.reshape(-1, 2).mean(axis=0)
    return (min(int(c[0] / w * n), n - 1), min(int(c[1] / h * n), n - 1))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    src = ap.add_mutually_exclusive_group()
    src.add_argument("--gst", help="GStreamer pipeline ending in an appsink producing BGR")
    src.add_argument("--images", nargs="+", help="image files instead of live capture")
    ap.add_argument("--fov-only", action="store_true",
                    help="print the FOV implied by --fx/--width and exit")
    ap.add_argument("--fx", type=float, help="for --fov-only")
    ap.add_argument("--width", type=int, default=1280, help="for --fov-only")
    ap.add_argument("--height", type=int, default=720, help="for --fov-only")

    ap.add_argument("--cols", type=int, default=9, help="INNER corners across")
    ap.add_argument("--rows", type=int, default=6, help="INNER corners down")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="measured square edge in mm — measure it, do not assume")

    ap.add_argument("--frames", type=int, default=20, help="views to collect")
    ap.add_argument("--interval", type=float, default=1.0, help="seconds between accepted views")
    ap.add_argument("--fisheye", action="store_true",
                    help="use the fisheye model (very wide lenses)")
    ap.add_argument("--save-dir", default=None, help="also save accepted frames here")
    ap.add_argument("--camera-id", default="down", help="label for the emitted YAML")
    args = ap.parse_args()

    if args.fov_only:
        if not args.fx:
            sys.exit("--fov-only needs --fx")
        print(f"\nfx={args.fx} at {args.width}x{args.height}:")
        report_fov(args.fx, args.fx, args.width, args.height)
        print()
        return 0

    if not args.gst and not args.images:
        sys.exit("pass --gst <pipeline> or --images <files>")

    pattern = (args.cols, args.rows)
    square_m = args.square_mm / 1000.0

    # Object points in the board frame, in METRES. Everything downstream inherits
    # this unit, so a millimetre slip here is a 1000x range error, not a subtle one.
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= square_m

    obj_points, img_points = [], []
    cells_seen = set()
    img_size = None

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    print(f"\nboard {args.cols}x{args.rows} inner corners, {args.square_mm} mm squares")
    print(f"target: {args.frames} views\n")

    if args.images:
        files = []
        for pat in args.images:
            files.extend(sorted(glob.glob(pat)) if any(c in pat for c in "*?[") else [pat])
        for f in files:
            img = cv2.imread(f)
            if img is None:
                print(f"  skip (unreadable): {f}")
                continue
            gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
            img_size = gray.shape[::-1]
            ok, corners = find_board(gray, pattern)
            print(f"  {'FOUND' if ok else '  -  '}  {os.path.basename(f)}")
            if ok:
                obj_points.append(objp.copy())
                img_points.append(corners)
                cells_seen.add(grid_cell(corners, *img_size))
    else:
        cap = open_capture(args.gst)
        last_accept = 0.0
        last_center = None
        t0 = time.time()
        while len(obj_points) < args.frames:
            ok, frame = cap.read()
            if not ok:
                print("  capture returned no frame; retrying")
                time.sleep(0.1)
                if time.time() - t0 > 60 and not obj_points:
                    sys.exit("no frames in 60 s — check the pipeline")
                continue

            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
            img_size = gray.shape[::-1]

            now = time.time()
            if now - last_accept < args.interval:
                continue

            found, corners = find_board(gray, pattern)
            if not found:
                continue

            # Reject a view that barely moved. Twenty near-identical views
            # constrain nothing and produce a confident-looking, wrong result.
            center = corners.reshape(-1, 2).mean(axis=0)
            if last_center is not None and np.linalg.norm(center - last_center) < 40:
                continue

            obj_points.append(objp.copy())
            img_points.append(corners)
            cell = grid_cell(corners, *img_size)
            cells_seen.add(cell)
            last_center, last_accept = center, now

            if args.save_dir:
                cv2.imwrite(os.path.join(args.save_dir, f"calib_{len(obj_points):03d}.png"), frame)

            print(f"  [{len(obj_points):2d}/{args.frames}] cell={cell} "
                  f"coverage={len(cells_seen)}/9")

        cap.release()

    n = len(obj_points)
    if n < 6:
        sys.exit(f"\nonly {n} usable views — need at least 6, ideally {args.frames}")

    print(f"\ncollected {n} views, frame coverage {len(cells_seen)}/9 cells")
    if len(cells_seen) < 5:
        print("  WARNING: the board stayed in a small part of the frame. Distortion and")
        print("  the principal point are poorly constrained by this set. Re-shoot with")
        print("  the board in every corner and at several distances and tilts.")

    print("\ncalibrating...")
    if args.fisheye:
        objp_f = [o.reshape(-1, 1, 3) for o in obj_points]
        K = np.zeros((3, 3))
        D = np.zeros((4, 1))
        rms, K, D, _, _ = cv2.fisheye.calibrate(
            objp_f, img_points, img_size, K, D,
            flags=cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC | cv2.fisheye.CALIB_FIX_SKEW,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
        dist = D.flatten()
        model = "fisheye"
    else:
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            obj_points, img_points, img_size, None, None)
        dist = D.flatten()[:5]
        model = "pinhole"

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    w, h = img_size

    print(f"\nRMS reprojection error: {rms:.4f} px")
    if rms > 1.0:
        print("  POOR. Above ~1 px usually means a non-flat board, a wrong --square-mm,")
        print("  motion blur, or too few distinct views. Do not ship this.")
    elif rms > 0.5:
        print("  acceptable, but tighter is better for a landing.")
    else:
        print("  good.")

    print(f"\nresolution {w}x{h}")
    hfov = report_fov(fx, fy, w, h)

    # The FOV number is the one that settles which sensor mode you are really in.
    # A cropped mode reports a markedly narrower FOV than a binned one at the same
    # output resolution, and that is measurable rather than a matter of opinion.
    print("\n  Compare this against the other sensor mode's calibration. A CROPPED")
    print("  mode reports a narrower FOV than a binned one at the same output")
    print("  resolution — that is how you tell them apart without a datasheet.")
    if hfov < 45:
        print(f"  {hfov:.0f} deg is narrow for precision landing: at 1 m altitude this")
        print(f"  sees about {2*1.0*math.tan(math.radians(hfov/2)):.2f} m across, and the pad")
        print("  can leave frame exactly when you need it.")

    # Principal point far from centre is a real signal, usually of a cropped or
    # misaligned readout rather than of a bad calibration.
    off_x, off_y = abs(cx - w / 2) / w, abs(cy - h / 2) / h
    if off_x > 0.05 or off_y > 0.05:
        print(f"\n  NOTE: principal point is {off_x*100:.1f}%/{off_y*100:.1f}% off centre "
              f"({cx:.1f}, {cy:.1f} vs {w/2:.0f}, {h/2:.0f}).")
        print("  Mild is normal; large usually means an off-centre sensor crop.")

    coeffs = ", ".join(f"{v:.6f}" for v in dist)
    print(f"""
------------------------------------------------------------------
Paste into the stream's `camera:` block (config/*.yaml):

    camera:
      fx: {fx:.4f}
      fy: {fy:.4f}
      cx: {cx:.4f}
      cy: {cy:.4f}
      distortion_model: {model}
      dist_coeffs: [{coeffs}]
      calib_width: {w}
      calib_height: {h}
      undistort: false
------------------------------------------------------------------

calib_width/calib_height record what this was measured at. If the stream ever
negotiates a different size the service rescales fx/fy/cx/cy and logs it — but
re-calibrating at the runtime resolution is better than relying on that.

These intrinsics belong to THIS camera and THIS lens at THIS sensor mode.
Re-run after any lens swap, refocus, or sensor-mode change.
""")
    return 0


if __name__ == "__main__":
    sys.exit(main())
