#!/usr/bin/env python3
"""Render a synthetic clip of a fiducial at a KNOWN pose, plus ground truth.

Why this exists: the detector is a pure vision process with no telemetry input, so replaying a file gives bit-identical output. That makes it
possible to check the pose solve against arithmetic rather than against a tape
measure — which matters because size_m, intrinsics and range are multiplicatively
coupled, and a 10% error in any of them looks exactly like a correct answer.

It is also how you test the whole service on a laptop with no camera, and how you
verify a config change did not silently move the range.

Usage
-----
  ./make_test_video.py --out /tmp/tag.mp4
  ./make_test_video.py --out /tmp/tag.mp4 --family aruco_4x4_50 --tag-id 0
  ./make_test_video.py --print-marker /tmp/tag0.png --tag-id 0 --px 1200

Then point a config at it:
  gst: "filesrc location=/tmp/tag.mp4 ! decodebin ! videoconvert !
        video/x-raw,format=GRAY8 ! appsink name=sink drop=true max-buffers=2"

and compare the published range against /tmp/tag.mp4.truth.json.
"""

import argparse
import json
import os
import sys

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs opencv-python:  pip install opencv-python")

# Same family -> dictionary mapping the service uses (include/aruco_compat.h).
FAMILIES = {
    "aruco_4x4_50": cv2.aruco.DICT_4X4_50,
    "aruco_4x4_100": cv2.aruco.DICT_4X4_100,
    "aruco_4x4_250": cv2.aruco.DICT_4X4_250,
    "aruco_5x5_50": cv2.aruco.DICT_5X5_50,
    "aruco_6x6_250": cv2.aruco.DICT_6X6_250,
    "aruco_original": cv2.aruco.DICT_ARUCO_ORIGINAL,
    "apriltag_16h5": cv2.aruco.DICT_APRILTAG_16h5,
    "apriltag_25h9": cv2.aruco.DICT_APRILTAG_25h9,
    "apriltag_36h10": cv2.aruco.DICT_APRILTAG_36h10,
    "apriltag_36h11": cv2.aruco.DICT_APRILTAG_36h11,
}


def get_dictionary(name):
    did = FAMILIES[name]
    if hasattr(cv2.aruco, "getPredefinedDictionary"):
        return cv2.aruco.getPredefinedDictionary(did)
    return cv2.aruco.Dictionary_get(did)


def draw_marker(dictionary, tag_id, px):
    if hasattr(cv2.aruco, "generateImageMarker"):
        return cv2.aruco.generateImageMarker(dictionary, tag_id, px)
    return cv2.aruco.drawMarker(dictionary, tag_id, px)


def marker_with_quiet_zone(dictionary, tag_id, px, quiet_ratio=0.25):
    """Marker plus a white quiet zone.

    The quiet zone is NOT part of size_m — size_m is the outer edge of the black
    border. Rendering the quiet zone and then measuring to its edge
    is precisely how a 25% range error gets introduced, so this function keeps
    the two separable: it returns the image and the fraction of it that is the
    black border.
    """
    marker = draw_marker(dictionary, tag_id, px)
    pad = int(round(px * quiet_ratio))
    canvas = np.full((px + 2 * pad, px + 2 * pad), 255, np.uint8)
    canvas[pad:pad + px, pad:pad + px] = marker
    border_fraction = px / float(px + 2 * pad)
    return canvas, border_fraction


# A marker seen head-on is NOT rvec = 0.
#
# OpenCV's marker frame has +X right and +Y UP out of the marker image, while the
# camera optical frame has +Y DOWN. So a fronto-parallel marker facing the camera
# is a 180-degree rotation about X — diag(1, -1, -1) — and rvec = 0 renders the
# tag vertically mirrored. A mirrored tag still looks entirely plausible to a
# human (it is a blocky black-and-white square either way) and decodes as
# nothing at all, which makes this a genuinely nasty way to lose an afternoon.
#
# This is the same convention the service's solvePnP object points use, and the
# same one Nexus's GzArucoTagSource uses, so ground truth rendered here is
# directly comparable to what the service publishes.
R_FACING_CAMERA = np.diag([1.0, -1.0, -1.0])


def compose_rvec(tilt_rvec):
    """Head-on attitude with `tilt_rvec` applied on top, as a single rvec."""
    R_tilt, _ = cv2.Rodrigues(np.asarray(tilt_rvec, dtype=np.float64))
    rvec, _ = cv2.Rodrigues(R_FACING_CAMERA @ R_tilt)
    return rvec


def render_frame(marker_img, border_fraction, K, size_m, rvec, tvec, w, h):
    """Warp the marker into the image plane at a given pose.

    `rvec` is the FULL tag->camera rotation, so pass compose_rvec(tilt) rather
    than a bare tilt. The marker image's OUTER EDGE spans size_m / border_fraction
    in the world, because size_m measures the black border only.
    """
    full_m = size_m / border_fraction
    half = full_m / 2.0
    # Marker-frame corners of the full rendered image, clockwise from the
    # marker's own top-left, with +Y up. Same ordering as the service's
    # solvePnP object points.
    obj = np.array([[-half,  half, 0.0],
                    [ half,  half, 0.0],
                    [ half, -half, 0.0],
                    [-half, -half, 0.0]], dtype=np.float64)

    img_pts, _ = cv2.projectPoints(obj, rvec, tvec, K, np.zeros(5))
    img_pts = img_pts.reshape(-1, 2).astype(np.float32)

    mh, mw = marker_img.shape[:2]
    src = np.array([[0, 0], [mw - 1, 0], [mw - 1, mh - 1], [0, mh - 1]], np.float32)

    H = cv2.getPerspectiveTransform(src, img_pts)

    # Mid-grey background: pure white or black gives the detector an unrealistically
    # clean threshold and hides marginal tuning.
    frame = np.full((h, w), 128, np.uint8)
    warped = cv2.warpPerspective(marker_img, H, (w, h),
                                 flags=cv2.INTER_LINEAR,
                                 borderMode=cv2.BORDER_TRANSPARENT,
                                 dst=frame)
    return warped


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", default="/tmp/fiducial_test.mp4")
    ap.add_argument("--print-marker", default=None,
                    help="write a printable marker PNG and exit")
    ap.add_argument("--family", default="apriltag_36h11", choices=sorted(FAMILIES))
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--px", type=int, default=800, help="marker render size in pixels")
    ap.add_argument("--width", type=int, default=1280)
    ap.add_argument("--height", type=int, default=720)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--fx", type=float, default=1283.0)
    ap.add_argument("--fy", type=float, default=None, help="defaults to fx")
    ap.add_argument("--cx", type=float, default=None, help="defaults to width/2")
    ap.add_argument("--cy", type=float, default=None, help="defaults to height/2")
    ap.add_argument("--size-m", type=float, default=0.30,
                    help="marker edge, OUTER EDGE OF THE BLACK BORDER")
    ap.add_argument("--start-range", type=float, default=8.0)
    ap.add_argument("--end-range", type=float, default=0.8)
    ap.add_argument("--duration", type=float, default=10.0)
    ap.add_argument("--tilt-deg", type=float, default=10.0,
                    help="peak obliquity; 0 renders a fronto-parallel tag, which is "
                         "an unrealistically easy pose")
    ap.add_argument("--noise", type=float, default=3.0, help="gaussian pixel noise sigma")
    args = ap.parse_args()

    fy = args.fy if args.fy is not None else args.fx
    cx = args.cx if args.cx is not None else args.width / 2.0
    cy = args.cy if args.cy is not None else args.height / 2.0
    K = np.array([[args.fx, 0, cx], [0, fy, cy], [0, 0, 1]], np.float64)

    dictionary = get_dictionary(args.family)

    if args.print_marker:
        img, bf = marker_with_quiet_zone(dictionary, args.tag_id, args.px)
        cv2.imwrite(args.print_marker, img)
        print(f"wrote {args.print_marker}")
        print(f"  family={args.family} id={args.tag_id}")
        print(f"  black border is {bf*100:.1f}% of the image edge.")
        print(f"  To make size_m = {args.size_m} m, print so the BLACK SQUARE measures "
              f"{args.size_m*100:.1f} cm across —")
        print(f"  the whole white sheet will then be {args.size_m/bf*100:.1f} cm. "
              f"Measure the black, not the paper.")
        return 0

    marker, border_fraction = marker_with_quiet_zone(dictionary, args.tag_id, args.px)

    n_frames = int(args.duration * args.fps)
    fourcc = cv2.VideoWriter_fourcc(*"mp4v")
    vw = cv2.VideoWriter(args.out, fourcc, args.fps, (args.width, args.height), isColor=False)
    if not vw.isOpened():
        sys.exit(f"cannot open VideoWriter for {args.out}")

    truth = {
        "source": os.path.abspath(args.out),
        "family": args.family,
        "tag_id": args.tag_id,
        "size_m": args.size_m,
        "intrinsics": {"fx": args.fx, "fy": fy, "cx": cx, "cy": cy,
                       "width": args.width, "height": args.height},
        "note": "t_tag_wrt_cam is the exact pose used to render each frame, in the "
                "camera optical frame (+x right, +y down, +z forward). Frame indices "
                "are 0-based; the service's frame_id starts at 1.",
        "frames": [],
    }

    for i in range(n_frames):
        f = i / max(n_frames - 1, 1)
        z = args.start_range * (1 - f) + args.end_range * f
        # Lateral swing so the x/y components are non-trivial: a tag pinned to the
        # optical axis cannot reveal a sign error in either the solve or a
        # downstream transform.
        x = 0.25 * np.sin(2 * np.pi * 0.5 * i / args.fps) * (z / args.start_range + 0.2)
        y = 0.15 * np.cos(2 * np.pi * 0.3 * i / args.fps) * (z / args.start_range + 0.2)

        tilt = np.deg2rad(args.tilt_deg) * np.sin(2 * np.pi * 0.4 * i / args.fps)
        tilt_rvec = np.array([tilt, tilt * 0.6,
                              0.05 * np.sin(2 * np.pi * 0.2 * i / args.fps)])
        rvec = compose_rvec(tilt_rvec)
        tvec = np.array([x, y, z])

        frame = render_frame(marker, border_fraction, K, args.size_m, rvec, tvec,
                             args.width, args.height)
        if args.noise > 0:
            noise = np.random.normal(0, args.noise, frame.shape)
            frame = np.clip(frame.astype(np.float32) + noise, 0, 255).astype(np.uint8)

        vw.write(frame)
        R, _ = cv2.Rodrigues(rvec)
        truth["frames"].append({
            "frame": i,
            "t_tag_wrt_cam": [float(x), float(y), float(z)],
            "range_m": float(z),
            "r_tag_to_cam": [float(v) for v in R.flatten()],
        })

    vw.release()

    truth_path = args.out + ".truth.json"
    with open(truth_path, "w") as fh:
        json.dump(truth, fh, indent=2)

    print(f"wrote {args.out}  ({n_frames} frames, {args.width}x{args.height} @ {args.fps}fps)")
    print(f"wrote {truth_path}")
    print()
    print("Point a config's gst: at it —")
    print(f'  gst: "filesrc location={args.out} ! decodebin ! videoconvert !')
    print('        video/x-raw,format=GRAY8 ! appsink name=sink drop=true max-buffers=2"')
    print()
    print("and set the config to MATCH THE GROUND TRUTH, or the range will be wrong:")
    print(f"  camera.fx/fy = {args.fx}/{fy}, cx/cy = {cx}/{cy}")
    print(f"  tags.default_size_m = {args.size_m}")
    print(f"  detector.family = {args.family}")
    print(f"  range sweeps {args.start_range} m -> {args.end_range} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
