#!/usr/bin/env python3
"""Measure camera intrinsics and emit the `camera:` block for a stream config.

Everything in this service that produces a NUMBER depends on these values. A 10%
error in fx/fy is a 10% error in every range, and range_m is what Nexus ranks
landing candidates on. The placeholders shipped in config/*.yaml are computed from
a nominal FOV and are not a calibration — they exist so the service starts, not so
it is correct.

Guided capture
--------------
This does not collect twenty frames of whatever the board happened to be doing.
It works through a QUEUE OF TARGET BOXES, one at a time, the way ModalAI's
voxl-camera-calibration does. Each target is a rectangle in the frame plus a fill
threshold and a sample count. A view is accepted only when

  * every detected corner lies inside the current box, and
  * the board's quadrilateral covers at least the fill fraction of that box.

You are therefore forced to put the board in the corners, at the edges, tilted,
and near enough to fill the frame. That is what constrains distortion and the
principal point. An unguided grab-twenty-frames run lets you satisfy the frame
count without ever leaving the middle of the image, and the result looks
confident and is wrong.

Before the detector runs, the current box is adaptively thresholded — a 3xN grid
of per-tile Otsu cutoffs, bilinearly interpolated across the box, with a one-pixel
white border so a board overhanging the box edge still reads as a complete board.
This is ModalAI's `mcv_threshold_partial` and it matters: it survives the lens
shading and uneven lighting that make `CALIB_CB_NORMALIZE_IMAGE` drop detections
exactly where you need them, at the edges of the frame. Corner refinement then
runs on the RAW pixels, never on the thresholded ones.

Watching it work
----------------
The board is headless, so the overlay is served as MJPEG over HTTP. SSH in, start
the tool, and open the printed URL in a browser on your laptop:

    http://<board-ip>:8080

The target box fades red to green as the board approaches the fill threshold, and
the bar across the top is total progress. This is the same feedback loop ModalAI
gets from voxl-portal.

Usage
-----
  # IMX219 on the Jetson, mode 3 — the mode config/jetson_down.yaml pins.
  # Builds the pipeline and seeds the intrinsics guess for you.
  ./tools/calibrate_camera.py --sensor imx219 --mode 3 --square-mm 24.8

  # Any other camera: give the pipeline yourself. It must end in an appsink
  # producing BGR (or GRAY8).
  ./tools/calibrate_camera.py --gst "libcamerasrc !
      video/x-raw,width=1280,height=720,format=NV12 ! videoconvert !
      video/x-raw,format=BGR ! appsink drop=true max-buffers=2"

  Copying a pipeline out of config/*.yaml works, with one wrinkle handled for
  you: those name the sink `sink`, because the service looks it up by that name,
  while OpenCV only recognises a sink whose name contains "appsink" or
  "opencvsink" and otherwise reports "cannot find appsink in manual pipeline" —
  which sounds like the element is missing rather than misnamed. The sink is
  renamed automatically and the change is printed.

  # From images you already shot (no guided capture, no preview)
  ./tools/calibrate_camera.py --images ./calib/*.jpg

  # Replay a recorded clip through the guided loop — useful for testing the
  # tool itself without a camera attached
  ./tools/calibrate_camera.py --video /tmp/board.mp4

  # Just tell me the field of view of each sensor mode (no board needed)
  ./tools/calibrate_camera.py --fov-only --fx 1357 --width 1640

  # What modes does this sensor have?
  ./tools/calibrate_camera.py --list-modes

While it runs, on stdin: ENTER skips the current target (use it when a target box
is physically unreachable — an obstructed corner, a lens hood), and `stop` ends
sampling early and calibrates on what has been collected.

The board
---------
Default is a 9x6 INNER-CORNER chessboard (a 10x7 grid of squares) with 25 mm
squares, but 25 mm is small for this workflow and you should print bigger.

FLATNESS is the thing that decides whether a calibration is any good. The solver
is told the corners are coplanar, so every millimetre they are not goes straight
into the fit, and no number of extra views averages it out. Measured on a 200 mm
board:

    bow 1 mm (0.5% of width)  ->  ~0.19 px median error
    bow 2 mm (1.0%)           ->  ~0.39 px
    bow 5 mm (2.5%)           ->  ~0.96 px, and fx wrong by ~4%

Only the RATIO of bow to board width matters, so keep it under about 0.5% of the
width. Paper taped to a wall does not manage that; paper spray-mounted to glass,
MDF or aluminium composite does. Note this is invisible in the preview — a bent
board looks perfectly sharp.

Defocus and straight-line motion blur are, perhaps surprisingly, NOT major
causes: both are symmetric and leave a chessboard saddle point where it was.
Rolling-shutter shear from a board moving during exposure is asymmetric and does
bias corners, so hold still.

SIZE matters because the fill thresholds fix a working distance: a given fraction
of a given box can only be satisfied from one range. 25 mm squares put an IMX219
at about 0.30 m, which is cramped and leaves little room to tilt; 50-65 mm puts
it at a comfortable 0.5-0.9 m. ModalAI's default is 65.5 mm. The tool computes
this at startup and says so.

Measure a printed square with calipers and pass the real number to --square-mm.
Printer scaling is routinely off by a few percent. This does NOT affect fx, which
is set by perspective rather than by absolute scale, but it does scale every
distance the service derives.
"""

import argparse
import glob
import math
import os
import re
import subprocess
import sys
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

import numpy as np

try:
    import cv2
except ImportError:
    sys.exit("needs OpenCV python bindings:  sudo apt-get install python3-opencv")


# Reprojection error above which the calibration is called a failure rather than
# a result. ModalAI's cutoffs, and they are not arbitrary: past these values the
# usual cause is motion blur or a non-flat board, both of which produce a fit
# that is precise and biased.
PLUMB_ERROR_CUTOFF = 0.75
FISHEYE_ERROR_CUTOFF = 0.60

# Corner refinement window, in pixels, on the raw image.
SUBPIX_WINDOW = 5

# How long to let the camera shut down before giving up on it. See the release
# call in main() for why this exists at all.
RELEASE_TIMEOUT = 5.0

# Nudges every tile threshold up or down. 0 suits a normal visible-light sensor.
# Raise it for a washed-out or low-contrast image, where the isodata cutoff lands
# too low and the black squares bleed into the background.
#
# Note this is NOT thermal support. ModalAI's thermal mode also swaps the
# chessboard for a circle grid, which is the part that matters: a printed
# chessboard has no thermal contrast at all, so a thermal calibration needs a
# target with actual temperature differences. Only the threshold bias is ported.
THRESH_ADJUSTMENT = 0

# BGR, because that is what cv2.imencode wants for the MJPEG stream.
GREEN = (72, 216, 76)
RED = (0, 0, 255)
WHITE = (255, 255, 255)


# ---------------------------------------------------------------------------
# Sensor presets
# ---------------------------------------------------------------------------
#
# fx_guess is a focal length in PIXELS at that mode's output resolution, derived
# from sensor geometry rather than measured. It seeds CALIB_USE_INTRINSIC_GUESS,
# which keeps the optimiser out of the local minima it otherwise falls into when
# the distortion is strong. It is a starting point, never a result.
#
# For the IMX219 the geometry is: 3280x2464 array, 1.12 um pixels, 3.04 mm lens.
#   full resolution  fx = 3040 um / 1.12 um = 2714 px
#   2x2 binned       fx = 3040 um / 2.24 um = 1357 px
# A CROP does not change the focal length in pixels, it only shows less of the
# image circle — which is exactly why the cropped modes have a narrower field of
# view at the same fx. That is the whole argument for mode 3 in
# config/jetson_down.yaml.

SENSORS = {
    "imx219": {
        "desc": "Raspberry Pi Camera v2 — 3280x2464, 1.12 um px, fixed 3.04 mm lens",
        "modes": {
            0: dict(w=3280, h=2464, fps=21, fx=2714.0, px_um=1.12, kind="full array"),
            1: dict(w=3280, h=1848, fps=28, fx=2714.0, px_um=1.12, kind="vertical crop"),
            2: dict(w=1920, h=1080, fps=30, fx=2714.0, px_um=1.12, kind="centre crop"),
            3: dict(w=1640, h=1232, fps=30, fx=1357.0, px_um=2.24, kind="2x2 binned, full FOV"),
            4: dict(w=1280, h=720, fps=59, fx=2714.0, px_um=1.12, kind="crop"),
        },
        "default_mode": 3,
    },
    # Interchangeable C/CS-mount lens, so there is no focal length to derive.
    # The pipeline is still worth having; fx is left to the optimiser.
    "imx477": {
        "desc": "Raspberry Pi HQ camera — 4056x3040, 1.55 um px, C/CS-mount lens",
        "modes": {
            0: dict(w=4056, h=3040, fps=10, fx=None, px_um=1.55, kind="full array"),
            1: dict(w=2028, h=1520, fps=40, fx=None, px_um=3.10, kind="2x2 binned, full FOV"),
            2: dict(w=2028, h=1080, fps=50, fx=None, px_um=3.10, kind="binned + vertical crop"),
            3: dict(w=1332, h=990, fps=120, fx=None, px_um=1.55, kind="crop"),
        },
        "default_mode": 1,
    },
}


def detect_platform():
    """Same test check_camera.sh uses, so the two tools never disagree."""
    if os.path.exists("/etc/nv_tegra_release"):
        return "jetson"
    try:
        with open("/proc/device-tree/model", "rb") as f:
            if b"raspberry" in f.read().lower():
                return "pi"
    except OSError:
        pass
    return "generic"


# OpenCV's GStreamer backend does not take the sink you hand it. It parses the
# pipeline and then scans for an element whose NAME contains "appsink" or
# "opencvsink" (modules/videoio/src/cap_gstreamer.cpp). An appsink named anything
# else is invisible to it, and the failure is the thoroughly unhelpful
#
#     GStreamer warning: cannot find appsink in manual pipeline
#
# which reads like the element is missing rather than misnamed.
#
# This is a trap rather than a detail, because the service's own configs name it
# `sink` — the C++ side looks it up with gst_bin_get_by_name(pipeline, "sink") —
# and the documented way to calibrate is to reuse the pipeline the service runs.
# The same element, with two incompatible lookup rules. An unnamed appsink is
# fine either way: GStreamer auto-names it "appsink0".
OPENCV_SINK_NAME = "opencvsink"


def normalise_appsink(pipeline):
    """Rename the appsink if OpenCV would not find it.

    Returns (pipeline, note) where note is None if nothing needed changing.
    Operates only on the appsink element, located by splitting on '!', rather
    than by pattern-matching the whole pipeline string — a `name=` belonging to
    some other element must not be touched.
    """
    parts = pipeline.split("!")
    for i, part in enumerate(parts):
        if part.strip().split()[:1] != ["appsink"]:
            continue
        m = re.search(r"\bname\s*=\s*(\S+)", part)
        if m is None:
            return pipeline, None          # unnamed: GStreamer calls it appsink0
        current = m.group(1)
        if "appsink" in current or "opencvsink" in current:
            return pipeline, None
        parts[i] = part[:m.start(1)] + OPENCV_SINK_NAME + part[m.end(1):]
        return "!".join(parts), (
            f"  note: renamed the appsink from '{current}' to "
            f"'{OPENCV_SINK_NAME}'.\n"
            f"        OpenCV locates the sink by name and ignores one called "
            f"'{current}'; the\n"
            f"        service itself requires 'sink', so a pipeline copied from "
            f"config/*.yaml\n"
            f"        needs exactly this one change.")
    return pipeline, None


def build_pipeline(platform, mode_info, sensor_mode, sensor_id=0):
    """A pipeline in the shape of the service's, but ending in BGR.

    The service asks for NV12 and reads plane 0 as grayscale at zero copy. Here
    we go through videoconvert to BGR instead: OpenCV's VideoCapture will not
    hand us NV12, and the conversion cost is irrelevant for a one-off calibration
    where the chessboard search dominates anyway.

    What must NOT change from the service's pipeline is the sensor mode and the
    output resolution. Intrinsics belong to a mode, not to a camera.

    The appsink's NAME differs from the service's, and it has to — see
    OPENCV_SINK_NAME.
    """
    w, h, fps = mode_info["w"], mode_info["h"], mode_info["fps"]
    if platform == "jetson":
        return (
            f"nvarguscamerasrc sensor-id={sensor_id} sensor-mode={sensor_mode} ! "
            f"video/x-raw(memory:NVMM),width={w},height={h},framerate={fps}/1 ! "
            f"nvvidconv ! video/x-raw,format=BGRx ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink name={OPENCV_SINK_NAME} drop=true max-buffers=2 sync=false"
        )
    if platform == "pi":
        return (
            f"libcamerasrc ! "
            f"video/x-raw,width={w},height={h},framerate={fps}/1,format=NV12 ! "
            f"videoconvert ! video/x-raw,format=BGR ! "
            f"appsink name={OPENCV_SINK_NAME} drop=true max-buffers=2 sync=false"
        )
    return (
        f"v4l2src device=/dev/video{sensor_id} ! "
        f"video/x-raw,width={w},height={h} ! "
        f"videoconvert ! video/x-raw,format=BGR ! "
        f"appsink name={OPENCV_SINK_NAME} drop=true max-buffers=2 sync=false"
    )


def _unit_active(name):
    try:
        return subprocess.run(["systemctl", "is-active", "--quiet", name],
                              timeout=5).returncode == 0
    except (OSError, subprocess.SubprocessError):
        return False        # no systemd, or systemctl unavailable: not a finding


def _other_argus_consumers():
    """Processes likely to be holding a camera, excluding this one."""
    try:
        out = subprocess.run(["ps", "-eo", "pid=,args="], capture_output=True,
                             text=True, timeout=5).stdout
    except (OSError, subprocess.SubprocessError):
        return []
    me = str(os.getpid())
    hits = []
    for line in out.splitlines():
        pid, _, args = line.strip().partition(" ")
        if pid == me or "calibrate_camera" in args:
            continue
        if any(k in args for k in ("gst-launch", "nvgstcapture", "argus_camera",
                                   "fiducial-detector-service", "nvargus_nvraw")):
            hits.append(f"{pid} {args[:70]}")
    return hits


def camera_preflight(platform):
    """Report what will stop Argus BEFORE sitting in a read loop waiting on it.

    Argus permits exactly one client per sensor, and says so only as
    `Failed to create CaptureSession` — printed well after the pipeline has been
    built and reported as successfully opened. OpenCV then simply returns no
    frames, and nothing on screen connects the two.

    The service is the overwhelmingly likely holder: it is installed on this same
    board, it is configured for sensor-id=0, and a calibration run gives it no
    reason to release anything.
    """
    notes = []
    if _unit_active("fiducial-detector-service"):
        notes.append(
            "fiducial-detector-service is RUNNING and holds the camera.\n"
            "    Argus allows one client per sensor, so calibration cannot open it:\n"
            "        sudo systemctl stop fiducial-detector-service\n"
            "    Start it again when you are done.")
    if platform == "jetson" and not _unit_active("nvargus-daemon"):
        notes.append(
            "nvargus-daemon is NOT active; nvarguscamerasrc cannot work without it:\n"
            "        sudo systemctl restart nvargus-daemon")
    others = _other_argus_consumers()
    if others:
        notes.append("another process may be holding the camera:\n" +
                     "\n".join(f"        {h}" for h in others))
    return notes


def argus_failure_advice(platform):
    """What to say when the pipeline opened but no frame ever arrived."""
    if platform != "jetson":
        return "  - run tools/check_camera.sh --capture to settle the ingest path"
    return (
        "  If Argus printed 'Failed to create CaptureSession', the daemon WAS\n"
        "  reachable but the sensor could not be acquired. That is a busy or stale\n"
        "  sensor — not a missing driver, and not a bad pipeline. In order of\n"
        "  likelihood:\n"
        "    1. the service holds it:  sudo systemctl stop fiducial-detector-service\n"
        "    2. a stale Argus session: sudo systemctl restart nvargus-daemon\n"
        "    3. another consumer:      ps aux | grep -E 'gst-launch|argus'\n"
        "    4. wrong --sensor-id, or that mode is unsupported:\n"
        "                              ./tools/check_camera.sh --capture")


def closest_working_distance(targets, width, height, cols, rows, square_m, fx):
    """Metres from lens to board at the most demanding target.

    The fill thresholds do not just ask for a position, they fix a DISTANCE: a
    given fraction of a given box, with a board of a given physical size, can only
    be satisfied from one range. Board size is therefore not a free choice, and it
    is the one parameter people pick by what their printer will do.
    """
    aspect = (cols - 1) / max(rows - 1, 1)
    span_m = (cols - 1) * square_m
    best = None
    for x1, y1, x2, y2, fill, _n in targets:
        bw = (x2 - x1) * width // 100
        bh = (y2 - y1) * height // 100
        inner_px = math.sqrt((fill / 100.0) * bw * bh * aspect)
        if inner_px <= 0:
            continue
        z = fx * span_m / inner_px
        best = z if best is None else min(best, z)
    return best


# Below this a fixed-focus module is working inside its near limit. The IMX219 in
# its usual fixed-focus form is set around a metre and is visibly soft at a third
# of one; the corners then localise poorly in a way that looks exactly like a
# systematically bad calibration, because that is what it is.
NEAR_FOCUS_LIMIT_M = 0.50


def warn_working_distance(targets, width, height, cols, rows, square_mm, fx):
    z = closest_working_distance(targets, width, height, cols, rows,
                                 square_mm / 1000.0, fx)
    if z is None or z >= NEAR_FOCUS_LIMIT_M:
        return
    need = square_mm * NEAR_FOCUS_LIMIT_M / z
    print(f"\n  NOTE: {square_mm:.0f} mm squares force the board to {z:.2f} m at the")
    print(f"  tightest target, inside the ~{NEAR_FOCUS_LIMIT_M:.2f} m near limit of a "
          f"typical fixed-focus module.")
    print("  Uniform softness on its own is fairly harmless — a defocused chessboard")
    print("  corner stays put. The problem at close range is depth of field across a")
    print("  TILTED board, where one end defocuses more than the other, and that")
    print("  asymmetry does move corners. Close range also leaves little room to")
    print("  tilt at all.")
    print(f"  Print the board at about {need:.0f} mm squares (or larger) to work at "
          f"{NEAR_FOCUS_LIMIT_M:.2f} m+.")
    print("  ModalAI's own default is 65.5 mm for this reason. If your camera focuses")
    print(f"  closer than {z:.2f} m, ignore this.\n")


def print_mode_table():
    for name, s in SENSORS.items():
        print(f"\n{name} — {s['desc']}")
        print("  mode  resolution     fps   fx guess   notes")
        for m, i in sorted(s["modes"].items()):
            fx = f"{i['fx']:.0f} px" if i["fx"] else "  (lens)"
            star = "  <-- default" if m == s["default_mode"] else ""
            print(f"   {m}    {i['w']}x{i['h']:<6} {i['fps']:>4}   {fx:>8}   "
                  f"{i['kind']}{star}")
    print("\nA cropped mode has the same fx as the full array and a NARROWER field")
    print("of view. A binned mode halves fx and keeps the full field of view. At 1 m")
    print("altitude that is the difference between seeing 1.2 m across and 0.5 m,")
    print("which is the difference between holding the pad and losing it on short")
    print("final. Calibrate the mode you will actually fly.\n")


# ---------------------------------------------------------------------------
# Adaptive thresholding — port of ModalAI's mcv_threshold_partial
# ---------------------------------------------------------------------------

def _tile_threshold(tile, adjustment):
    """Iterative-isodata cutoff for one tile, capped at 10 passes.

    This is the cheap approximation to Otsu that ModalAI uses: start at the mean,
    then repeatedly split into the two class means and take the midpoint. It
    converges in three or four passes on a chessboard tile, where the histogram is
    genuinely bimodal, and the iteration cap bounds the cost on tiles where it is
    not.
    """
    flat = tile.ravel()
    total = flat.size
    if total == 0:
        return 128
    bins = np.bincount(flat, minlength=256).astype(np.int64)
    cum_n = np.cumsum(bins)
    cum_w = np.cumsum(np.arange(256, dtype=np.int64) * bins)
    prediction = int(cum_w[255] // total)

    for _ in range(10):
        p = int(min(max(prediction, 1), 255))
        n1 = int(cum_n[p - 1])
        n2 = total - n1
        if n1 == 0 or n2 == 0:
            break
        avg1 = int(cum_w[p - 1]) // n1
        avg2 = (int(cum_w[255]) - int(cum_w[p - 1])) // n2
        old = prediction
        prediction = (avg1 + avg2) // 2
        if abs(old - prediction) <= 2:
            break

    return int(min(max(prediction + adjustment, 5), 250))


def threshold_box(gray, box, adjustment=THRESH_ADJUSTMENT):
    """Binarise one region of the frame; leave the rest as it came.

    Thresholding a region rather than the whole frame is the point. A single
    global cutoff cannot serve both a board filling a bright centre and a board in
    a vignetted corner, and per-frame full-image adaptive thresholding costs more
    than the chessboard search. Restricting to a region means the tiles all see
    roughly the same illumination as the board itself.

    `box` is the THRESHOLD region, which is deliberately larger than the target
    box the board has to sit in — see TargetQueue.thresh_box for why.

    Returns a new uint8 image, binary inside the region.
    """
    x, y, w, h = box
    # Clip to the frame. Padding routinely pushes a region past an edge, and a
    # negative origin would silently wrap the slices below.
    fh, fw = gray.shape[:2]
    x2, y2 = min(x + w, fw), min(y + h, fh)
    x, y = max(x, 0), max(y, 0)
    w, h = x2 - x, y2 - y
    out = gray.copy()
    if w < 8 or h < 8:
        return out

    # Roughly square tiles: the long edge gets 3, the short edge gets however
    # many fit. Fewer than 3 in one axis is normal for a wide, short target.
    tmax = 3
    if w >= h:
        nx = tmax
        ny = max(1, int(round(h / (w / nx))))
    else:
        ny = tmax
        nx = max(1, int(round(w / (h / ny))))

    tile_w, tile_h = w // nx, h // ny
    if tile_w < 2 or tile_h < 2:
        return out

    # A (ny+2, nx+2) grid: the interior holds one cutoff per tile at the tile
    # centre, and the ring around it is extrapolated so the interpolation below
    # covers the box edges instead of clamping there.
    q = np.zeros((ny + 2, nx + 2), np.float64)
    qx = np.zeros(nx + 2, np.float64)
    qy = np.zeros(ny + 2, np.float64)

    for i in range(ny):
        ty = y + i * tile_h
        qy[i + 1] = ty + tile_h / 2.0
        for j in range(nx):
            tx = x + j * tile_w
            qx[j + 1] = tx + tile_w / 2.0
            q[i + 1, j + 1] = _tile_threshold(
                gray[ty:ty + tile_h, tx:tx + tile_w], adjustment)

    qx[0], qx[nx + 1] = qx[1] - tile_w, qx[nx] + tile_w
    qy[0], qy[ny + 1] = qy[1] - tile_h, qy[ny] + tile_h

    # Linear extrapolation outward, so lens shading that is still darkening at
    # the last tile keeps darkening past it rather than flattening off.
    if nx == 1:
        q[1:ny + 1, 0] = q[1:ny + 1, 1]
        q[1:ny + 1, nx + 1] = q[1:ny + 1, nx]
    else:
        q[1:ny + 1, 0] = 2 * q[1:ny + 1, 1] - q[1:ny + 1, 2]
        q[1:ny + 1, nx + 1] = 2 * q[1:ny + 1, nx] - q[1:ny + 1, nx - 1]
    if ny == 1:
        q[0, 1:nx + 1] = q[1, 1:nx + 1]
        q[ny + 1, 1:nx + 1] = q[ny, 1:nx + 1]
    else:
        q[0, 1:nx + 1] = 2 * q[1, 1:nx + 1] - q[2, 1:nx + 1]
        q[ny + 1, 1:nx + 1] = 2 * q[ny, 1:nx + 1] - q[ny - 1, 1:nx + 1]
    np.clip(q, 5, 250, out=q)

    # Corners have no single direction to extrapolate along, so average the two
    # neighbours and bias down: the corners are the most vignetted part of the
    # frame and a cutoff that is too high loses the black squares entirely.
    q[0, 0] = (q[0, 1] + q[1, 0]) / 2 - 12
    q[0, nx + 1] = (q[0, nx] + q[1, nx + 1]) / 2 - 12
    q[ny + 1, 0] = (q[ny, 0] + q[ny + 1, 1]) / 2 - 12
    q[ny + 1, nx + 1] = (q[ny + 1, nx] + q[ny, nx + 1]) / 2 - 12
    np.clip(q, 5, 250, out=q)

    # Bilinear interpolation of the coarse grid up to per-pixel cutoffs. Done as
    # two separable passes; a per-tile step function instead would put a visible
    # seam through the board wherever a tile boundary crosses it, and that seam
    # lands in the corner positions.
    xs = np.arange(x, x + w)
    ys = np.arange(y, y + h)
    rows = np.empty((ny + 2, w), np.float64)
    for i in range(ny + 2):
        rows[i] = np.interp(xs, qx, q[i])

    li = np.clip(np.searchsorted(qy, ys, side="right") - 1, 0, ny)
    span = qy[li + 1] - qy[li]
    t = ((ys - qy[li]) / span)[:, None]
    thresh = rows[li] * (1.0 - t) + rows[li + 1] * t

    region = gray[y:y + h, x:x + w]
    out[y:y + h, x:x + w] = np.where(region >= thresh, 255, 0).astype(np.uint8)

    # One-pixel white border. findChessboardCorners needs a quiet zone around the
    # board, and a board deliberately pushed to fill the box does not have one
    # inside the box. Without this the fill thresholds are unreachable and the
    # guided run stalls on the corner targets.
    out[y, x:x + w] = 255
    out[y + h - 1, x:x + w] = 255
    out[y:y + h, x] = 255
    out[y:y + h, x + w - 1] = 255
    return out


# ---------------------------------------------------------------------------
# Target sets — where in the frame the board has to go, and how much of it to fill
# ---------------------------------------------------------------------------
#
# Each entry is (x1, y1, x2, y2, fill_pct, n_samples) with the rectangle in
# percent of the frame from the top left. fill_pct is the board quadrilateral's
# area as a percentage of the box area.

TARGETS_STANDARD = [
    # 2x2 grid over the whole frame, overlapping in the middle
    (0, 0, 60, 60, 45, 2),
    (40, 0, 100, 60, 45, 2),
    (40, 40, 100, 100, 45, 2),
    (0, 40, 60, 100, 45, 2),
    (15, 10, 85, 90, 45, 3),      # centre, filling most of the frame
    # Vertical skew — the board tilted top-away and bottom-away. Without these
    # fy and the tangential terms are barely observed.
    (25, 10, 75, 45, 30, 2),
    (25, 55, 75, 90, 30, 2),
    # Full-height slabs left, centre, right
    (0, 0, 50, 100, 45, 2),
    (25, 0, 75, 100, 45, 2),
    (50, 0, 100, 100, 45, 2),
]

# Fisheye needs the board further into the corners and tolerates less fill,
# because at the edge of a fisheye frame the board is compressed into a smaller
# area than its geometry suggests.
TARGETS_FISHEYE = [
    (0, 0, 70, 60, 45, 3),
    (30, 0, 100, 60, 45, 3),
    (30, 40, 100, 100, 45, 3),
    (0, 40, 70, 100, 45, 3),
    (15, 10, 85, 90, 50, 3),
]

TARGETS_FISHEYE_WIDE = [
    (0, 0, 50, 60, 40, 3),
    (50, 0, 100, 60, 40, 3),
    (50, 40, 100, 100, 40, 3),
    (0, 40, 50, 100, 40, 3),
    (15, 10, 85, 90, 45, 3),
]


def get_targets(width, height, fisheye):
    if not fisheye:
        return list(TARGETS_STANDARD)
    # A 4:3-ish fisheye frame is the old VGA shape and wants the gentler set; a
    # wide one has enough frame to push the board properly into the corners.
    return list(TARGETS_FISHEYE if (width / height) < 1.45 else TARGETS_FISHEYE_WIDE)


# Leave this much headroom under the geometric maximum. A board is never
# perfectly aligned with the box, and any tilt — which the skew targets exist to
# produce — shrinks its quadrilateral further.
FILL_HEADROOM = 0.85

# Below this a target constrains nothing: the board can sit in one corner of the
# box and still pass, which is the unguided behaviour this tool exists to avoid.
FILL_FLOOR = 15


def _axis_limit(b1, b2, frame, c):
    """Largest inner-corner span this axis allows, in pixels.

    Two containments have to hold at once, and they pull against each other:

        the inner corners must lie inside the TARGET BOX, which is what the fill
        fraction is measured against and what the operator is aiming at;

        the whole physical board must lie inside the FRAME, and the board reaches
        one square (a fraction c of the inner span) past its corners on each side.

    For a box flush against a frame edge those are directly opposed: pushing the
    board in far enough to keep its outer ring on-screen pushes its corners out of
    the box. The span below is the largest one for which some placement satisfies
    both; past it no placement exists and the target can never be completed.
    """
    limits = [
        b2 - b1,                     # inner span fits the box at all
        (frame - b1) / (1.0 + c),    # pushed to the box's near edge, outer in frame
        b2 / (1.0 + c),              # pushed to the box's far edge, outer in frame
        frame / (1.0 + 2 * c),       # outer board fits the frame at all
    ]
    return max(min(limits), 0.0)


def max_fill_pct(rect, frame_w, frame_h, cols, rows):
    """Largest fraction of the box a board can cover and still be detectable.

    Fronto-parallel and perfectly placed — the true achievable value is lower,
    which is what FILL_HEADROOM is for.
    """
    x1, y1, x2, y2 = rect
    box_w, box_h = x2 - x1, y2 - y1
    if box_w <= 0 or box_h <= 0:
        return 0.0
    # One square as a fraction of the inner span, per axis.
    cx = 1.0 / max(cols - 1, 1)
    cy = 1.0 / max(rows - 1, 1)
    aspect = (cols - 1) / max(rows - 1, 1)

    sx = _axis_limit(x1, x2, frame_w, cx)
    sy = _axis_limit(y1, y2, frame_h, cy)

    # The board scales rigidly, so the binding axis wins.
    span = min(sx, aspect * sy)
    return 100.0 * (span * span / aspect) / (box_w * box_h)


def fit_targets(targets, width, height, cols, rows, verbose=True):
    """Clamp fill thresholds to what this board can physically achieve.

    ModalAI's numbers assume their default 5x6 board, whose inner-corner span is
    TALLER than it is wide. This repo defaults to 9x6, which is wider than it is
    tall, and the full-height slab targets then demand a board wider than the box
    itself. Nothing in the loop can detect that: the operator just moves the board
    around forever while the fill percentage sits below the threshold.

    So the thresholds are derived rather than fixed. The target GEOMETRY — where in
    the frame the board has to go — is what carries the calibration value, and that
    is preserved; only the fill bar moves, and only down.
    """
    out, notes = [], []
    for i, (x1, y1, x2, y2, fill, n) in enumerate(targets):
        rect = (x1 * width // 100, y1 * height // 100,
                x2 * width // 100, y2 * height // 100)
        bw, bh = rect[2] - rect[0], rect[3] - rect[1]
        reachable = max_fill_pct(rect, width, height, cols, rows) * FILL_HEADROOM
        if reachable < FILL_FLOOR:
            notes.append(f"  target {i + 1} dropped: a {cols}x{rows} board can cover "
                         f"at most {reachable / FILL_HEADROOM:.0f}% of its "
                         f"{bw}x{bh} box and still fit the frame")
            continue
        if fill > reachable:
            notes.append(f"  target {i + 1} fill {fill}% -> {int(reachable)}% "
                         f"({bw}x{bh} box)")
            fill = int(reachable)
        out.append((x1, y1, x2, y2, fill, n))

    if notes and verbose:
        print("  thresholds adjusted for this board's geometry:")
        for line in notes:
            print(line)
    return out


class TargetQueue:
    """The guided sequence. Holds the current box and what it still wants."""

    def __init__(self, targets, width, height, pattern=(9, 6)):
        self._targets = list(targets)
        self._w, self._h = width, height
        self._cols, self._rows = pattern
        self._total_samples = sum(t[5] for t in self._targets)
        self._taken = 0
        self._index = 0
        self._load()

    def _load(self):
        if self.done:
            return
        x1, y1, x2, y2, fill, n = self._targets[self._index]
        bx1 = x1 * self._w // 100
        by1 = y1 * self._h // 100
        bx2 = x2 * self._w // 100
        by2 = y2 * self._h // 100
        self.box = (bx1, by1, bx2 - bx1, by2 - by1)
        self.rect = (bx1, by1, bx2, by2)
        self.fill_pct = float(fill)
        self.remaining = n

        # Runtime adaptation state. fit_targets() reasons about a pinhole
        # projection, which is an upper bound and sometimes a loose one — see
        # maybe_relax().
        self.best_fill = 0.0
        self.in_box_hits = 0
        self.t_start = time.monotonic()
        self._relaxed = False
        self._warned = False

        # The threshold region is the target box plus one board square on every
        # side. These are two different things and conflating them is a bug:
        #
        #   the target box is a PLACEMENT constraint — where the detected inner
        #   corners have to land, and what the fill fraction is measured against;
        #
        #   the threshold region is an IMAGE PROCESSING extent, and it has to
        #   contain the whole physical board, not just its inner corners.
        #
        # A chessboard extends a full square beyond its outermost inner corners.
        # Threshold exactly the target box and, once the board is filling it, the
        # white border gets drawn straight through that outer ring of squares —
        # within a few pixels of real corners. findChessboardCorners then locks
        # onto the border instead of the corner and returns a position ~20 px out,
        # which no amount of subpixel refinement will recover. It is not a
        # detection failure either: the board is still found, so a bad view is
        # silently accepted and quietly poisons the fit.
        #
        # Sizing: at full fill the board's inner span matches the box, so one
        # square is box_w/(cols-1). That is the worst case, so use it directly.
        pad_x = int(self.box[2] / max(self._cols - 1, 1)) + 8
        pad_y = int(self.box[3] / max(self._rows - 1, 1)) + 8
        tx1 = max(bx1 - pad_x, 0)
        ty1 = max(by1 - pad_y, 0)
        tx2 = min(bx2 + pad_x, self._w)
        ty2 = min(by2 + pad_y, self._h)
        self.thresh_box = (tx1, ty1, tx2 - tx1, ty2 - ty1)

    @property
    def done(self):
        return self._index >= len(self._targets)

    @property
    def index(self):
        return self._index

    @property
    def count(self):
        return len(self._targets)

    @property
    def progress(self):
        return self._taken / self._total_samples if self._total_samples else 1.0

    def box_area(self):
        return abs(self.box[2] * self.box[3])

    def contains(self, corners):
        x1, y1, x2, y2 = self.rect
        pts = corners.reshape(-1, 2)
        return bool(np.all((pts[:, 0] >= x1) & (pts[:, 0] <= x2) &
                           (pts[:, 1] >= y1) & (pts[:, 1] <= y2)))

    def note(self, inside, fill):
        if inside:
            self.in_box_hits += 1
            self.best_fill = max(self.best_fill, fill)

    def maybe_relax(self, after_sec):
        """Lower an unreachable threshold to what is actually being achieved.

        fit_targets() rules out the geometrically impossible before the run
        starts, but it reasons about a PINHOLE projection. A real wide lens
        compresses the board — most at the edges of the frame, which is exactly
        where the corner targets put it — so the achievable fill can sit well under
        the pinhole bound. Measured on a mild fisheye, the corner targets cap out
        at 39% against a pinhole bound of 66%, and the shipped threshold is 40%:
        unreachable by one percentage point, with nothing on screen to say so. The
        operator sees the box refusing to turn green and no reason why.

        Modelling the projection properly is not worth it — the distortion is the
        unknown being solved for. Observing the plateau is both simpler and more
        general: it covers a partly obstructed lens or an oddly proportioned board
        just as well.

        Deliberately conservative. It requires a sustained, genuine attempt — the
        board repeatedly detected INSIDE the box — so simply being slow to pick up
        the board never trips it, and it fires once per target and says so loudly.
        """
        if after_sec <= 0 or self._relaxed or self.done:
            return None
        if time.monotonic() - self.t_start < after_sec:
            return None
        if self.in_box_hits < 15 or self.best_fill >= self.fill_pct:
            return None
        if self.best_fill < FILL_FLOOR:
            if not self._warned:
                self._warned = True
                return (f"  target {self._index + 1}: best fill so far is only "
                        f"{self.best_fill:.0f}%, far short of {self.fill_pct:.0f}%. "
                        f"Move the board closer; press ENTER to skip this target.")
            return None
        old = self.fill_pct
        self.fill_pct = max(float(FILL_FLOOR), self.best_fill - 1.0)
        self._relaxed = True
        return (f"  target {self._index + 1}: {old:.0f}% is not reachable here "
                f"(best {self.best_fill:.0f}% over {self.in_box_hits} attempts) — "
                f"threshold lowered to {self.fill_pct:.0f}%")

    def accept(self):
        self._taken += 1
        self.remaining -= 1
        if self.remaining <= 0:
            self.advance(count_samples=False)

    def advance(self, count_samples=True):
        """Move to the next target. Skipping forfeits the samples it wanted."""
        if count_samples:
            self._taken += self.remaining
        self._index += 1
        self._load()


def quad_area(corners, cols, rows):
    """Shoelace area of the board's four outer corners, in pixels squared.

    Using the outer corners rather than a bounding box is what makes the fill
    fraction meaningful under tilt: a board rotated 30 degrees away has a smaller
    quadrilateral, so it has to come closer to satisfy the same threshold, which
    is exactly the behaviour wanted.
    """
    pts = corners.reshape(-1, 2)
    a = pts[0]
    b = pts[cols - 1]
    c = pts[cols * rows - 1]
    d = pts[cols * (rows - 1)]
    quad = np.array([a, b, c, d])
    x, y = quad[:, 0], quad[:, 1]
    return 0.5 * abs(np.dot(x, np.roll(y, -1)) - np.dot(y, np.roll(x, -1)))


def fill_colour(current, target):
    """Red at nothing, green at the threshold, with the change crowded into the
    last stretch so 'nearly there' still reads as 'not there'."""
    r = min(max(current / target if target else 1.0, 0.0), 1.0) ** 5
    return (0, int(255 * r), int(255 * (1 - r)))


# ---------------------------------------------------------------------------
# Overlay
# ---------------------------------------------------------------------------

def make_overlay(color, pct, header, footer):
    h = color.shape[0]
    if color.shape[1] < 640:
        scale = 640.0 / color.shape[1]
        color = cv2.resize(color, None, fx=scale, fy=scale)
        h = color.shape[0]
    band = max(24, h // 12)
    out = cv2.copyMakeBorder(color, band, band // 2, 0, 0, cv2.BORDER_CONSTANT, value=0)

    cv2.rectangle(out, (0, 0), (int(pct * out.shape[1]), band // 2), GREEN, -1)
    fs, th = band / 80.0, max(1, band // 40)
    cv2.putText(out, header, (int(0.02 * out.shape[1]), int(band * 0.9)),
                cv2.FONT_HERSHEY_SIMPLEX, fs, WHITE, th, cv2.LINE_AA)
    cv2.putText(out, footer, (int(0.02 * out.shape[1]), out.shape[0] - max(4, band // 8)),
                cv2.FONT_HERSHEY_SIMPLEX, fs, WHITE, th, cv2.LINE_AA)
    return out


# ---------------------------------------------------------------------------
# MJPEG preview — the stand-in for voxl-portal
# ---------------------------------------------------------------------------

_PAGE = b"""<!doctype html><html><head><title>calibrate_camera</title>
<style>html,body{margin:0;background:#111;height:100%}
img{display:block;width:100%;height:100%;object-fit:contain}</style></head>
<body><img src="/stream.mjpg"></body></html>"""


class MjpegServer:
    """One latest-frame slot, any number of watchers.

    Deliberately drops frames rather than queueing them: a slow or stalled
    browser must never backpressure the capture loop, because the thing the
    person is reacting to is the live image, not a backlog of stale ones.
    """

    def __init__(self, port, bind="0.0.0.0", quality=80, scale=1.0):
        self._cond = threading.Condition()
        self._jpeg = None
        self._seq = 0
        self._quality = quality
        self._scale = scale
        server = self

        class Handler(BaseHTTPRequestHandler):
            protocol_version = "HTTP/1.1"

            def log_message(self, *a):
                pass

            def do_GET(self):
                if self.path in ("/", "/index.html"):
                    self.send_response(200)
                    self.send_header("Content-Type", "text/html")
                    self.send_header("Content-Length", str(len(_PAGE)))
                    self.end_headers()
                    self.wfile.write(_PAGE)
                    return
                if self.path != "/stream.mjpg":
                    self.send_error(404)
                    return
                self.send_response(200)
                self.send_header("Age", "0")
                self.send_header("Cache-Control", "no-cache, private")
                self.send_header(
                    "Content-Type",
                    "multipart/x-mixed-replace; boundary=FRAME")
                self.end_headers()
                last = -1
                try:
                    while True:
                        jpeg, last = server._wait(last)
                        if jpeg is None:
                            break
                        self.wfile.write(b"--FRAME\r\n")
                        self.wfile.write(b"Content-Type: image/jpeg\r\n")
                        self.wfile.write(
                            b"Content-Length: %d\r\n\r\n" % len(jpeg))
                        self.wfile.write(jpeg)
                        self.wfile.write(b"\r\n")
                except (BrokenPipeError, ConnectionResetError):
                    pass

        self._httpd = ThreadingHTTPServer((bind, port), Handler)
        self._httpd.daemon_threads = True
        self._thread = threading.Thread(target=self._httpd.serve_forever, daemon=True)
        self._closed = False

    def start(self):
        self._thread.start()

    def _wait(self, last_seq, timeout=5.0):
        """Block until there is a frame newer than last_seq.

        Loops rather than returning on timeout: a browser that connects before
        the first frame is published — which is the normal case, since you open
        the page and then start waving a board at the camera — would otherwise be
        handed None and have its stream closed on it.
        """
        with self._cond:
            while not self._closed and (self._seq == last_seq or self._jpeg is None):
                self._cond.wait(timeout)
            if self._closed:
                return None, self._seq
            return self._jpeg, self._seq

    def publish(self, bgr):
        if self._scale != 1.0:
            bgr = cv2.resize(bgr, None, fx=self._scale, fy=self._scale,
                             interpolation=cv2.INTER_AREA)
        ok, buf = cv2.imencode(".jpg", bgr,
                               [int(cv2.IMWRITE_JPEG_QUALITY), self._quality])
        if not ok:
            return
        with self._cond:
            self._jpeg = buf.tobytes()
            self._seq += 1
            self._cond.notify_all()

    def close(self):
        with self._cond:
            self._closed = True
            self._cond.notify_all()
        self._httpd.shutdown()


def local_ips():
    ips = []
    try:
        import socket
        for info in socket.getaddrinfo(socket.gethostname(), None):
            ip = info[4][0]
            if ":" not in ip and not ip.startswith("127.") and ip not in ips:
                ips.append(ip)
    except Exception:
        pass
    return ips


# ---------------------------------------------------------------------------
# stdin control
# ---------------------------------------------------------------------------

class StdinControl:
    """ENTER skips the current target, `stop` finishes sampling early.

    Skipping is not a convenience. Some targets are physically unreachable on a
    real airframe — a lens hood clipping a corner, a camera recessed in a mount —
    and without a skip the run simply never completes.
    """

    def __init__(self):
        self.skip = False
        self.stop = False
        self._thread = threading.Thread(target=self._run, daemon=True)

    def start(self):
        if sys.stdin and sys.stdin.isatty():
            self._thread.start()

    def _run(self):
        for line in sys.stdin:
            s = line.strip().lower()
            if s == "":
                print("  [skipping this target]")
                self.skip = True
            elif s in ("stop", "q", "quit"):
                print("  [stopping sampling]")
                self.stop = True
                return
            else:
                print("  ENTER to skip this target, 'stop' to finish sampling")


# ---------------------------------------------------------------------------
# Detection
# ---------------------------------------------------------------------------

def find_board(thresh_img, raw_gray, pattern):
    """Search the thresholded image, refine on the raw one.

    Flags are zero on purpose. NORMALIZE_IMAGE is redundant with the thresholding
    above and slower. FAST_CHECK actively hurts: it rejects the board exactly when
    it fills a large fraction of the frame, which is the case the fill thresholds
    are driving towards.
    """
    ok, corners = cv2.findChessboardCorners(thresh_img, pattern, flags=0)
    if not ok:
        return False, None
    corners = cv2.cornerSubPix(
        raw_gray, corners, (SUBPIX_WINDOW, SUBPIX_WINDOW), (-1, -1),
        (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.1))
    return True, corners


def to_gray(frame):
    if frame.ndim == 3:
        return cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    return frame


# ---------------------------------------------------------------------------
# Calibration
# ---------------------------------------------------------------------------

def calibrate(obj_points, img_points, img_size, fisheye, fx_guess):
    w, h = img_size
    K = np.eye(3, dtype=np.float64)
    if fx_guess:
        K[0, 0] = K[1, 1] = fx_guess
        K[0, 2], K[1, 2] = w / 2.0, h / 2.0

    if fisheye:
        objp = [o.reshape(-1, 1, 3).astype(np.float64) for o in obj_points]
        imgp = [p.reshape(-1, 1, 2).astype(np.float64) for p in img_points]
        D = np.zeros((4, 1))
        flags = (cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC |
                 cv2.fisheye.CALIB_CHECK_COND |
                 cv2.fisheye.CALIB_FIX_SKEW)
        if fx_guess:
            flags |= cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
        else:
            # The fisheye solver is far more sensitive to its starting point than
            # the pinhole one. 0.43*width is ModalAI's empirical ratio; it holds
            # across the OV7251 at 640 and the AR0144 at 1280, so it is a property
            # of the lens class rather than of one module.
            K[0, 0] = K[1, 1] = 0.43 * w
            K[0, 2], K[1, 2] = w / 2.0, h / 2.0
            flags |= cv2.fisheye.CALIB_USE_INTRINSIC_GUESS
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            objp, imgp, (w, h), K, D, flags=flags,
            criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 100, 1e-6))
        return rms, K, D.flatten(), rvecs, tvecs, "fisheye"

    flags = cv2.CALIB_USE_INTRINSIC_GUESS if fx_guess else 0
    rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, (w, h), K if fx_guess else None, None, flags=flags)
    return rms, K, D.flatten()[:5], rvecs, tvecs, "pinhole"


def per_view_errors(obj_points, img_points, rvecs, tvecs, K, D, fisheye):
    """Where the error actually lives.

    A single RMS hides the shape of the problem. One view at 3 px among twenty at
    0.3 px is a blurred or mis-ordered detection, and dropping it fixes the
    calibration; a uniform 1.5 px across every view is a wrong --square-mm or a
    board that is not flat, and dropping views will not help at all.
    """
    errs = []
    for i, obj in enumerate(obj_points):
        if fisheye:
            proj, _ = cv2.fisheye.projectPoints(
                obj.reshape(-1, 1, 3).astype(np.float64), rvecs[i], tvecs[i], K, D.reshape(4, 1))
        else:
            proj, _ = cv2.projectPoints(obj, rvecs[i], tvecs[i], K, D)
        d = proj.reshape(-1, 2) - img_points[i].reshape(-1, 2)
        errs.append(float(np.sqrt((d ** 2).sum() / len(d))))
    return errs


def report_fov(fx, fy, width, height):
    hfov = 2.0 * math.atan(width / (2.0 * fx))
    vfov = 2.0 * math.atan(height / (2.0 * fy))
    dfov = 2.0 * math.atan(math.hypot(width, height) /
                           (2.0 * math.hypot(fx, fy) / math.sqrt(2)))
    print(f"  horizontal FOV : {math.degrees(hfov):.1f} deg")
    print(f"  vertical   FOV : {math.degrees(vfov):.1f} deg")
    print(f"  diagonal   FOV : {math.degrees(dfov):.1f} deg")
    return math.degrees(hfov)


def camera_block(K, dist, model, w, h, indent="    "):
    coeffs = ", ".join(f"{v:.6f}" for v in dist)
    lines = [
        "camera:",
        f"  fx: {K[0, 0]:.4f}",
        f"  fy: {K[1, 1]:.4f}",
        f"  cx: {K[0, 2]:.4f}",
        f"  cy: {K[1, 2]:.4f}",
        f"  distortion_model: {model}",
        f"  dist_coeffs: [{coeffs}]",
        f"  calib_width: {w}",
        f"  calib_height: {h}",
        "  undistort: false",
    ]
    return "\n".join(indent + ln for ln in lines)


def write_result(path, K, dist, model, w, h, rms, args, n_views):
    """Plain YAML, so it can be read back by anything.

    Not OpenCV's FileStorage format: yaml-cpp, which is what the service uses,
    chokes on FileStorage's type tags. --opencv-yml writes that format separately
    for anyone who wants the ModalAI-compatible artifact.
    """
    coeffs = ", ".join(f"{v:.6f}" for v in dist)
    with open(path, "w") as f:
        f.write(f"# fiducial-detector-service camera intrinsics\n")
        f.write(f"# measured {time.strftime('%Y-%m-%d %H:%M:%S')}\n")
        f.write(f"# camera_id: {args.camera_id}\n")
        if args.sensor:
            f.write(f"# sensor: {args.sensor} mode {args.mode}\n")
        f.write(f"# board: {args.cols}x{args.rows} inner corners, "
                f"{args.square_mm} mm squares\n")
        f.write(f"# views: {n_views}   rms reprojection error: {rms:.4f} px\n")
        f.write("#\n")
        f.write("# Paste the camera block below into the matching stream in\n")
        f.write("# config/*.yaml. These intrinsics belong to THIS camera, THIS\n")
        f.write("# lens and THIS sensor mode.\n\n")
        f.write(camera_block(K, dist, model, w, h, indent="") + "\n")


def write_opencv_yml(path, K, dist, model, w, h, rms):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("M", K)
    fs.write("D", np.asarray(dist, dtype=np.float64).reshape(-1, 1))
    fs.write("reprojection_error", float(rms))
    fs.write("width", int(w))
    fs.write("height", int(h))
    fs.write("distortion_model", "fisheye" if model == "fisheye" else "plumb_bob")
    fs.write("calibration_time", time.strftime("%Y-%m-%d %H:%M:%S"))
    fs.release()


# ---------------------------------------------------------------------------
# Capture loops
# ---------------------------------------------------------------------------

def out_path(args):
    return args.out or f"calib_{re.sub(r'[^A-Za-z0-9_.-]', '_', args.camera_id)}.yaml"


def load_points(path):
    d = np.load(path)
    obj = [o for o in d["obj"]]
    img = [i for i in d["img"]]
    return obj, img, tuple(int(v) for v in d["img_size"])


def open_capture(gst=None, video=None):
    if video:
        cap = cv2.VideoCapture(video)
        if not cap.isOpened():
            sys.exit(f"cannot open video file: {video}")
        return cap
    gst, note = normalise_appsink(gst)
    if note:
        print(note)
    cap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)
    if not cap.isOpened():
        sys.exit("cannot open the GStreamer pipeline.\n"
                 "  - check it ends in an appsink producing BGR\n"
                 "  - 'cannot find appsink in manual pipeline' means the appsink is\n"
                 "    misnamed, not missing: OpenCV only recognises a sink whose name\n"
                 "    contains 'appsink' or 'opencvsink'. Drop `name=sink`.\n"
                 "  - check OpenCV was built with GStreamer: "
                 "python3 -c \"import cv2; print(cv2.getBuildInformation())\" "
                 "| grep -i gstreamer\n"
                 "  - run tools/check_camera.sh --capture to settle the ingest path")
    return cap


def guided_capture(args, cap, preview, objp, pattern):
    cols, rows = pattern
    adjustment = args.threshold_bias

    obj_points, img_points = [], []
    img_size = None
    queue = None
    control = StdinControl()
    control.start()

    last_accept = 0.0
    t0 = time.time()
    frames = 0
    waiting_said = False

    while True:
        ok, frame = cap.read()
        if not ok:
            if args.video:
                print("\n  end of file reached before sampling completed")
                break
            # cap.read() failing on a live source does not mean the pipeline is
            # broken — it is also what a camera that has not warmed up looks like.
            # The distinction is how long it goes on for.
            if not waiting_said and frames == 0:
                waiting_said = True
                print(f"  waiting for the first frame "
                      f"(giving up after {args.open_timeout:.0f} s)...")
            time.sleep(0.05)
            if frames == 0 and time.time() - t0 > args.open_timeout:
                print(f"\nthe pipeline opened but produced no frame in "
                      f"{args.open_timeout:.0f} s.\n")
                print(argus_failure_advice(args.platform or detect_platform()))
                sys.exit(1)
            continue

        frames += 1
        gray = to_gray(frame)
        h, w = gray.shape[:2]

        if queue is None:
            img_size = (w, h)
            # Inner-corner span, which is what quad_area measures. The OUTER board
            # is one square larger in each axis, but the detector never sees it.
            board_aspect = (cols - 1) / (rows - 1)
            targets = fit_targets(get_targets(w, h, args.fisheye), w, h, cols, rows)
            if not targets:
                sys.exit("no target is reachable with this board at this frame size")
            queue = TargetQueue(targets, w, h, pattern)
            print(f"  frame size {w}x{h}, {queue.count} targets, "
                  f"{'fisheye' if args.fisheye else 'pinhole'} set, "
                  f"board {board_aspect:.2f}:1\n")
            if args.fx_guess:
                warn_working_distance(targets, w, h, cols, rows,
                                      args.square_mm, args.fx_guess)

        if control.stop or queue.done:
            break
        if control.skip:
            control.skip = False
            queue.advance()
            if queue.done:
                break

        thresh = threshold_box(gray, queue.thresh_box, adjustment)

        base = thresh if args.thresh_overlay else gray
        color = cv2.cvtColor(base, cv2.COLOR_GRAY2BGR)

        found, corners = find_board(thresh, gray, pattern)
        fill = 0.0
        inside = False

        if found:
            cv2.drawChessboardCorners(color, pattern, corners, True)
            inside = queue.contains(corners)
            if inside:
                fill = 100.0 * quad_area(corners, cols, rows) / max(queue.box_area(), 1)

        if inside:
            cv2.rectangle(color, queue.rect[:2], queue.rect[2:],
                          fill_colour(fill, queue.fill_pct), 2)
        else:
            cv2.rectangle(color, queue.rect[:2], queue.rect[2:], RED, 2)

        queue.note(inside, fill)
        relaxed = queue.maybe_relax(args.adapt_after)
        if relaxed:
            print(relaxed)

        now = time.time()
        if (inside and fill >= queue.fill_pct
                and now - last_accept >= args.settle):
            obj_points.append(objp.copy())
            img_points.append(corners)
            last_accept = now
            if args.save_dir:
                cv2.imwrite(os.path.join(
                    args.save_dir, f"calib_{len(obj_points):03d}.png"), frame)
            print(f"  target {queue.index + 1}/{queue.count}  "
                  f"sample accepted ({len(obj_points)} total, "
                  f"fill {fill:.0f}% >= {queue.fill_pct:.0f}%)")
            queue.accept()
            if queue.done:
                if preview:
                    done_img = cv2.flip(color, 1) if args.mirror else color
                    preview.publish(make_overlay(done_img, 1.0,
                                                 "Sampling complete", ""))
                break

        header = (f"Target {queue.index + 1}/{queue.count}   "
                  f"need {queue.remaining} more   "
                  f"fill {fill:.0f}%/{queue.fill_pct:.0f}% (best {queue.best_fill:.0f}%)   "
                  f"total {queue.progress * 100:.0f}%")
        footer = (f"Board {cols}x{rows}  {args.square_mm:.1f}mm   "
                  f"views {len(obj_points)}")

        # PREVIEW ONLY, and only after detection, acceptance and every overlay
        # has been drawn. Two reasons it has to be here:
        #
        #   flipping the image before detection would hand the solver a mirrored
        #   board — the chirality reverses, so the fitted pose is a reflection and
        #   the intrinsics come out quietly wrong;
        #
        #   flipping the composite rather than just the video keeps the target box
        #   registered with the scene. Both move together, so the box still marks
        #   where the board physically has to go.
        if args.mirror:
            color = cv2.flip(color, 1)
        if preview:
            preview.publish(make_overlay(color, queue.progress, header, footer))

    return obj_points, img_points, img_size


def image_capture(args, objp, pattern):
    obj_points, img_points, img_size = [], [], None
    files = []
    for pat in args.images:
        files.extend(sorted(glob.glob(pat)) if any(c in pat for c in "*?[") else [pat])
    for f in files:
        img = cv2.imread(f)
        if img is None:
            print(f"  skip (unreadable): {f}")
            continue
        gray = to_gray(img)
        img_size = gray.shape[::-1]
        # No target box offline, so threshold the whole frame as one box.
        thresh = threshold_box(gray, (0, 0, gray.shape[1], gray.shape[0]),
                               args.threshold_bias)
        ok, corners = find_board(thresh, gray, pattern)
        print(f"  {'FOUND' if ok else '  -  '}  {os.path.basename(f)}")
        if ok:
            obj_points.append(objp.copy())
            img_points.append(corners)
    return obj_points, img_points, img_size


# ---------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)

    src = ap.add_mutually_exclusive_group()
    src.add_argument("--gst", help="GStreamer pipeline ending in an appsink producing BGR")
    src.add_argument("--images", nargs="+", help="image files instead of live capture")
    src.add_argument("--video", help="video file, run through the guided loop")
    src.add_argument("--from-points", help="re-solve from a saved *_points.npz "
                                           "instead of capturing again")

    ap.add_argument("--sensor", choices=sorted(SENSORS),
                    help="build the pipeline and seed the intrinsics guess")
    ap.add_argument("--mode", type=int, help="sensor mode for --sensor")
    ap.add_argument("--sensor-id", type=int, default=0, help="CSI port for --sensor")
    ap.add_argument("--platform", choices=["jetson", "pi", "generic"],
                    help="override platform detection for --sensor")
    ap.add_argument("--list-modes", action="store_true",
                    help="print the sensor mode table and exit")

    ap.add_argument("--fov-only", action="store_true",
                    help="print the FOV implied by --fx/--width and exit")
    ap.add_argument("--fx", type=float, help="for --fov-only")
    ap.add_argument("--width", type=int, default=1280, help="for --fov-only")
    ap.add_argument("--height", type=int, default=720, help="for --fov-only")

    ap.add_argument("--cols", type=int, default=9, help="INNER corners across")
    ap.add_argument("--rows", type=int, default=6, help="INNER corners down")
    ap.add_argument("--square-mm", type=float, default=25.0,
                    help="measured square edge in mm — measure it, do not assume")

    ap.add_argument("--fisheye", action="store_true",
                    help="use the fisheye model and the fisheye target set")
    ap.add_argument("--threshold-bias", type=int, default=THRESH_ADJUSTMENT,
                    help="shift every adaptive threshold by this many grey levels. "
                         "Raise it if the board washes out and detection drops")
    ap.add_argument("--fx-guess", type=float,
                    help="seed CALIB_USE_INTRINSIC_GUESS (overrides --sensor)")
    ap.add_argument("--settle", type=float, default=0.5,
                    help="minimum seconds between accepted samples. ModalAI takes "
                         "consecutive frames; a gap here stops one board placement "
                         "from filling a target with near-identical views, which "
                         "inflate the view count without constraining anything. "
                         "0 for ModalAI's behaviour")

    ap.add_argument("--open-timeout", type=float, default=15.0,
                    help="seconds to wait for the first frame before giving up "
                         "with a diagnosis")
    ap.add_argument("--adapt-after", type=float, default=20.0,
                    help="seconds of visible effort on one target before its fill "
                         "threshold is lowered to what is actually achievable. "
                         "0 disables, leaving the run to stall on an unreachable "
                         "target until you skip it")
    ap.add_argument("--preview-port", type=int, default=8080,
                    help="MJPEG preview port, 0 to disable")
    ap.add_argument("--preview-bind", default="0.0.0.0")
    ap.add_argument("--preview-scale", type=float, default=1.0,
                    help="downscale the preview stream to save bandwidth")
    ap.add_argument("--preview-quality", type=int, default=80)
    ap.add_argument("--raw-overlay", dest="thresh_overlay", action="store_false",
                    help="show raw pixels in the preview instead of the thresholded "
                         "image (cosmetic; detection is unaffected)")
    ap.add_argument("--no-mirror", dest="mirror", action="store_false",
                    help="do not mirror the preview. Mirrored is the default because "
                         "you are facing the camera holding the board")

    ap.add_argument("--save-dir", help="also save accepted frames here")
    ap.add_argument("--camera-id", default="down", help="label for the emitted YAML")
    ap.add_argument("--out", help="write intrinsics here "
                                  "(default ./calib_<camera_id>.yaml)")
    ap.add_argument("--opencv-yml", help="also write ModalAI-style OpenCV FileStorage")
    args = ap.parse_args()

    if args.list_modes:
        print_mode_table()
        return 0

    if args.fov_only:
        if not args.fx:
            sys.exit("--fov-only needs --fx")
        print(f"\nfx={args.fx} at {args.width}x{args.height}:")
        report_fov(args.fx, args.fx, args.width, args.height)
        print()
        return 0

    # --- resolve the source -------------------------------------------------
    fx_guess = args.fx_guess
    if args.sensor:
        if args.gst or args.images or args.video:
            sys.exit("--sensor builds the pipeline; do not also pass "
                     "--gst/--images/--video")
        spec = SENSORS[args.sensor]
        mode = args.mode if args.mode is not None else spec["default_mode"]
        if mode not in spec["modes"]:
            sys.exit(f"{args.sensor} has no mode {mode}. "
                     f"Run --list-modes.")
        args.mode = mode
        info = spec["modes"][mode]
        platform = args.platform or detect_platform()
        args.gst = build_pipeline(platform, info, mode, args.sensor_id)
        if fx_guess is None:
            fx_guess = info["fx"]
        print(f"\n{args.sensor} mode {mode}: {info['w']}x{info['h']} @ {info['fps']} "
              f"({info['kind']})")
        print(f"platform: {platform}")
        if fx_guess:
            print(f"intrinsics guess: fx = fy = {fx_guess:.0f} px "
                  f"({math.degrees(2 * math.atan(info['w'] / (2 * fx_guess))):.1f} deg "
                  f"horizontal, from sensor geometry — not a measurement)")
        else:
            print("no intrinsics guess: this sensor takes interchangeable lenses, "
                  "so there is no focal length to derive")
        print(f"pipeline:\n  {args.gst}\n")
    elif not (args.gst or args.images or args.video or args.from_points):
        sys.exit("pass --sensor <name>, --gst <pipeline>, --video <file>, "
                 "--images <files>, or --from-points <file>")

    args.fx_guess = fx_guess        # guided_capture warns on working distance
    if args.mode is None:
        args.mode = ""
    pattern = (args.cols, args.rows)
    square_m = args.square_mm / 1000.0

    # Object points in the board frame, in METRES. Everything downstream inherits
    # this unit, so a millimetre slip here is a 1000x range error, not a subtle one.
    objp = np.zeros((args.rows * args.cols, 3), np.float32)
    objp[:, :2] = np.mgrid[0:args.cols, 0:args.rows].T.reshape(-1, 2)
    objp *= square_m

    if args.save_dir:
        os.makedirs(args.save_dir, exist_ok=True)

    print(f"board {args.cols}x{args.rows} inner corners, {args.square_mm} mm squares")

    # --- collect ------------------------------------------------------------
    if args.from_points:
        obj_points, img_points, img_size = load_points(args.from_points)
        print(f"  replaying {len(obj_points)} saved detections from "
              f"{args.from_points} (no camera used)")
    elif args.images:
        obj_points, img_points, img_size = image_capture(args, objp, pattern)
    else:
        preview = None
        if args.preview_port:
            try:
                preview = MjpegServer(args.preview_port, args.preview_bind,
                                      args.preview_quality, args.preview_scale)
                preview.start()
            except OSError as e:
                print(f"  preview disabled: port {args.preview_port}: {e}")
                preview = None
        if preview:
            print(f"\npreview:  http://localhost:{args.preview_port}")
            for ip in local_ips():
                print(f"          http://{ip}:{args.preview_port}")
            print("\nOpen that in a browser. Put the board inside the red box and")
            print("move it closer until the box turns green.")
            print("ENTER skips a target, 'stop' finishes sampling early.\n")

        if not args.video:
            for note in camera_preflight(args.platform or detect_platform()):
                print(f"  WARNING: {note}\n")

        cap = open_capture(args.gst, args.video)
        try:
            obj_points, img_points, img_size = guided_capture(
                args, cap, preview, objp, pattern)
        except KeyboardInterrupt:
            print("\ninterrupted")
            obj_points, img_points, img_size = [], [], None
        finally:
            # nvarguscamerasrc can block forever tearing the Argus session down,
            # after it has already printed "GST_ARGUS: Done Success". That is the
            # camera stack's problem, but without this it becomes ours: the views
            # are collected and sitting in memory, and a wedged release() strands
            # them behind an unkillable-looking prompt. Release on a daemon thread
            # and move on — the process is about to exit anyway, so a leaked
            # capture costs nothing, and the calibration is what matters.
            releaser = threading.Thread(target=cap.release, daemon=True)
            releaser.start()
            releaser.join(RELEASE_TIMEOUT)
            if releaser.is_alive():
                print(f"  (camera teardown did not finish in {RELEASE_TIMEOUT:.0f} s "
                      f"— continuing; this is an nvarguscamerasrc quirk)")
            if preview:
                time.sleep(0.5)   # let the last overlay reach the browser
                preview.close()

    n = len(obj_points)
    if n < 6 or img_size is None:
        sys.exit(f"\nonly {n} usable views — need at least 6. Nothing written.")

    # Persist the detections before solving. Collecting them is the part that
    # costs minutes of standing in front of a camera; solving takes a second and
    # may well want repeating with different flags. --from-points replays these
    # without touching the camera, which also means a failed run is worth keeping.
    if not args.from_points:
        pts_path = os.path.splitext(out_path(args))[0] + "_points.npz"
        try:
            np.savez(pts_path,
                     obj=np.array(obj_points, dtype=np.float32),
                     img=np.array(img_points, dtype=np.float32),
                     img_size=np.array(img_size),
                     cols=args.cols, rows=args.rows, square_mm=args.square_mm)
            print(f"  detections saved to {pts_path}")
            print(f"  (re-solve without the camera: --from-points {pts_path})")
        except OSError as e:
            print(f"  could not save detections: {e}")

    # --- solve --------------------------------------------------------------
    print(f"\ncollected {n} views, calibrating...")
    try:
        rms, K, dist, rvecs, tvecs, model = calibrate(
            obj_points, img_points, img_size, args.fisheye, fx_guess)
    except cv2.error as e:
        # CALIB_CHECK_COND raises rather than returning a bad fit. The view it
        # names is almost always one where the board is nearly edge-on.
        sys.exit(f"\ncalibration failed: {e}\n"
                 "For a fisheye run this usually names one ill-conditioned view. "
                 "Re-shoot that target with the board less edge-on.")

    fx, fy = K[0, 0], K[1, 1]
    cx, cy = K[0, 2], K[1, 2]
    w, h = img_size
    cutoff = FISHEYE_ERROR_CUTOFF if args.fisheye else PLUMB_ERROR_CUTOFF

    print(f"\nRMS reprojection error: {rms:.4f} px   (pass threshold {cutoff})")
    errs = per_view_errors(obj_points, img_points, rvecs, tvecs, K, dist, args.fisheye)
    worst = int(np.argmax(errs))
    print(f"  per-view: best {min(errs):.3f}  median {np.median(errs):.3f}  "
          f"worst {max(errs):.3f} (view {worst + 1})")

    med = float(np.median(errs))
    if rms > cutoff:
        print("\n  FAILED. Do not ship this.")
        # An outlier and a systematic problem are not alternatives, and treating
        # them as such misreads the common case: a high median with one worse view
        # still means EVERY view is bad, and dropping the worst fixes nothing.
        # The median is what separates them, so test it first and independently.
        if med > cutoff:
            span_mm = (args.cols - 1) * args.square_mm
            print(f"  The MEDIAN view is {med:.2f} px, already past the {cutoff} px")
            print("  threshold, so this is systematic — not one bad capture.")
            print()
            print("  By far the most likely cause is that THE BOARD IS NOT FLAT.")
            print("  The solver is told the corners are coplanar; every millimetre")
            print("  they are not goes straight into the fit and cannot be averaged")
            print(f"  out by more views. On your {span_mm:.0f} mm board, measured:")
            print("      bow  1 mm (0.5% of width) -> ~0.19 px")
            print("      bow  2 mm (1.0%)          -> ~0.39 px")
            print("      bow  5 mm (2.5%)          -> ~0.96 px  and fx off by ~4%")
            print(f"  Keep it flat to about 0.5% of its width — {span_mm * 0.005:.1f} mm here.")
            print("  Paper taped to a wall or held in the hand does not meet that;")
            print("  glass, MDF or aluminium composite does. This is the single")
            print("  highest-value thing to fix.")
            print()
            print("  Less likely, in order:")
            print("    - the print is scaled non-uniformly (check x and y separately")
            print("      against a steel rule, not just one square).")
            print("    - the board moved during exposure. The IMX219 is rolling")
            print("      shutter, so motion shears it rather than smearing it, and")
            print("      shear does bias corners. Hold still between captures.")
            print("  Plain defocus and straight-line motion blur are NOT likely causes:")
            print("  both are symmetric and leave a chessboard saddle point where it")
            print("  was. A sharp-looking preview does not rule out a bent board.")
            if max(errs) > 3 * med:
                print(f"\n  View {worst + 1} is additionally an outlier at {max(errs):.2f} px.")
        elif max(errs) > 3 * med:
            print(f"  The median view is {med:.2f} px, which is fine — view {worst + 1} alone")
            print(f"  is bad at {max(errs):.2f} px. One bad detection, not a systematic")
            print("  problem. Re-run; if it recurs, that target is where the blur is.")
        else:
            print("  The error is spread across views without a clear outlier, which")
            print("  points at the board or the capture conditions rather than at any")
            print("  one view.")
    elif rms > cutoff / 2:
        print("  acceptable, but tighter is better for a landing.")
    else:
        print("  good.")

    print(f"\nresolution {w}x{h}")
    hfov = report_fov(fx, fy, w, h)

    # The FOV number is the one that settles which sensor mode you are really in.
    # A cropped mode reports a markedly narrower FOV than a binned one at the same
    # output resolution, and that is measurable rather than a matter of opinion.
    if args.sensor and fx_guess:
        expected = math.degrees(2 * math.atan(w / (2 * fx_guess)))
        drift = hfov - expected
        print(f"\n  geometry predicted {expected:.1f} deg, measured {hfov:.1f} deg "
              f"({abs(drift):.1f} deg apart).")
        if abs(drift) > 5:
            if rms > cutoff:
                print("  Treat that gap as unexplained for now: the fit did not pass,")
                print("  and a bad fit moves fx directly. Fix the error first — the")
                print("  question usually answers itself.")
            # The SIGN carries the diagnosis, and collapsing it to a magnitude
            # throws away the only thing that distinguishes the two causes.
            elif drift > 0:
                mode_info = SENSORS[args.sensor]["modes"].get(args.mode, {})
                px_um = mode_info.get("px_um")
                print("  Measured WIDER than predicted. A crop can only ever narrow the")
                print("  field of view, and this preset already describes the widest")
                print("  mode, so the sensor mode cannot explain it. The lens can:")
                print("  the preset assumes the stock 3.04 mm Pi Camera v2 lens, and")
                print("  plenty of IMX219 boards ship a wider one.")
                if px_um:
                    print(f"  Measured fx implies a focal length of about "
                          f"{fx * px_um / 1000.0:.2f} mm.")
                print("  Nothing is wrong with the calibration — the preset's guess was")
                print("  for a different lens. Use the measured numbers.")
            else:
                print("  Measured NARROWER than predicted, which is what a cropped")
                print("  sensor mode looks like. Check the mode Argus actually ran")
                print("  against the one requested:  tools/check_camera.sh --capture")
                print("  A longer lens than the preset assumes would look the same.")
    if hfov < 45:
        print(f"  {hfov:.0f} deg is narrow for precision landing: at 1 m altitude this")
        print(f"  sees about {2 * 1.0 * math.tan(math.radians(hfov / 2)):.2f} m across, "
              "and the pad")
        print("  can leave frame exactly when you need it.")

    # Principal point far from centre is a real signal, usually of a cropped or
    # misaligned readout rather than of a bad calibration.
    off_x, off_y = abs(cx - w / 2) / w, abs(cy - h / 2) / h
    if off_x > 0.05 or off_y > 0.05:
        print(f"\n  NOTE: principal point is {off_x * 100:.1f}%/{off_y * 100:.1f}% "
              f"off centre ({cx:.1f}, {cy:.1f} vs {w / 2:.0f}, {h / 2:.0f}).")
        print("  Mild is normal; large usually means an off-centre sensor crop.")

    # --- write --------------------------------------------------------------
    out = out_path(args)
    write_result(out, K, dist, model, w, h, rms, args, n)
    if args.opencv_yml:
        write_opencv_yml(args.opencv_yml, K, dist, model, w, h, rms)

    print(f"""
------------------------------------------------------------------
Written to {out}{f" and {args.opencv_yml}" if args.opencv_yml else ""}

Paste into the stream's `camera:` block (config/*.yaml):

{camera_block(K, dist, model, w, h)}
------------------------------------------------------------------

calib_width/calib_height record what this was measured at. If the stream ever
negotiates a different size the service rescales fx/fy/cx/cy and logs it — but
re-calibrating at the runtime resolution is better than relying on that.

These intrinsics belong to THIS camera and THIS lens at THIS sensor mode.
Re-run after any lens swap, refocus, or sensor-mode change.
""")
    return 0 if rms <= cutoff else 1


if __name__ == "__main__":
    sys.exit(main())
