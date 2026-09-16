#!/bin/bash
################################################################################
# check_camera.sh — does the camera enumerate, and which ingest path works?
#
# Whether the Jetson ingest path is nvarguscamerasrc or a debayer pipeline is a
# device-tree and cabling question rather than a software one. Run this first: a
# camera that does not enumerate produces the same symptom as a bad pipeline
# string.
#
# Safe to run on both platforms; each section skips itself where it does not apply.
#
#   ./tools/check_camera.sh              # probe everything
#   ./tools/check_camera.sh --capture    # also try to pull real frames (slower)
################################################################################

set -u

CAPTURE=false
[ "${1:-}" = "--capture" ] && CAPTURE=true

hr() { printf '%*s\n' 78 '' | tr ' ' '-'; }
have() { command -v "$1" >/dev/null 2>&1; }

echo
hr
echo " fiducial-detector-service — camera probe"
hr

# ---- platform ---------------------------------------------------------------
PLATFORM="unknown"
if [ -f /etc/nv_tegra_release ]; then
    PLATFORM="jetson"
    echo "Platform : Jetson — $(head -1 /etc/nv_tegra_release)"
elif grep -qi raspberry /proc/device-tree/model 2>/dev/null; then
    PLATFORM="pi"
    echo "Platform : $(tr -d '\0' < /proc/device-tree/model)"
else
    echo "Platform : generic $(uname -m) — expect the USB/v4l2src or file path"
fi
echo "Kernel   : $(uname -r)"
have gst-launch-1.0 && echo "GStreamer: $(gst-launch-1.0 --version | head -1)" \
                    || echo "GStreamer: NOT INSTALLED — this service cannot ingest anything"
echo

# ---- driver bind ------------------------------------------------------------
hr
echo " 1. Did a sensor driver bind?"
hr
DMESG=$(dmesg 2>/dev/null | grep -iE "imx477|imx219|imx296|ov9281|ov5647" | tail -10)
if [ -n "$DMESG" ]; then
    echo "$DMESG"
else
    echo "  no sensor driver messages in dmesg."
    echo "  (dmesg may need root — retry with sudo before concluding anything)"
    if [ "$PLATFORM" = "jetson" ]; then
        echo
        echo "  On Jetson this usually means a device-tree overlay is missing, or the"
        echo "  ribbon is wrong: the Pi HQ camera is 15-pin and many carriers are 22-pin,"
        echo "  so it needs an adapter. Check ARK's docs for which CSI ports are wired."
    fi
fi
echo

# ---- video nodes ------------------------------------------------------------
hr
echo " 2. Video device nodes"
hr
if ls /dev/video* >/dev/null 2>&1; then
    ls -l /dev/video*
else
    echo "  no /dev/video* nodes."
fi
echo
if have v4l2-ctl; then
    echo "-- v4l2-ctl --list-devices --"
    v4l2-ctl --list-devices 2>&1 | sed 's/^/  /'
    echo
    for dev in /dev/video0 /dev/video1; do
        [ -e "$dev" ] || continue
        echo "-- formats on $dev --"
        # Raw Bayer only (RG10/BG10/...) means no ISP in the path: you debayer
        # yourself, which costs CPU and complexity. NV12/GRAY8 means you are fine.
        v4l2-ctl -d "$dev" --list-formats-ext 2>&1 | head -30 | sed 's/^/  /'
        echo
    done
else
    echo "  v4l2-ctl not installed:  sudo apt-get install v4l-utils"
fi

# ---- platform-specific enumeration -----------------------------------------
if [ "$PLATFORM" = "pi" ] || have rpicam-hello || have libcamera-hello; then
    hr
    echo " 3. libcamera enumeration (Pi / Bookworm)"
    hr
    if have rpicam-hello; then
        rpicam-hello --list-cameras 2>&1 | sed 's/^/  /'
    elif have libcamera-hello; then
        libcamera-hello --list-cameras 2>&1 | sed 's/^/  /'
    else
        echo "  rpicam-hello not found:  sudo apt-get install rpicam-apps"
    fi
    echo
    if have gst-inspect-1.0; then
        if gst-inspect-1.0 libcamerasrc >/dev/null 2>&1; then
            echo "  libcamerasrc element: PRESENT"
        else
            echo "  libcamerasrc element: MISSING"
            echo "    sudo apt-get install gstreamer1.0-libcamera"
            echo "    (rpicamsrc is the legacy Buster element and is NOT the answer on Bookworm)"
        fi
    fi
    echo
fi

if [ "$PLATFORM" = "jetson" ]; then
    hr
    echo " 3. Argus enumeration (Jetson)"
    hr
    if have gst-inspect-1.0; then
        if gst-inspect-1.0 nvarguscamerasrc >/dev/null 2>&1; then
            echo "  nvarguscamerasrc element: PRESENT"
        else
            echo "  nvarguscamerasrc element: MISSING — JetPack multimedia API not installed"
        fi
    fi
    systemctl is-active nvargus-daemon >/dev/null 2>&1 \
        && echo "  nvargus-daemon: active" \
        || echo "  nvargus-daemon: NOT active — sudo systemctl restart nvargus-daemon"
    echo
    # Argus only prints its mode table while starting a real capture. Which modes
    # exist decides the FOV, and none of them is likely to be 720p — so pin
    # sensor-mode in the config rather than letting Argus choose for you.
    echo "-- sensor modes Argus reports --"
    timeout 25 gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=1 \
        ! 'video/x-raw(memory:NVMM)' ! fakesink 2>&1 \
        | grep -E "GST_ARGUS:.*FR =" | sed 's/^GST_ARGUS: /  mode: /' \
        || echo "  (none reported — see the capture test below)"
    echo
    echo "  None of these is 1280x720? Expected. Request a REAL mode from the sensor"
    echo "  and let nvvidconv scale down — see config/jetson_down.yaml. Which mode you"
    echo "  pick changes the field of view, and a cropped mode narrows it silently."
    echo
fi

# ---- decisive capture test --------------------------------------------------
if [ "$CAPTURE" = true ]; then
    hr
    echo " 4. Decisive test — can we actually pull frames?"
    hr
    try_pipeline() {
        local name="$1"; shift
        echo "-- $name --"
        if timeout 20 gst-launch-1.0 "$@" >/dev/null 2>&1; then
            echo "   WORKS. Use this source in the gst: line of your config."
            return 0
        fi
        echo "   no frames."
        return 1
    }

    if [ "$PLATFORM" = "jetson" ]; then
        try_pipeline "nvarguscamerasrc (Jetson ISP — the good path)" \
            nvarguscamerasrc num-buffers=30 \
            ! 'video/x-raw(memory:NVMM),width=1280,height=720,framerate=30/1' \
            ! nvvidconv ! video/x-raw,format=NV12 ! fakesink \
        && echo "   -> config/jetson_down.yaml works as shipped."
    fi

    if have gst-inspect-1.0 && gst-inspect-1.0 libcamerasrc >/dev/null 2>&1; then
        try_pipeline "libcamerasrc (Pi / Bookworm)" \
            libcamerasrc num-buffers=30 \
            ! video/x-raw,width=1280,height=720,format=NV12 ! fakesink \
        && echo "   -> config/pi_down.yaml works as shipped."
    fi

    if [ -e /dev/video0 ]; then
        try_pipeline "v4l2src /dev/video0 (USB / UVC)" \
            v4l2src device=/dev/video0 num-buffers=30 \
            ! video/x-raw ! videoconvert ! video/x-raw,format=GRAY8 ! fakesink
    fi
    echo
else
    hr
    echo " 4. Decisive test — skipped"
    hr
    echo "  Re-run with --capture to actually pull frames. That test is what"
    echo "  settles the ingest path; everything above is circumstantial."
    echo
fi

hr
echo " Interpretation"
hr
cat <<'NOTE'
  Argus/libcamera produces NV12 frames  -> best case. Use the shipped config.
  Only raw Bayer (RG10/BG10) from V4L2  -> no ISP in the path. You debayer
                                           yourself: more CPU, more complexity.
                                           Insert bayer2rgb + videoconvert into
                                           the gst: line and expect to pay for it.
  Nothing enumerates                    -> device tree and/or ribbon. This is
                                           hardware, not this service.

  No camera at all? The detector is a pure vision process — replay a file:
    gst: "filesrc location=/tmp/clip.mp4 ! decodebin ! videoconvert !
          video/x-raw,format=GRAY8 ! appsink name=sink drop=true max-buffers=2"
  and see tools/make_test_video.py for a synthetic clip with known ground truth.
NOTE
echo
