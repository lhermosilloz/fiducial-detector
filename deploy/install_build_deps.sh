#!/bin/bash
################################################################################
# install_build_deps.sh — build dependencies, both platforms, one script.
#
# The entire dependency surface of this service is apt-installable on JetPack 6
# and Raspberry Pi OS Bookworm. That is not an accident — it is the property
# that lets this run on a Pi where jetson-vision-service (TensorRT) cannot, and
# it is why this script has no per-platform package list beyond the camera
# element.
#
#   sudo ./deploy/install_build_deps.sh
################################################################################

set -euo pipefail

if [ "$EUID" -ne 0 ]; then
    echo "run as root:  sudo $0" >&2
    exit 1
fi

PLATFORM="generic"
if [ -f /etc/nv_tegra_release ]; then
    PLATFORM="jetson"
elif grep -qi raspberry /proc/device-tree/model 2>/dev/null; then
    PLATFORM="pi"
fi
echo "[deps] platform: ${PLATFORM}"

apt-get update

# ---- common: the whole dependency surface --------------------------------
#
# libopencv-dev on Debian/Ubuntu bundles the contrib modules, which is what
# supplies cv::aruco on OpenCV < 4.7 (Bookworm ships 4.6). JetPack ships its own
# OpenCV >= 4.7 where aruco lives in core objdetect. include/aruco_compat.h
# handles both; nothing here needs to choose.
apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    pkg-config \
    git \
    libopencv-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libyaml-cpp-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-tools \
    gstreamer1.0-plugins-base \
    gstreamer1.0-plugins-good \
    v4l-utils

# ---- the camera element: the one genuine per-platform package ------------
case "${PLATFORM}" in
  pi)
    # Bookworm is libcamera. rpicamsrc is the legacy Buster/Broadcom element and
    # is not available or wanted here.
    apt-get install -y --no-install-recommends \
        gstreamer1.0-libcamera \
        libcamera-tools \
        rpicam-apps || \
      echo "[deps] WARNING: libcamera packages unavailable — check you are on Bookworm"
    ;;
  jetson)
    # nvarguscamerasrc ships with the JetPack multimedia API, which is already
    # on the board. Nothing to install; verify instead.
    if gst-inspect-1.0 nvarguscamerasrc >/dev/null 2>&1; then
        echo "[deps] nvarguscamerasrc: present"
    else
        echo "[deps] WARNING: nvarguscamerasrc missing. Install the JetPack"
        echo "[deps]          multimedia API (nvidia-l4t-gstreamer) via SDK Manager."
    fi
    ;;
  *)
    echo "[deps] generic host — install a USB camera or use file replay."
    apt-get install -y --no-install-recommends gstreamer1.0-plugins-bad || true
    ;;
esac

# ---- optional: the python test harness -----------------------------------
apt-get install -y --no-install-recommends python3-protobuf python3-numpy || true

echo
echo "[deps] done."
echo
echo "  OpenCV    : $(pkg-config --modversion opencv4 2>/dev/null || echo '?')"
echo "  GStreamer : $(pkg-config --modversion gstreamer-1.0 2>/dev/null || echo '?')"
echo "  protoc    : $(protoc --version 2>/dev/null || echo '?')"
echo
echo "Next:"
echo "  ./deploy/build.sh"
echo "  ./tools/check_camera.sh --capture      # settle the ingest path first"
