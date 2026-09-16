#!/bin/bash
################################################################################
# deploy_remote.sh — copy this tree to a board, build there, install there.
#
# Builds NATIVELY on the target rather than cross-compiling. That is a
# deliberate choice and it is only affordable because the dependency surface is
# small and apt-installable: OpenCV, protobuf, GStreamer, yaml-cpp. A Pi 4
# builds this in a couple of minutes, an Orin NX in well under one. Setting up
# and maintaining two cross toolchains to save that is a bad trade — and a
# native build is also the only one that proves the board's own OpenCV works,
# which is exactly the thing that differs between these platforms.
#
#   ./deploy/deploy_remote.sh jetson --host 192.168.55.1 --user jetson
#   ./deploy/deploy_remote.sh pi     --host raspberrypi.local --user pi --start
#
# Options:
#   --host <h>   target hostname or IP          (required)
#   --user <u>   ssh user (default: jetson / pi by platform)
#   --dir <d>    remote source dir (default: ~/fiducial-detector)
#   --deps       run install_build_deps.sh on the target first (needs sudo there)
#   --start      start the service after installing
#   --no-install build only; skip the systemd install
################################################################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PLATFORM="${1:-}"; shift || true
HOST=""; USER_NAME=""; REMOTE_DIR="fiducial-detector"
DO_DEPS=false; DO_START=false; DO_INSTALL=true

case "$PLATFORM" in
    jetson) USER_NAME="jetson" ;;
    pi)     USER_NAME="pi" ;;
    *) echo "usage: $0 {jetson|pi} --host <h> [options]" >&2; exit 1 ;;
esac

while [ $# -gt 0 ]; do
    case "$1" in
        --host) HOST="$2"; shift 2 ;;
        --user) USER_NAME="$2"; shift 2 ;;
        --dir)  REMOTE_DIR="$2"; shift 2 ;;
        --deps) DO_DEPS=true; shift ;;
        --start) DO_START=true; shift ;;
        --no-install) DO_INSTALL=false; shift ;;
        --help|-h) sed -n '2,/^####/p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 1 ;;
    esac
done

[ -n "$HOST" ] || { echo "--host is required" >&2; exit 1; }

TARGET="${USER_NAME}@${HOST}"
echo "[deploy] ${PLATFORM} -> ${TARGET}:~/${REMOTE_DIR}"

command -v rsync >/dev/null || { echo "rsync not installed locally" >&2; exit 1; }

# Source only. build/ is excluded on purpose: a host-built x86 tree copied onto
# an aarch64 board is the classic way to spend an hour debugging a linker error
# that is really an architecture mismatch.
rsync -az --delete \
    --exclude 'build/' \
    --exclude '.git/' \
    --exclude '.git' \
    --exclude '*.pyc' \
    --exclude '__pycache__/' \
    --exclude 'tools/fiducial_pb2.py' \
    "${ROOT}/" "${TARGET}:${REMOTE_DIR}/"

echo "[deploy] source synced"

if [ "$DO_DEPS" = true ]; then
    echo "[deploy] installing build deps on target (sudo required there)"
    ssh -t "$TARGET" "cd ${REMOTE_DIR} && sudo ./deploy/install_build_deps.sh"
fi

echo "[deploy] building on target"
ssh "$TARGET" "cd ${REMOTE_DIR} && ./deploy/build.sh"

if [ "$DO_INSTALL" = true ]; then
    echo "[deploy] installing service on target (sudo required there)"
    START_FLAG=""
    [ "$DO_START" = true ] && START_FLAG="--start"
    ssh -t "$TARGET" "cd ${REMOTE_DIR} && sudo ./deploy/${PLATFORM}/install_service.sh ${START_FLAG}"
fi

cat <<NOTE

[deploy] done.

  ssh ${TARGET}
  journalctl -u fiducial-detector-service -f
  ~/${REMOTE_DIR}/tools/check_camera.sh --capture
  ~/${REMOTE_DIR}/tools/recv_fiducial.py --raw

NOTE
