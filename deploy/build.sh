#!/bin/bash
################################################################################
# build.sh — build the service. Same script, both platforms.
#
# There is no per-platform build. No CUDA flags, no TensorRT paths, no
# cross-compile toolchain: it is plain CMake against apt packages, so it builds
# natively on the Jetson, on the Pi, and on an x86 laptop for testing.
#
#   ./deploy/build.sh [--clean] [--nexus-protos <path>]
#
# --nexus-protos <path>   Build against the canonical fiducial.proto in a
#                         nexus-protos checkout instead of the in-repo bootstrap
#                         copy. Do this once the proto has landed upstream
# — two copies of a wire contract kept in
#                         sync by hand is the hazard this service exists to
#                         avoid repeating.
################################################################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD="${ROOT}/build"
CLEAN=false
NEXUS_PROTOS=""

while [ $# -gt 0 ]; do
    case "$1" in
        --clean) CLEAN=true; shift ;;
        --nexus-protos) NEXUS_PROTOS="$2"; shift 2 ;;
        --help|-h) sed -n '2,/^####/p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown argument: $1" >&2; exit 1 ;;
    esac
done

[ "$CLEAN" = true ] && rm -rf "$BUILD"

CMAKE_ARGS=(-DCMAKE_BUILD_TYPE=Release)
[ -n "$NEXUS_PROTOS" ] && CMAKE_ARGS+=("-DNEXUS_PROTOS_DIR=$(cd "$NEXUS_PROTOS" && pwd)")

# The Pi 4 has 4 cores and, often, no swap worth the name. Building protobuf
# generated code with -j4 on 2 GB can OOM the box; cap by available memory.
JOBS="$(nproc)"
MEM_GB="$(awk '/MemTotal/ {printf "%d", $2/1024/1024}' /proc/meminfo)"
if [ "$MEM_GB" -le 2 ] && [ "$JOBS" -gt 2 ]; then
    JOBS=2
    echo "[build] ${MEM_GB} GB RAM — capping to -j${JOBS} to avoid the OOM killer"
fi

cmake -S "$ROOT" -B "$BUILD" "${CMAKE_ARGS[@]}"
cmake --build "$BUILD" -j"${JOBS}"

echo
echo "[build] ok -> ${BUILD}/fiducial-detector-service"
"${BUILD}/fiducial-detector-service" --version
echo
echo "Next:"
echo "  sudo ./deploy/jetson/install_service.sh     # or deploy/pi/install_service.sh"
