#!/bin/bash
################################################################################
# install_service.sh — install binary, config and systemd unit.
#
# One installer for both platforms. The platform argument selects which config is
# installed and nothing else: same binary, same unit, same paths. A second
# per-platform branch here would indicate the portability property has been
# broken upstream; fix it there rather than here.
#
#   sudo ./deploy/install_service.sh jetson [--start]
#   sudo ./deploy/install_service.sh pi     [--start]
#
# Usually invoked through deploy/jetson/install_service.sh or deploy/pi/…
################################################################################

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

PLATFORM="${1:-}"
shift || true
START=false
for arg in "$@"; do
    case "$arg" in
        --start) START=true ;;
        *) echo "unknown argument: $arg" >&2; exit 1 ;;
    esac
done

case "$PLATFORM" in
    jetson) CONFIG_SRC="${ROOT}/config/jetson_down.yaml" ;;
    pi)     CONFIG_SRC="${ROOT}/config/pi_down.yaml" ;;
    *) echo "usage: $0 {jetson|pi} [--start]" >&2; exit 1 ;;
esac

[ "$EUID" -eq 0 ] || { echo "run as root:  sudo $0 $PLATFORM $*" >&2; exit 1; }

BINARY_SRC="${ROOT}/build/fiducial-detector-service"
UNIT_SRC="${ROOT}/systemd/fiducial-detector-service.service"

BINARY_DST="/usr/local/bin/fiducial-detector-service"
UNIT_DST="/etc/systemd/system/fiducial-detector-service.service"
CONF_DIR="/etc/fiducial-detector-service"
CONF_DST="${CONF_DIR}/config.yaml"
DOC_DIR="/usr/share/doc/fiducial-detector-service"
TOOLS_DIR="/usr/local/share/fiducial-detector-service/tools"

if [ ! -x "$BINARY_SRC" ]; then
    echo "[ERROR] no binary at ${BINARY_SRC} — run ./deploy/build.sh first" >&2
    exit 1
fi

echo
echo "============================================"
echo " fiducial-detector-service installer (${PLATFORM})"
echo "============================================"

# ---- 1. service account --------------------------------------------------
# The unit runs unprivileged; the video group is what grants camera access on
# both platforms. render covers the Pi's libcamera/DRM path.
if ! getent passwd fiducial >/dev/null; then
    echo "[1/6] creating service account 'fiducial'"
    useradd --system --no-create-home --shell /usr/sbin/nologin fiducial
else
    echo "[1/6] service account 'fiducial' exists"
fi
for grp in video render; do
    getent group "$grp" >/dev/null && usermod -aG "$grp" fiducial
done

# ---- 2. binary -----------------------------------------------------------
echo "[2/6] binary  -> ${BINARY_DST}"
install -m 0755 "$BINARY_SRC" "$BINARY_DST"

# ---- 3. config -----------------------------------------------------------
echo "[3/6] config  -> ${CONF_DST}"
install -d -m 0755 -o fiducial -g fiducial "$CONF_DIR"
if [ -f "$CONF_DST" ]; then
    # Never overwrite an existing config: it holds this vehicle's calibrated
    # intrinsics.
    echo "      ${CONF_DST} already exists — LEFT ALONE"
    echo "      reference config: ${CONFIG_SRC}"
    cp "$CONFIG_SRC" "${CONF_DIR}/$(basename "$CONFIG_SRC").reference"
    echo "      copied alongside as $(basename "$CONFIG_SRC").reference for diffing"
else
    install -m 0644 -o fiducial -g fiducial "$CONFIG_SRC" "$CONF_DST"
    echo "      installed from $(basename "$CONFIG_SRC")"
    echo
    echo "      NOTE: the intrinsics in it are placeholders."
    echo "      Calibrate this camera and lens before flight: a 10% error in"
    echo "      fx/fy or tags.default_size_m produces a 10% range error on"
    echo "      every detection. Use tools/calibrate_camera.py."
fi

# ---- 4. tools and docs ---------------------------------------------------
echo "[4/6] tools   -> ${TOOLS_DIR}"
install -d -m 0755 "$TOOLS_DIR" "$DOC_DIR"
install -m 0755 "${ROOT}/tools/"*.py "${ROOT}/tools/"*.sh "$TOOLS_DIR/" 2>/dev/null || true
install -m 0644 "${ROOT}/README.md" "$DOC_DIR/" 2>/dev/null || true
install -d -m 0755 "${DOC_DIR}/protos"
install -m 0644 "${ROOT}/protos/fiducial.proto" "${DOC_DIR}/protos/" 2>/dev/null || true

# ---- 5. systemd unit -----------------------------------------------------
echo "[5/6] unit    -> ${UNIT_DST}"
install -m 0644 "$UNIT_SRC" "$UNIT_DST"
systemctl daemon-reload
systemctl enable fiducial-detector-service >/dev/null
echo "      enabled (starts on boot)"

# ---- 6. validate ---------------------------------------------------------
echo "[6/6] validating installed config"
if "$BINARY_DST" -c "$CONF_DST"; then
    echo "      config OK"
else
    echo "      [WARN] config is INVALID — the service will refuse to start."
    echo "             Edit ${CONF_DST} and re-check with:"
    echo "               ${BINARY_DST} -c ${CONF_DST}"
fi

if [ "$START" = true ]; then
    echo
    echo "[INFO] starting..."
    systemctl restart fiducial-detector-service
    sleep 2
    systemctl status fiducial-detector-service --no-pager || true
fi

cat <<NOTE

============================================
 Installed
============================================

  binary  : ${BINARY_DST}
  config  : ${CONF_DST}
  unit    : ${UNIT_DST}
  tools   : ${TOOLS_DIR}

  systemctl start   fiducial-detector-service
  systemctl status  fiducial-detector-service
  journalctl -u fiducial-detector-service -f

  Verify it is actually publishing (from this box or from Nexus's box):
    ${TOOLS_DIR}/recv_fiducial.py --raw

  Drive Nexus with no camera at all:
    ${TOOLS_DIR}/fake_publisher.py --profile descent

  Publishing to 127.0.0.1:5602. 5601 belongs to jetson-vision-service.

NOTE
