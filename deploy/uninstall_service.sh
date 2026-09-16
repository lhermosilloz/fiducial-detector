#!/bin/bash
################################################################################
# uninstall_service.sh — remove everything install_service.sh created.
#
# The counterpart to install_service.sh, for verifying a deployment from a known
# clean state rather than on top of whatever a previous attempt left behind.
#
#   sudo ./deploy/uninstall_service.sh            # keep config and service user
#   sudo ./deploy/uninstall_service.sh --purge    # also remove those
#   sudo ./deploy/uninstall_service.sh --yes      # skip the confirmation prompt
#
# The config is preserved by default because it holds this vehicle's calibrated
# intrinsics, which are measured, not reproducible from the repository.
#
# Only the paths install_service.sh writes are touched. Nothing else is removed.
################################################################################

set -euo pipefail

PURGE=false
ASSUME_YES=false
for arg in "$@"; do
    case "$arg" in
        --purge) PURGE=true ;;
        --yes|-y) ASSUME_YES=true ;;
        --help|-h) sed -n '2,/^####/p' "$0" | sed 's/^# \?//'; exit 0 ;;
        *) echo "unknown argument: $arg" >&2; exit 1 ;;
    esac
done

[ "$EUID" -eq 0 ] || { echo "run as root:  sudo $0 $*" >&2; exit 1; }

UNIT="/etc/systemd/system/fiducial-detector-service.service"
BINARY="/usr/local/bin/fiducial-detector-service"
CONF_DIR="/etc/fiducial-detector-service"
DOC_DIR="/usr/share/doc/fiducial-detector-service"
TOOLS_DIR="/usr/local/share/fiducial-detector-service"

echo
echo "This will remove:"
echo "    ${UNIT}"
echo "    ${BINARY}"
echo "    ${TOOLS_DIR}"
echo "    ${DOC_DIR}"
if [ "$PURGE" = true ]; then
    echo "    ${CONF_DIR}          (--purge: INCLUDING CALIBRATED INTRINSICS)"
    echo "    the 'fiducial' service user"
else
    echo
    echo "Preserved (use --purge to remove):"
    echo "    ${CONF_DIR}"
    echo "    the 'fiducial' service user"
fi
echo

if [ "$ASSUME_YES" != true ]; then
    read -r -p "Proceed? [y/N] " reply
    case "$reply" in [yY]*) ;; *) echo "aborted"; exit 0 ;; esac
fi

echo "[1/5] stopping and disabling the service"
systemctl stop fiducial-detector-service 2>/dev/null || true
systemctl disable fiducial-detector-service 2>/dev/null || true
# A manual run holds the camera just as firmly as the service does, and is the
# usual reason a fresh install cannot open a capture session.
pkill -f 'fiducial-detector-service' 2>/dev/null || true

echo "[2/5] removing the unit"
rm -f "$UNIT"
systemctl daemon-reload
systemctl reset-failed 2>/dev/null || true

echo "[3/5] removing the binary"
rm -f "$BINARY"

echo "[4/5] removing tools and documentation"
rm -rf "$TOOLS_DIR" "$DOC_DIR"

echo "[5/5] config and service user"
if [ "$PURGE" = true ]; then
    rm -rf "$CONF_DIR"
    echo "      removed ${CONF_DIR}"
    if getent passwd fiducial >/dev/null; then
        userdel fiducial 2>/dev/null || true
        echo "      removed user 'fiducial'"
    fi
    if getent group fiducial >/dev/null; then
        groupdel fiducial 2>/dev/null || true
    fi
else
    echo "      kept ${CONF_DIR} and user 'fiducial'"
fi

# Argus keeps per-client state, and a service that has been restart-looping can
# leave the daemon unable to hand out a capture session to the next client.
if systemctl list-unit-files nvargus-daemon.service >/dev/null 2>&1; then
    echo
    echo "[argus] restarting nvargus-daemon so the next install starts clean"
    systemctl restart nvargus-daemon || true
fi

cat <<NOTE

Uninstalled.

Verify nothing is left holding the camera:
    ps aux | grep -E 'fiducial-detector|gst-launch' | grep -v grep

Then reinstall from a clean build:
    cd <source tree>
    rm -rf build
    ./deploy/build.sh
    sudo ./deploy/jetson/install_service.sh --start

NOTE
