#!/bin/bash
# Install fiducial-detector-service on pi.
#
# Thin wrapper: the installer is shared because the install is identical across
# platforms. This file only selects which config ships as the starting point;
# that config's `gst:` line is the entire platform difference.
#
#   sudo ./deploy/pi/install_service.sh [--start]
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/install_service.sh" pi "$@"
