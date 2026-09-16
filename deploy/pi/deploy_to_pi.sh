#!/bin/bash
# Deploy to a pi from a development machine. Thin wrapper; see
# deploy/deploy_remote.sh for the options.
#   ./deploy/pi/deploy_to_pi.sh --host <ip> [--deps] [--start]
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/deploy_remote.sh" pi "$@"
