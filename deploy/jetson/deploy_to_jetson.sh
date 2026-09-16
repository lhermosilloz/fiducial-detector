#!/bin/bash
# Deploy to a jetson from a development machine. Thin wrapper; see
# deploy/deploy_remote.sh for the options.
#   ./deploy/jetson/deploy_to_jetson.sh --host <ip> [--deps] [--start]
exec "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)/deploy_remote.sh" jetson "$@"
