#!/usr/bin/env bash
# Offline Rerun export; does not open a GUI or connect to cameras.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
exec "$HERE/venv/bin/python" "$HERE/orbbec/view_map2d_rerun.py" "$@"
