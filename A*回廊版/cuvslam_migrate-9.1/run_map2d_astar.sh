#!/usr/bin/env bash
# Standalone two-point A* planner. Run on the desktop for the interactive window.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
exec "$HERE/venv/bin/python" "$HERE/orbbec/plan_map2d.py" "$@"
