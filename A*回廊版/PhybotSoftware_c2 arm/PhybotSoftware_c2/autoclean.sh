#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROOT_DIR="$SCRIPT_DIR"
BUILD_DIR_NAME="${BUILD_DIR_NAME:-build}"
BUILD_DIR="$ROOT_DIR/$BUILD_DIR_NAME"

mkdir -p "$BUILD_DIR"
echo "Cleaning build directory: $BUILD_DIR"
find "$BUILD_DIR" -mindepth 1 -maxdepth 1 -exec rm -rf {} +
