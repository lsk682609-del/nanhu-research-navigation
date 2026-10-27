#!/usr/bin/env bash
set -euo pipefail

cd "$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF "$@"
