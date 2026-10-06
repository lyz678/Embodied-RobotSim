#!/usr/bin/env bash
# Explicit target-machine setup; no sudo, ROS installation or system mutation.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
DEST="$ROOT_DIR/src/vendor"
command -v vcs >/dev/null || { echo 'Install python3-vcstool first.' >&2; exit 1; }
mkdir -p "$DEST"
vcs import "$DEST" < "$ROOT_DIR/scripts/setup/vendor.repos"
actual="$(git -C "$DEST/FAST_LIO_ROS2" rev-parse HEAD)"
[[ "$actual" == 2fffc570a25d0df172720bac034fbdb6a13d2162 ]] || {
    echo "Existing checkout is not pinned: $actual. Resolve manually; not overwriting it." >&2; exit 1;
}
git -C "$DEST/FAST_LIO_ROS2" submodule update --init --recursive
echo 'Dependencies ready. Run rosdep install --from-paths src --ignore-src -r -y, then colcon build.'
