#!/usr/bin/env bash
set -eo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
MODE="${1:-all}"
[[ $# == 0 ]] || shift
BUILD=false
ARGS=()
for arg in "$@"; do
    if [[ "$arg" == --build ]]; then BUILD=true; else ARGS+=("$arg"); fi
done
source /opt/ros/jazzy/setup.bash
if [[ "$BUILD" == true ]]; then
    (cd "$ROOT_DIR" && colcon build --symlink-install --packages-up-to x_bot_isaac x_bot_moveit_config x_bot_manipulation)
fi
[[ ! -f "$ROOT_DIR/install/setup.bash" ]] || source "$ROOT_DIR/install/setup.bash"
case "$MODE" in
    all|collect|grasp) exec /usr/bin/python3 "$ROOT_DIR/src/manipulation/handeye_demo/launch.py" "$MODE" "${ARGS[@]}" ;;
    assets|calibrate|evaluate|report) exec /usr/bin/python3 "$ROOT_DIR/src/manipulation/handeye_demo/run.py" "$MODE" "${ARGS[@]}" ;;
    *) echo '用法：./start_handeye_demo.sh {all|collect|calibrate|evaluate|grasp|report} --output DIR [--headless] [--build] [--config FILE] [--domain-id 73]' >&2; exit 2 ;;
esac
