#!/usr/bin/env bash
# Source ROS/workspace before invoking. This check never stops running services.
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
PLUGIN_DIR="$(ros2 pkg prefix x_bot_gazebo)/lib"
if [[ -d "$ROOT_DIR/.cache/embree/usr/lib/x86_64-linux-gnu" ]]; then
    export LD_LIBRARY_PATH="$ROOT_DIR/.cache/embree/usr/lib/x86_64-linux-gnu${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
for library in libembodied_gazebo_backend.so libgazebo_drives.so; do
    path="$PLUGIN_DIR/$library"
    [[ -f "$path" ]] || {
        echo "错误：Gazebo 插件未构建：$path；安装 README 中的 Gazebo 依赖后重建 x_bot_gazebo。" >&2
        exit 1
    }
    dependencies="$(ldd "$path" 2>&1)" || { echo "$dependencies" >&2; exit 1; }
    if missing="$(awk '/=> not found/{print $1}' <<<"$dependencies")" && [[ -n "$missing" ]]; then
        echo "错误：Gazebo 插件 $library 缺少动态库：" >&2
        echo "$missing" >&2
        if [[ "$missing" == *libembree4.so.4* ]]; then
            echo "请手动执行：sudo apt install libembree4-4 libembree-dev" >&2
        fi
        echo "依赖检查未通过，不启动 Gazebo。安装说明见 README.md 的 Gazebo 依赖。" >&2
        exit 1
    fi
done
info="$(gz plugin --info --plugin "$PLUGIN_DIR/libembodied_gazebo_backend.so" 2>&1)" || {
    echo "$info" >&2
    exit 1
}
if [[ "$info" != *embodied::GazeboBackend* ]]; then
    echo "错误：Gazebo 插件未注册 embodied::GazeboBackend：" >&2
    echo "$info" >&2
    exit 1
fi
