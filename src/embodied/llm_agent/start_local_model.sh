#!/usr/bin/env bash
set -euo pipefail
ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/../../.." && pwd)"
QWEN_HOME="${QWEN_HOME:-$ROOT_DIR/.cache/qwen}"
SERVER="$QWEN_HOME/llama.cpp/build/bin/llama-server"
MODEL="$QWEN_HOME/models/Qwen3.5-4B-Q4_K_M.gguf"
PROJECTOR="$QWEN_HOME/models/mmproj-F16.gguf"
for file in "$SERVER" "$MODEL" "$PROJECTOR"; do
    [[ -f "$file" ]] || { echo "缺少 $file；请先执行 python3 src/embodied/llm_agent/setup_local_model.py" >&2; exit 1; }
done
[[ -x "$SERVER" ]] || { echo "模型服务不可执行：$SERVER" >&2; exit 1; }
[[ "${1:-}" != --check ]] || exit 0
# One slot, bounded context and batch size for a GPU shared with Isaac Sim.
exec "$SERVER" --model "$MODEL" --mmproj "$PROJECTOR" \
    --alias embodied-qwen-local --host 127.0.0.1 --port 8000 \
    --n-gpu-layers 99 --ctx-size 4096 --parallel 1 \
    --batch-size 256 --ubatch-size 128 --image-max-tokens 1024 --jinja \
    --reasoning off "$@"
