# 机器人 Agent：本地 Qwen / DashScope

本地方案使用 **Qwen3.5-4B Q4_K_M + F16 视觉投影 + llama.cpp CUDA**。
同一实例处理文字、工具调用和相机图片，默认 4096 上下文、单并发、关闭 thinking，
图片最长边 640 像素、最多 1024 图像 tokens。模型服务只监听 `127.0.0.1:8000`。

## 安装与启动

在项目根目录执行。需要已有 NVIDIA 驱动、CUDA Toolkit（含 nvcc）、
`build-essential cmake git curl libssl-dev`。本机 RTX 5060 Ti 使用 CUDA 架构 120。

```bash
# 下载模型、视觉投影并编译 CUDA 服务；首次需要几分钟
python3 src/embodied/llm_agent/setup_local_model.py
# 完整具身入口，默认使用本地模型；无需云端 API Key
./start_embodied.sh --llm local
# 使用 DashScope
export DASHSCOPE_API_KEY='你的 API Key'
./start_embodied.sh --llm cloud
```

Web/Agent Python 和 ROS 依赖仍按根目录 README 安装；本地推理不需要 PyTorch、
vLLM 或 TensorRT-LLM。安装器固定 llama.cpp 提交和模型 revision，校验两份模型
SHA256；约下载 3.4 GB。默认文件位于 `.cache/qwen`，不纳入 Git。
`QWEN_HOME` 可改存储目录，安装和启动须使用同一值；其他 GPU 用 `--cuda-arch` 指定架构。
安装器不修改系统驱动、系统 CUDA 或机器人依赖。

完整入口先检查模型文件与 Web 依赖，再清理上次会话；启动模型服务并等待模型可用后
启动 Agent。模型日志在服务终端，headless 时在 `log/isaac/Local_Qwen_Model.log`
（Gazebo 为 `log/gazebo/`）。`./stop_robot_sim.sh` 会停止项目启动的模型服务。
仅修改环境变量不会改变已运行的 Agent，需重新启动或单独重启 Agent。

Agent 的聊天服务不等待定位/Nav2 就绪，避免定位启动超时导致网页反复断开。
导航、抓取、底盘移动和机械臂归位在执行时检查新鲜的 `/localization/ready`；
未就绪时返回明确错误，聊天、场景识别和停止指令仍可用。`/api/status` 返回
`localization_ready`，底盘安全门继续独立检查每条速度命令；导航还要求 Nav2 action 服务可用。

## 单独启动（保留现有仿真）

```bash
# 终端 1：无需 ROS 环境
bash src/embodied/llm_agent/start_local_model.sh
# 终端 2：选择你的 Agent Python 环境
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export LLM_BACKEND=local
python3 src/embodied/llm_agent/wait_local_model.py
# 如使用系统目录环境，将 .venv/agent/bin/python 换为 /opt/embodied-agent/bin/python
.venv/agent/bin/python src/embodied/llm_agent/agent_server.py --ros-args -p use_sim_time:=true
```

如果已有 Agent 占用 8889，须先停止该 Agent；不要重复启动。
Web UI 仍使用 `http://localhost:8888`，Agent WebSocket 为 `ws://localhost:8889/ws/chat`。

## 明确的取物指令

例如“到厨房拿可乐”，入口使用相机深度和 manipulation YOLOE 配置，
FAST-LIO 负责定位。当前 FAST-LIO 语义分支没有发布 GraspNet 所需的
三维检测和彩色点云，具身入口会拒绝该组合。

明确的单步取物意图直接执行“导航→抓取”，复杂指令仍由模型处理。
具身导航启用 `precise_parking:=true`，目标容差为 10 cm / 0.10 rad，
采用低速 RPP 跟踪和末端朝向控制；普通导航入口配置不变。
Agent 还检查新鲜 map→base_footprint 变换、实际停车误差（≤12 cm / 0.15 rad）
和连续一秒的底盘位姿稳定性。
抓取使用多帧新鲜候选、预抓取姿态、直线接近、竖直抬升和持物回 Home，
任何阶段失败都停止并报告，不会继续宣告成功。夹爪夹持检查仅是关节接触代理，
实际成功仍需查看物体抬起和回 Home 后的稳定持握。

| 环境变量 | 用途 / 默认值 |
|---|---|
| `LLM_BACKEND` | `local` / `cloud`；根目录具身入口默认 local，直接运行 Agent 默认 cloud |
| `AGENT_API_BASE` | 本地默认 `http://127.0.0.1:8000/v1`；云端默认 DashScope 地址 |
| `AGENT_API_KEY` | 本地默认占位值 `local`；云端未设置时读取 `DASHSCOPE_API_KEY` |
| `AGENT_MODEL` / `VL_MODEL` | 本地默认同为 `embodied-qwen-local`；云端默认 `qwen3.6-flash` |
| `AGENT_PYTHON` | 完整入口使用的 Python，支持 `/opt/embodied-agent/bin/python` |
| `QWEN_HOME` | 模型和 llama.cpp 路径，默认工作区 `.cache/qwen` |

从自定义云端模型切换本地时，清除旧的 `AGENT_MODEL`、`VL_MODEL`、`AGENT_API_BASE`
覆盖项，或设置成表中的本地值。自定义 endpoint 时，完整入口仍会启动本机模型；
远程服务应使用单独启动方式。

## 验证

文本、原生工具调用、工具结果回传和图片测试只调用模型，**不发送机器人动作**：

```bash
LLM_BACKEND=local .venv/agent/bin/python src/embodied/llm_agent/check_local_model.py \
    --image /path/to/camera.jpg --output .cache/qwen/validation.json
# 已 source ROS / 工作区时，加 --agent 验证现有 Agent 循环（仍使用模拟机器人）
python3 -m unittest discover -s tests -p test_llm_config.py
```

接入验证需检查返回标准 `tool_calls`，不能把模型输出的一段 JSON 文本当作工具调用。
本地 Agent 限制输出为 512 tokens，并保留最近四条、每条最多 800 字符的会话历史；
极长指令或多轮工具结果仍可能超出 4096 上下文，此时 API 返回错误，不能当作动作成功。
量化降低权重占用，实际显存还包含视觉编码器、KV Cache 和计算缓冲，应在仿真、
感知与建图同时运行时检查 `nvidia-smi`。单独启动时可传 `--ctx-size 8192` 覆盖默认值，
但会增加资源占用。

上游：[原始模型](https://huggingface.co/Qwen/Qwen3.5-4B)、
[社区 GGUF / 投影文件](https://huggingface.co/unsloth/Qwen3.5-4B-GGUF)、
[llama.cpp](https://github.com/ggml-org/llama.cpp)。

## 本机验证（2026-10-09）

RTX 5060 Ti 16 GB、CUDA 13.3；保留 Isaac Sim、YOLOE、GraspNet 和语义建图运行。
固定 llama.cpp 提交 `5e1d74043eb84f5f82471a1c5d4c8171a4c0fcba`，模型 revision
`e87f176479d0855a907a41277aca2f8ee7a09523`。

- 两份模型 SHA256 校验、CUDA 编译及 GPU 加载通过。
- 文本、标准工具调用、工具结果回传、相机图片和 Agent 模拟工具循环共五项通过，整组耗时约 8 秒。
- 模型进程显存采样约 3704–3828 MiB，包含视觉推理后的采样；这不是压力测试峰值。
- 现有 ROS Agent 切换本地后，WebSocket 对话和实时相机场景识别通过，识别为客厅，描述出电视、沙发和人物。
- 五项离线配置测试通过。本地客户端忽略桌面代理，避免 SOCKS 配置干扰回环请求。

报告在 `.cache/qwen/validation.json`、`.cache/qwen/websocket_validation.json`；
本次独立部署日志在 `.cache/qwen/server.log`、`.cache/qwen/agent.log`。
测试未发送真实导航或抓取动作，尚不能据此认定复杂多步骤任务的成功率。

## 厨房取物优化（2026-10-12）

固定厨房布局下，最终配置三次独立冷启动取可乐均完成抓取、抬升、持物回 Home。
原因分析、失败诊断、配置和回放索引见 [厨房取物报告](KITCHEN_FETCH_REPORT.md)。
