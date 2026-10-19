# 具身 Agent

前端使用原生HTML/CSS/JavaScript，没有使用React。Agent使用原生tool_calls循环：模型选择工具、执行工具、读取结果后继续，最多8轮；属于ReAct风格，没有引入专用ReAct框架。明确的“到厨房拿可乐”指令走固定导航→抓取流程，不调用LLM决策。

保留文件：`agent_server.py`（ROS工具、Agent循环和HTTP/WebSocket接口）、`fetch_policy.py`（明确取物意图与稳定抓取候选）、`model_config.py`（本地／云端配置）、`setup_local_model.py`（模型安装）、`start_local_model.sh`（服务启动）、`wait_local_model.py`（就绪检查）。

## 本地模型验证结果

2026-10-09，RTX5060Ti 16GB：Qwen3.5-4B Q4_K_M、F16视觉投影、llama.cpp CUDA。文本、标准工具调用、工具结果回传、相机图片、模拟Agent工具循环五项通过，整组约8秒；模型进程显存采样3704–3828MiB。WebSocket对话及实时相机场景识别通过。这些检查不代表导航或抓取成功率。

## 历史厨房复测

2026-10-12，Isaac固定厨房同一可乐实例，传统YOLOE／GraspNet／MoveIt最终配置三次独立冷启动均完成抓取、抬升、持物回Home。在线接触判断之外还人工核验了RGB回放，不能外推为普遍100%成功率。

| 轮次 | 停车误差 | 导航墙钟耗时 | 总墙钟耗时 | 视频 |
|---|---|---|---|---|
| 8 | 0.073m／3.8° | 42.11s | 69.20s | [回放](../../../log/embodied_optimization/trial_8.mp4) |
| 9 | 0.075m／3.7° | 44.03s | 71.65s | [回放](../../../log/embodied_optimization/trial_9.mp4) |
| 10 | 0.072m／3.4° | 42.18s | 75.87s | [回放](../../../log/embodied_optimization/trial_10.mp4) |

[历史验证证据](../../../log/embodied_optimization/final_validation.json)。该结果使用旧地图配置，与Gazebo实验分开统计。

## Gazebo结果

传统链路全部采集尝试60/110成功（54.5%），到达厨房后60/94（63.8%）；延长位姿观察窗口并增加启动等待后的35次尝试为26/35（74.3%）。与ACT使用不同种子，且存在配置调整，不是配对消融。

厨房数据重训后ACT：RGB完整成功2/10、RGB＋深度4/10、RGB＋XYZ2/10。以上闭环结果在局部／全局OctoMap拆分之前获得。[ACT结果、loss曲线与逐回合视频](../act_policy/README.md)。
