# 安装与构建

[返回项目说明](README.md)


### 1. 公共环境

使用 **Ubuntu 24.04、ROS 2 Jazzy、NVIDIA GPU、CUDA Toolkit 和 TensorRT 开发环境**。本项目使用 TensorRT 10；CUDA 需提供 `nvcc`，TensorRT 需提供头文件、库及 `trtexec`。用系统 Python 3.12 构建 ROS，先退出 Conda。

```bash
sudo apt update
sudo apt install build-essential cmake git curl python3-vcstool python3-rosdep python3-colcon-common-extensions \
  python3-numpy python3-yaml python3-dev python3-venv libeigen3-dev libpcl-dev \
  libglut-dev libgl-dev libglu1-mesa-dev libssl-dev \
  ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-nav2-graceful-controller \
  ros-jazzy-moveit ros-jazzy-moveit-ros-perception \
  ros-jazzy-ros2-control ros-jazzy-ros2-controllers \
  ros-jazzy-cv-bridge ros-jazzy-sensor-msgs-py

# 具身 Web 服务（start_embodied.sh 需要，安装到系统）
sudo apt install ros-jazzy-rosbridge-server ros-jazzy-web-video-server

# rosdep 未初始化时先执行一次：sudo rosdep init
bash scripts/setup/setup_dependencies.sh
bash -c 'source /opt/ros/jazzy/setup.bash && rosdep update && rosdep install --from-paths src --ignore-src -r -y'
```

安装脚本下载固定版本的 `src/localization/FAST_LIO_ROS2` 并初始化 ikd-Tree 子模块。项目自带 Livox 消息包，不需要真实 Livox SDK。

具身 Agent 的 Python 依赖可手动安装到系统目录下的独立环境，避免依赖工作区 `.venv/agent`：

```bash
sudo python3 -m venv --system-site-packages /opt/embodied-agent
sudo /opt/embodied-agent/bin/python -m pip install openai fastapi uvicorn websockets pillow
```

在 `~/.zshrc`（Bash 使用 `~/.bashrc`）加入以下配置，并在当前终端执行一次：

```bash
export AGENT_PYTHON=/opt/embodied-agent/bin/python
```

安装后检查（新终端中执行，避免加载工作区缓存环境）：

```bash
bash -c 'source /opt/ros/jazzy/setup.bash && ros2 pkg prefix rosbridge_server && ros2 pkg prefix web_video_server'
/opt/embodied-agent/bin/python -c 'import openai, fastapi, uvicorn, websockets; print("Agent dependencies OK")'
```

两个 ROS 包应位于 `/opt/ros/jazzy`。以上安装完成后，具身 Web/Agent 可替代
`.cache/embodied_sysroot` 和 `.venv/agent`；`.cache` 还可能包含 VIO 运行库及场景资产，
不能因此整目录删除。可选 VIO 依赖与版本要求见 [定位模块说明](src/localization/README.md)。

### 2. 选择仿真后端

**Isaac Sim：**安装 Isaac Sim 6.1，目录必须包含 `python.sh`。在 `~/.zshrc`（Bash 使用 `~/.bashrc`）配置实际路径：

```bash
export ISAAC_SIM_PATH="$HOME/isaacsim"
export ISAAC_ASSETS_PATH="$HOME/isaacsim_assets/6.1"
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:/usr/src/tensorrt/bin:$PATH"
export LD_LIBRARY_PATH="$CUDA_HOME/lib64${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
```

TensorRT 使用 tar 包安装时，还需加入其 `bin` 和 `lib` 路径。重新打开终端，确认 `nvcc --version`、`trtexec --version` 和 `test -x "$ISAAC_SIM_PATH/python.sh"` 成功。

**Gazebo：**额外安装 Harmonic 桥接、控制及射线查询依赖；只使用 Isaac 时不需要这些包。

```bash
sudo apt install ros-jazzy-ros-gz ros-jazzy-gz-ros2-control \
  ros-jazzy-gz-sim-vendor ros-jazzy-gz-common-vendor ros-jazzy-gz-plugin-vendor libembree4-4 libembree-dev
```

`libembree4-4` 是 Gazebo 后端的运行依赖，`libembree-dev` 用于编译。
若旧插件曾链接 `.cache/embree`，删除该目录前须先安装系统依赖。
出现 `No plugins detected in library` 时可运行以下检查，确认动态库依赖及插件注册：

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
bash scripts/runtime/check_gazebo_plugins.sh
```

两种后端共用 `simple_room`、`manipulation_test`，以及定位、导航、感知、建图和抓取链路。Office、NVIDIA Simple Room 等 USD 场景仅支持 Isaac。

场景工具从固定历史提交 `92b0409ccf83549e74d03966bdde7f0f700ff927` 提取原始资产，克隆时须保留包含该提交的 Git 历史。默认资产首次启动自动准备，也可提前执行：

```bash
# Isaac：转换共同场景为 USD
"$ISAAC_SIM_PATH/python.sh" scripts/assets/migrate_gazebo_scenes.py --worlds simple_room manipulation_test
# Gazebo：准备 SDF 与模型，不依赖 Isaac
python3 scripts/assets/prepare_gazebo_scene.py --world simple_room
# 可选：下载 Isaac Office / NVIDIA Simple Room
"$ISAAC_SIM_PATH/python.sh" scripts/assets/download_isaac_environments.py
```

### 3. 模型与构建

从 [模型目录](https://drive.google.com/drive/folders/1gPPyvKqiYd7cg2vUyqucV1CjLTf6J0y2?usp=drive_link) 下载以下文件，并保留源码中的 `yoloe_infer/models/tokenizer_data.json.gz`：

| 模型 | 放置目录 |
|---|---|
| `yoloe-v8l-text-prompt-multi_nc10_fp16.onnx` | `src/perception/yoloe_infer/models/` |
| `graspnet.onnx` | `src/perception/graspnet_infer/` |

```bash
bash src/perception/graspnet_infer/tensorrt_plugins/build.sh
trtexec --onnx=src/perception/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.onnx \
  --saveEngine=src/perception/yoloe_infer/models/yoloe-v8l-text-prompt-multi_nc10_fp16.engine
trtexec --onnx=src/perception/graspnet_infer/graspnet.onnx \
  --saveEngine=src/perception/graspnet_infer/graspnet.trt \
  --staticPlugins=src/perception/graspnet_infer/tensorrt_plugins/build/libfps_plugin.so
bash -c 'source /opt/ros/jazzy/setup.bash && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release'
```

换 GPU 或 TensorRT 版本后重新生成 engine。交互式 Zsh 加载 `setup.zsh`，Bash 加载 `setup.bash`；启动脚本内部自动加载 Bash 环境。

### 4. 具身闭环依赖

先按“公共环境”安装 Web ROS 包及 Agent Python 依赖。具身入口默认使用本地 Qwen，首次使用先安装模型：

```bash
/opt/embodied-agent/bin/python src/embodied/llm_agent/setup_local_model.py --cuda-arch 120
```

上述架构值 `120` 用于 RTX 5060 Ti，其他 GPU 按实际 CUDA 架构修改；脚本下载模型并编译 llama.cpp，启动入口会拉起本地服务。使用 DashScope 时：

```bash
export AGENT_PYTHON=/opt/embodied-agent/bin/python
export DASHSCOPE_API_KEY='你的 API Key'
./start_embodied.sh --llm cloud
```

Web UI：**http://localhost:8888**。云端模式需要 API Key，本地模式无需云端密钥。
如需无需 sudo 的工作区安装，可改用 `bash scripts/setup/setup_embodied_dependencies.sh --local`，
依赖将放入 `.cache/embodied_sysroot` 和 `.venv/agent`；使用该环境时不要设置上述 `AGENT_PYTHON`。
