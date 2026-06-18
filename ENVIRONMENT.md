# 环境依赖与配置

## 0. 前置要求总览（先装这些）

按下面三类装齐，再做后面的工作区编译。

| 类别 | 前置要求 | 安装方式 |
|------|---------|---------|
| **ROS / 系统** | ROS **Noetic**（建议 `desktop-full`，含 Gazebo 11、RViz）；Ubuntu 20.04 | `sudo apt install ros-noetic-desktop-full`；并 `source /opt/ros/noetic/setup.bash` |
| **C++ 工具链** | `build-essential`（g++ 9）、`cmake`、`libgazebo11-dev`（编译 `block_follower_plugin` Gazebo 插件必需）、`rosdep` | 见下方 §2 apt 清单 |
| **Python** | Python **3.8+**、`python3-pip`；ROS 端 `rospy`（随 ROS 装好） | `sudo apt install python3-pip`；大脑依赖见 §4 |

> **C++ 说明**：本项目只有一个自写 C++ 组件 —— 小脑的 `block_follower_plugin.cpp`（Gazebo World Plugin，1000Hz 搬运）。它通过 catkin 用 `roscpp + gazebo` 编译，**不需要单独的构建系统**，但必须先装 `libgazebo11-dev`（提供 `find_package(gazebo)` 的头文件与库），否则 `catkin_make` 会报找不到 gazebo。
>
> **Python 说明**：分两套且互不依赖 —— ① 小脑 ROS 节点（`arm_task_server.py`、`get_scene.py` 等）用系统 `python3` + ROS 自带的 `rospy/rospy`，无额外 pip 包（`get_scene.py` 另需 `trimesh`，见 §4）；② 大脑 `big_brain/` 是纯 Python，依赖见 §4 的 `requirements.txt`。

## 1. 系统平台

| 项 | 版本 / 说明 |
|----|------------|
| OS | Ubuntu 20.04；WSL2 或原生 Ubuntu 均可 |
| ROS | Noetic |
| Gazebo | 11 |
| RViz | 1.14.26 |
| Python | 3.8（ROS 小脑节点）；大脑 `big_brain/` 用 3.8+ 均可 |
| GPU | 非必需；云端模型负责推理，本地 GPU 只影响 Gazebo/RViz 显示 |
| Mesa | WSL2 下若 RViz STL mesh 不显示，可升级 Mesa 或检查 OpenGL 渲染环境 |

> WSL2 下 RViz/MoveIt 如遇 mesh 不显示，可设置
> `LD_LIBRARY_PATH=/usr/local/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH`，
> 并优先确认 OpenGL 驱动、Mesa 与 WSLg 图形环境正常。

## 2. ROS 依赖（apt）

```bash
sudo apt install \
  build-essential cmake python3-pip python3-rosdep \
  libgazebo11-dev \
  ros-noetic-rtabmap-ros \
  ros-noetic-move-base ros-noetic-teb-local-planner ros-noetic-global-planner \
  ros-noetic-costmap-2d ros-noetic-navigation \
  ros-noetic-moveit ros-noetic-moveit-resources \
  ros-noetic-depthimage-to-laserscan \
  ros-noetic-gazebo-ros-pkgs ros-noetic-gazebo-ros-control \
  ros-noetic-ros-control ros-noetic-ros-controllers \
  ros-noetic-effort-controllers ros-noetic-joint-trajectory-controller \
  ros-noetic-robot-state-publisher ros-noetic-xacro
```

> 前 3 行是 **C++ 工具链 + Python pip + Gazebo dev**（编译 `block_follower_plugin` 与 catkin 必需）；
> 其余为本项目用到的 ROS 功能包（RTAB-Map / 导航 / MoveIt / 控制器）。
> 也可在工作区用 `rosdep install --from-paths src --ignore-src -r -y` 自动补齐 package.xml 声明的依赖。

## 3. 工作区搭建（catkin）

本包的 ROS 包（`small_brain/`、`arm/*`、`scene/aws-robomaker-small-house-world/`）需放进 catkin 工作区编译。
注意 `small_brain/` 的包名为 **`small_brain_sim`**。

> **一键搭建（推荐）**：`./setup_workspace.sh [catkin_ws_目录]` 自动完成下面的软链 + `catkin_make`，
> 并打印需要的 `GAZEBO_MODEL_PATH` / `PROJECT_ROOT`。搭好后用 `./smoke_test.sh` 自检关键 topic/service。
> 大脑读 `gazebo_scene.yaml` 的路径由 `PROJECT_ROOT` 环境变量决定（默认 = 本包目录，已内置一份样本）。

手动方式：

```bash
mkdir -p ~/catkin_ws/src && cd ~/catkin_ws/src
# 软链或拷贝本包内的 ROS 包
ln -s /path/to/code/small_brain            small_brain_sim
ln -s /path/to/code/arm/mini_mec_six_arm   .
ln -s /path/to/code/arm/mini_mec_six_arm_moveit_config .
ln -s /path/to/code/scene/aws-robomaker-small-house-world .
# coke 模型放进 Gazebo 模型路径
export GAZEBO_MODEL_PATH=/path/to/code/scene:$GAZEBO_MODEL_PATH

cd ~/catkin_ws && catkin_make
# C++ 搬运插件（block_follower_plugin）
catkin_make --pkg small_brain_sim
source devel/setup.bash
```

> `arm/` 内另含 `wheeltec_arm_pick`、`wheeltec_arm_rc`、`wheeltec_tracker_pkg`，按需 link。
> 抓取若用到 link_attacher，需另装 `gazebo_ros_link_attacher`（已改用 SetWorldPose 插件，通常不必）。

## 4. 大脑 Python 依赖

```bash
cd big_brain
pip install -r requirements.txt        # numpy, sentence-transformers, openai
```

`sentence-transformers` 用于 RAG 句向量编码（首次运行会下载 ~400MB 模型）。
所有云端模型通过 OpenAI 兼容端点调用，故只需 `openai` 一个客户端库。

小脑侧脚本 `scripts/get_scene.py` 额外需要 `trimesh`（读 SDF collision mesh 算 bbox / surface_z）：

```bash
pip install trimesh
```

## 5. ★ API Key 配置（运行前必做）

`big_brain/config.py` 中的 key **已抹除为占位符**，需自行填写。涉及的服务：

| 角色 | 默认模型 | 端点 |
|------|---------|------|
| Planner / Judge LLM | DeepSeek `deepseek-chat` | `https://api.deepseek.com/v1` |
| VLM judge | Gemini `gemini-2.5-flash` / `gemini-flash-lite` | Google OpenAI 兼容端点 |
| 备用 | 智谱 GLM、SiliconFlow、GPT、Ollama 本地 | 见 config.py |

**方式 A**：直接编辑 `config.py`，把 `"<YOUR_*_API_KEY>"` 换成真实 key。
**方式 B（推荐）**：改为读环境变量，例如

```python
import os
DEEPSEEK_API_KEY = os.environ.get("DEEPSEEK_API_KEY", "")
GEMINI_API_KEY   = os.environ.get("GEMINI_API_KEY", "")
```

然后 `export DEEPSEEK_API_KEY=...` 后再运行，避免明文 key 进版本库。

## 6. 启动顺序

见 [README.md](README.md)「快速启动」、[scripts/README.md](scripts/README.md)
以及 [项目文档/00_总览.md](项目文档/00_总览.md)。这些文档包含各终端依赖顺序、
RTAB-Map 建图/定位切换、TEB 调参、机械臂姿态和实验脚本入口。
