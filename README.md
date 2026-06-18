# 大小脑协同的低成本室内服务机器人系统

> 项目代码包
> 麦克纳姆轮底盘 + 六自由度机械臂，云端 LLM/VLM「大脑」+ 本地 ROS「小脑」分层架构。

面向家庭/办公室室内场景的服务型移动操作机器人。硬件刻意限定为**单个 RGB-D 相机 + 普通消费级笔记本 + 开源 ROS/Gazebo 栈**（不依赖激光雷达与本地大算力 GPU），在此约束下完成「环境建图 → 语义对象抽取 → 自然语言任务执行」的完整闭环。

---

## 目录结构

```
code/
├── README.md            本文件：项目总览与启动
├── ENVIRONMENT.md       环境依赖、安装与 API 配置（先读这个）
│
├── big_brain/           【大脑】云端 LLM/VLM 规划层（纯 Python，非 ROS 包）
│   ├── big_brain.py         主入口：RAG → Planner(DeepSeek) → exec → Judge
│   ├── ros_bridge.py        ROS 桥接：导航/转向/对象查询
│   ├── action/robot_api.py  底层行动原语（move/pick/put）
│   ├── model/               PlannerLLM / JudgeLLM / RAGManager
│   ├── prompt/              多层级提示词模板（BASE_PROMPT）
│   ├── config.py            ★ API key 已抹除，运行前需填（见 ENVIRONMENT.md）
│   └── ...
│
├── small_brain/         【小脑】ROS 包 small_brain_sim（导航 + 抓取执行）
│   ├── launch/              Gazebo / RTAB-Map / TEB 启动文件
│   ├── config/nav/          TEB + move_base + costmap 参数
│   ├── scripts/             arm_task_server.py（机械臂服务）等
│   ├── src/                 block_follower_plugin.cpp（C++ 搬运插件）
│   ├── urdf/                仿真机器人 URDF（含相机/夹爪）
│   └── models/              red/yellow/blue_block 等场景模型
│
├── arm/                 机械臂描述与 MoveIt 配置（ROS 包）
│   ├── mini_mec_six_arm/                URDF + meshes
│   ├── mini_mec_six_arm_moveit_config/  MoveIt（OMPL/KDL/SRDF）
│   └── wheeltec_arm_pick / _rc / tracker_pkg
│
├── scene/               Gazebo 场景资产
│   ├── aws-robomaker-small-house-world/ 小屋场景（AWS RoboMaker，开源）
│   └── coke/                            水杯模型（C1 任务用）
│
├── experiments/         实验脚本（E1–E4 评估）
│   ├── N1-N4/               导航能力评估脚本
│   ├── G1-G4/               抓取/放置能力评估脚本
│   └── C1-C4/               复合任务评估脚本
│
├── scripts/             根目录任务/工具脚本（小脑独立运行，不走 LLM）
│   ├── get_scene.py            从 Gazebo 抽取 L2 语义-几何资料库 → gazebo_scene.yaml
│   ├── pick_deliver_teb.py     完整 pick-deliver 任务（TEB 主流程）
│   ├── put_on_table_test.py    桌面抓取姿态调校
│   ├── run_gazebo.sh / run_rtab.sh / run_teb_compare.sh  启动脚本
│   └── ...
│
└── 项目文档/             当前项目说明
    ├── 00_总览.md                    系统架构与启动顺序
    ├── 01_车辆配置.md                导航、运动规划与控制器参数
    ├── 02_RTAB-Map.md                建图与定位配置
    ├── 03_车辆本体.md                URDF、关节、相机与 Gazebo 插件
    ├── 04_大脑.md                    LLM/VLM 规划、自验证与 RAG
    └── 05_其他.md                    抓取系统、场景抽取、脚本与实验
```

---

## 系统架构

```
用户自然语言指令
        │
        ▼
┌─────────────────────────────────────┐
│  大脑 big_brain/  （低频语义推理）   │
│  RAG 检索 → Planner(DeepSeek) 生成   │
│  Python 行动脚本 → exec → JudgeLLM   │
│  (规则 + Gemini VLM) 判定 → 失败重规划│
└──────────────────┬──────────────────┘
                   │ robot_api 原语
                   ▼
┌─────────────────────────────────────┐
│  小脑 small_brain/ （本地实时控制）  │
│  RTAB-Map 视觉 SLAM（RGB-D → 2D 栅格）│
│  TEB + move_base 导航                │
│  MoveIt + arm_task_server 机械臂抓取 │
│  block_follower_plugin 零延迟搬运    │
└─────────────────────────────────────┘
```

详细分层、参数与数据流见 [项目文档/00_总览.md](项目文档/00_总览.md)。

---

## 快速启动（TEB 主流程）

> 完整环境配置见 [ENVIRONMENT.md](ENVIRONMENT.md)。需先把本包的 ROS 包链接进 catkin 工作区并 `catkin_make`。

最小安装流程：

```bash
sudo apt install ros-noetic-desktop-full python3-pip python3-rosdep
./setup_workspace.sh ~/catkin_ws
source ~/catkin_ws/devel/setup.bash
export PROJECT_ROOT="$(pwd)"
export GAZEBO_MODEL_PATH="$PROJECT_ROOT/scene:$PROJECT_ROOT/small_brain/models:$GAZEBO_MODEL_PATH"
```

脚本用途见 [scripts/README.md](scripts/README.md)。系统启动后可运行 `./smoke_test.sh` 检查关键 topic、service 和场景文件是否就绪。

```bash
# 终端 1：Gazebo 仿真（含小屋场景 + 机械臂初始化）
./scripts/run_gazebo.sh
# 终端 2：RTAB-Map（首次建图 ./run_rtab.sh map，之后定位 ./run_rtab.sh）
./scripts/run_rtab.sh
# 终端 3：TEB + move_base
./scripts/run_teb_compare.sh
# 终端 4：抽取场景对象坐标（一次性）
python3 scripts/get_scene.py
# 终端 5：机械臂服务
python3 small_brain/scripts/arm_task_server.py

# —— 小脑独立任务（不走 LLM）——
python3 scripts/pick_deliver_teb.py

# —— 大脑（自然语言任务）——
python3 big_brain/vlm_image_writer.py &   # VLM 图像桥接
python3 big_brain/big_brain.py            # 输入指令
```

各终端依赖顺序与参数说明见 [项目文档/00_总览.md](项目文档/00_总览.md)。

---

## 整理说明

本项目按代码留档整理：不包含真实 API key，不包含本地机器上的运行日志或缓存文件。首次运行前请设置：

```bash
export PROJECT_ROOT="$(pwd)"
export GAZEBO_MODEL_PATH="$PROJECT_ROOT/scene:$PROJECT_ROOT/small_brain/models:$GAZEBO_MODEL_PATH"
```

云端模型 key 请在 `big_brain/config.py` 中填入，或按 [ENVIRONMENT.md](ENVIRONMENT.md) 改成从环境变量读取。

## 实验复现（E1–E4）

`experiments/` 下 N/G/C 三组保留复现实验所需脚本，对应实验评估。运行后生成的 `outputs/`、JSON、jsonl、截图或临时文件不纳入代码目录。
评估口径：strict / Gazebo / VLM / semantic。

---

## 安全提示

本包内所有 API key（DeepSeek / Gemini / 智谱 / SiliconFlow / GPT）**已抹除为占位符**。
运行前请在 `big_brain/config.py` 填入自己的 key，或改用环境变量（见 ENVIRONMENT.md）。
