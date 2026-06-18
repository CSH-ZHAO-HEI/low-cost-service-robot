# scripts 脚本说明

本目录保存系统启动、建图导航、场景抽取、抓取调试和可视化辅助脚本。脚本默认在 `code/` 根目录或 `code/scripts/` 目录下运行；运行前请先完成 `ENVIRONMENT.md` 中的工作区配置，并在终端执行：

```bash
source ~/catkin_ws/devel/setup.bash
```

如需使用 Gazebo 模型，请在 `code/` 根目录设置：

```bash
export PROJECT_ROOT="$(pwd)"
export GAZEBO_MODEL_PATH="$PROJECT_ROOT/scene:$PROJECT_ROOT/small_brain/models:$GAZEBO_MODEL_PATH"
```

## 1. 推荐启动顺序

| 顺序 | 脚本 | 作用 |
|---|---|---|
| 1 | `run_gazebo.sh` | 启动 Gazebo 小屋场景、机器人本体和机械臂初始化流程。 |
| 2 | `run_rtab.sh` | 启动 RTAB-Map 建图或定位。首次建图可使用 map 模式，已有地图可使用定位模式。 |
| 3 | `run_teb_compare.sh` | 启动 move_base + TEB 局部规划器，用于导航执行。 |
| 4 | `get_scene.py --deploy` | 从 Gazebo 抽取对象位姿，生成语义场景文件并部署到运行路径。 |
| 5 | `small_brain/scripts/arm_task_server.py` | 机械臂服务在 `small_brain/` 内，不在本目录；启动后抓取脚本才能调用机械臂服务。 |

常用组合：

```bash
./scripts/run_gazebo.sh
./scripts/run_rtab.sh
./scripts/run_teb_compare.sh
python3 scripts/get_scene.py --deploy
python3 small_brain/scripts/arm_task_server.py
```

## 2. 启动与可视化脚本

| 脚本 | 说明 |
|---|---|
| `run_gazebo.sh` | 启动 Gazebo 仿真场景，清理旧 Gazebo/ROS 进程，并可启动图像窗口。 |
| `run_rtab.sh` | 启动 RTAB-Map 建图或定位流程。 |
| `run_teb_compare.sh` | 启动 TEB 导航配置。 |
| `show_robot.sh` | 只显示机器人模型，适合检查 URDF、关节和 TF。 |
| `map_rviz.sh` | 打开地图相关 RViz 配置。 |
| `moveit_rviz.sh` | 打开 MoveIt RViz 配置，检查机械臂规划。 |
| `debug_rviz.sh` | 打开调试用 RViz 配置，辅助检查 TF、点云、地图和代价地图。 |
| `chase_cam.sh` | 显示机器人相机画面。 |
| `pip_cam.py` | 订阅图像 topic 并弹出画面窗口，默认可用于相机画面检查。 |
| `start_control.sh` | 启动键盘控制脚本，适合手动移动机器人。 |

## 3. 场景与对象数据脚本

| 脚本 | 说明 |
|---|---|
| `get_scene.py` | 从 Gazebo 读取模型位姿，生成 `gazebo_scene.yaml`；加 `--deploy` 可部署为大脑层使用的语义地图。 |
| `gazebo_scene.yaml` | 场景抽取结果示例文件，记录对象名称、坐标、尺寸和接近点。 |
| `respawn_blocks.py` | 重置或重新生成红、黄、蓝色块，供抓取与放置实验使用。 |
| `set_initial_pose.py` | 设置机器人初始位姿。 |
| `set_balcony_pose.py` | 设置 C1 阳台桌相关目标位姿。 |
| `set_nightstand_pose.py` | 设置床头柜相关目标位姿。 |

## 4. 导航脚本

| 脚本 | 说明 |
|---|---|
| `goto.py` | 根据目标对象或坐标调用导航接口，适合单点导航调试。 |
| `goto.sh` | `goto.py` 的 shell 包装脚本。 |
| `pure_nav.py` | 直接按坐标发送导航目标，不经过语义对象解析。 |
| `pick_deliver_teb.py` | 使用 TEB 的完整抓取搬运流程，是小脑独立任务的主流程之一。 |
| `pick_deliver.py` | 早期抓取搬运流程，保留用于对照和调试。 |

## 5. 机械臂与抓取调试脚本

| 脚本 | 说明 |
|---|---|
| `arm_clamp_init.py` | 初始化夹爪闭合状态，常由 `run_gazebo.sh` 自动调用。 |
| `arm_reset.py` | 将机械臂恢复到预设初始姿态。 |
| `pick_and_place.py` | 基础抓取放置测试脚本。 |
| `test_coke_pick.py` | 可乐罐抓取调试脚本，支持跳过导航、只抓取、只测试姿态等模式。 |
| `put_on_table_test.py` | 桌面放置调试脚本，用于调整接近方向、放置目标和机械臂姿态。 |
| `between_blocks_test.py` | 两物体之间放置测试脚本，对应 G4/C2 类空间关系。 |
| `demo_read_to_trash.py` | 演示视频对应脚本：红色方块放入垃圾桶，展示规划、规则/VLM 判定和失败微调恢复过程。 |

## 6. 使用建议

- 启动类脚本优先在 `code/` 根目录执行，避免相对路径找不到场景或模型。
- 建图、导航、抓取建议分终端启动，便于观察日志。
- 若导航失败，先检查 `/rtabmap/grid_map`、`/ground_truth/odom`、`/move_base/status` 和 RViz 中的 costmap。
- 若抓取失败，先确认 `arm_task_server.py` 已启动，再检查物体是否已由 `get_scene.py --deploy` 写入语义地图。
- 实验脚本集中在 `experiments/`，本目录脚本主要用于系统启动、单项调试和小脑独立流程。
