# RTAB-Map 建图与定位

> 逐一核对自 `small_brain/launch/include/rtabmap_mapping.launch`（建图）、
> `rtabmap_nav.launch`（定位）。本文档汇总当前建图与定位 launch 的关键参数。

**纯 RGB-D 视觉 SLAM，无激光雷达**。定位用 Gazebo 真值里程计（零漂移），RTAB-Map 只负责建二维占用栅格与三维点云。

---

## 1. 定位架构（关键设计）

**Gazebo ground truth 负责定位，RTAB-Map 纯建图，不发 TF。**

| 元件 | 职责 |
|------|------|
| `libgazebo_ros_p3d` | 发 `/ground_truth/odom`（frame=`world`，零漂移，50Hz） |
| `static_transform_publisher` | `map→world` identity + `world→odom` identity（在 `mec_six_arm_house.launch`） |
| `libgazebo_ros_planar_move` | 发 `odom→base_link` TF（仿真真值位姿） |
| RTAB-Map（`publish_tf=false`） | 只提供 `/rtabmap/grid_map`，不影响 TF 树 |

**TF 链**：`map → world → odom → base_link`（全 identity 或真值，RViz 与 Gazebo 完全一致）。

> 重建地图时先执行 `./run_rtab.sh map`，使数据库按当前 `odom_frame_id=world` 重新生成。

---

## 2. 公共订阅（建图/定位一致）

| 项 | 值 |
|----|----|
| frame_id | `base_link` |
| odom_frame_id | `world` |
| 订阅模式 | `subscribe_depth=true`（RGB-D 直接订阅，不用 rgbd_sync，避免 stereo model 兼容问题） |
| subscribe_rgb / rgbd / scan | 全 false |
| remap rgb/image | `/camera/rgb/image_raw` |
| remap depth/image | `/camera/depth/image_raw` |
| remap rgb/camera_info | `/camera/rgb/camera_info` |
| remap odom | `/ground_truth/odom` |
| database_path | `~/.ros/rtabmap.db` |

---

## 3. 建图模式（`rtabmap_mapping.launch`，`./run_rtab.sh map`）

- `args="--delete_db_on_start"`（每次重建）；`output=screen`，附带 `rtabmap_viz`（默认开）。
- `Mem/IncrementalMemory=true`，`Rtabmap/DetectionRate=2`（关键帧 2Hz）。

### 视觉配准（无 ICP/雷达）

| 参数 | 值 |
|------|----|
| Reg/Strategy | **0**（Visual 纯视觉） |
| Reg/Force3DoF | true（平面三自由度） |
| Vis/MinInliers | 25 |
| Vis/MaxDepth | 8.0 |
| RGBD/LinearUpdate / AngularUpdate | 0.05 / 0.05（移动才加节点） |

### 二维占用栅格（从深度图）

| 参数 | 值 | 备注 |
|------|----|------|
| Grid/Sensor | 1 | 从深度生成 |
| **Grid/CellSize** | **0.04** | 4cm/格 |
| Grid/DepthDecimation | 2 | |
| Grid/MaxObstacleHeight | 2.0 | |
| Grid/MinGroundHeight / MaxGroundHeight | −0.2 / 0.05 | 地面高度窗 |
| Grid/RangeMin / RangeMax | 0.3 / 8.0 | 对齐相机量程 |
| Grid/RayTracing | true | |
| **Grid/NoiseFilteringMinNeighbors** | **3** | |
| Grid/NoiseFilteringRadius | 0.04 | |

### 回环检测：完全禁用

> 真值里程计零漂移 → 回环只会在相似房间间造成误合并。

| 参数 | 值 |
|------|----|
| **Kp/MaxFeatures** | **−1**（关特征提取 → 无回环检测） |
| RGBD/ProximityBySpace | false |
| RGBD/NeighborLinkRefining | false |
| VhEp/Enabled | false |

---

## 4. 定位模式（`rtabmap_nav.launch`，`./run_rtab.sh`）

- `localization=true` 时 `args=""`（不删库）；`output=log`（不刷终端）；`rtabmapviz` 默认关。
- **`publish_tf=false`**（不发 map→world，由 static identity 处理）。

| 参数 | 值 | 与建图差异 |
|------|----|-----------|
| Vis/MinInliers | 15 | 建图为 25 |
| Grid/FromDepth | true | |
| Grid/CellSize | 0.04 | |
| Grid/MaxObstacleHeight | 1.5 | 建图为 2.0 |
| GridGlobal/MinSize | 20 | 全局栅格最小尺寸 |
| Mem/IncrementalMemory | false（定位）/ true（非定位） | |
| Mem/InitWMWithAllNodes | true | 载入全部节点做全局重定位 |
| Rtabmap/StartNewMapOnLoopClosure | false | |

---

## 5. 输入 / 输出 topic

**输入**：`/camera/rgb/image_raw`、`/camera/rgb/camera_info`、`/camera/depth/image_raw`、`/ground_truth/odom`。

**输出**：

| Topic | 类型 | 说明 |
|-------|------|------|
| `/rtabmap/grid_map` | `nav_msgs/OccupancyGrid` | 2D 占用栅格（导航/costmap 用，**不是 `/map`**） |
| `/rtabmap/cloud_map` | `sensor_msgs/PointCloud2` | 三维彩色点云 |
| `/rtabmap/localization_pose` | `PoseWithCovarianceStamped` | 定位位姿（定位模式） |
| `/rtabmap/mapData` | `rtabmap_msgs/MapData` | 图数据（节点+连接） |

---

## 6. 对原真机配置的改动

| 项 | 原真机配置 | 本 Sim |
|----|-----------|--------|
| frame_id | `base_footprint` | `base_link` |
| subscribe_scan | true（有雷达） | false |
| Reg/Strategy | 1（ICP） | 0（Visual） |
| Grid 来源 | 雷达 | 深度图 |
| odom | 轮速 | `/ground_truth/odom` 真值 |

---

## 7. 建图能力实测（建图评估）

- 从零建图收敛：约 **89.8 s**（最终约 7.8 万已知栅格）。
- 已有地图 bring-up：约 **3.2 s** 进入可导航。
- ATE-RMSE **0.146 m**（均值 0.110，最大 0.373，393 配对点）。
- N1–N4 终点误差均值约 **0.11 m**（51 样本，最大 0.147）。
