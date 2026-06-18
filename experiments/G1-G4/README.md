# G1-G4 抓取与放置实验

G 组考察移动操作能力，覆盖从单次抓取到桌面放置、两物体之间放置的典型操作。该组会使用 Gazebo、导航、机械臂服务、`robot_api` 行动原语和 JudgeLLM/VLM 判定。

## 任务定义

| 任务 | 内容 | 判定重点 |
|------|------|----------|
| G1 | 红色方块已在机械臂可达范围内，直接抓取 | 方块是否被夹起，z 高度是否超过阈值 |
| G2 | 导航到地面红色方块并抓取 | 导航接近、抓取服务和夹爪动作是否连续成功 |
| G3 | 抓取红色方块并放到 CoffeeTable | 放置点是否落在桌面范围内，z 误差是否满足阈值 |
| G4 | 抓取红色方块并放到蓝色、黄色方块之间 | 是否落在两物体连线之间，横向误差是否满足阈值 |

## 脚本说明

| 脚本 | 用途 |
|------|------|
| `manip_capability_test.py` | 小脑手动能力测试，重置机器人和方块后直接执行 G1-G4 |
| `test_e2_vlm_manip.py` | G 组 E2/VLM 闭环测试，记录 Judge、VLM、重规划和 Gazebo 真值 |
| `run_e1_manip.py` | G 组 E1 主实验，默认 `adjust_policy=2` |
| `run_e3_manip.py` | G 组 E3 静态规划，只请求模型生成代码 |
| `run_e4_manip.py` | G 组 E4 人工 API 基线 |
| `run_e1e3_manip_pipeline.py` | 顺序运行 E1/E2/E3，避免 Gazebo 状态相互污染 |
| `build_g_appendix_summary.py`、`create_g_data_pages.py` | 根据本地结果生成附录和数据页 |

## 实验版本

| 版本 | 运行内容 | 重复次数默认值 | 输出 |
|------|----------|----------------|------|
| E1 | LLM 规划 + JudgeLLM/VLM + Gazebo 执行，`adjust_policy=2` | 5 | `G1-G4/outputs/E1_manip_results.csv` |
| E2 | LLM 规划 + JudgeLLM/VLM + Gazebo 执行，主要用于闭环消融 | 3 | `G1-G4/outputs/E2_vlm_smoke_results.csv` |
| E3 | 只生成代码，不执行机器人 | 3 | `G1-G4/outputs/E3_manip_static.csv`、`.json` |
| E4 | 人工 API 序列执行，不调用 LLM/VLM | 3 | `G1-G4/E4_G_original.csv`、`E4_G_appendix.csv` |

G 组 E1/E2 会读取 judge_camera 图像，因此需要保证 `big_brain/vlm_image_writer.py` 正在写入新鲜图片。E3 不需要 Gazebo，可用于快速比较模型、温度和 RAG 设置。

## 版本开关

| 开关 | 说明 |
|------|------|
| `--start-writer` | 自动启动 VLM 图像写入进程 |
| `--allow-stale-image` | 允许使用缓存的 judge_camera 图像，调试时可用 |
| `--no-rag` | E3 关闭历史任务检索，只看基础提示词能力 |
| `--models flash pro` | E3 比较不同模型 |
| `--temperatures 0.0 0.4 0.8` | E3 比较不同采样温度 |

## 常用命令

先启动仿真、定位、导航、场景抽取和机械臂服务：

```bash
./scripts/run_gazebo.sh
./scripts/run_rtab.sh
./scripts/run_teb_compare.sh
python3 scripts/get_scene.py --deploy
python3 small_brain/scripts/arm_task_server.py
```

手动能力测试：

```bash
python3 experiments/G1-G4/manip_capability_test.py --all
```

E3 静态规划：

```bash
python3 experiments/G1-G4/run_e3_manip.py --tasks G1 G2 G3 G4 --models flash --temperatures 0.0 --repeats 3
```

E1 主实验：

```bash
python3 experiments/G1-G4/run_e1_manip.py --tasks G1 G2 G3 G4 --repeats 5 --start-writer
```

E2 闭环测试：

```bash
python3 experiments/G1-G4/test_e2_vlm_manip.py --tasks G1 G2 G3 G4 --repeats 3 --start-writer
```

E4 人工基线：

```bash
python3 experiments/G1-G4/run_e4_manip.py --all --repeats 3
```
