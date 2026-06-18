# N1-N4 导航实验

N 组只考察底盘导航能力，不调用机械臂，不调用 VLM，也不需要抓取服务。主要用于验证 RTAB-Map 栅格地图、move_base、GlobalPlanner 和 TEB 的组合是否能稳定完成室内导航任务。

## 任务定义

| 任务 | 内容 | 判定重点 |
|------|------|----------|
| N1 | 导航到固定坐标 `(1.0, 2.0)` | 终点位置误差 |
| N2 | 导航到最近的垃圾桶接近点 | 能否从语义地图选择目标并到达 approach 点 |
| N3 | 围绕沙发执行 4m × 4m 方形路径 | 多航点路径跟随能力 |
| N4 | 到最近垃圾桶后返回起点 | 往返导航与起点恢复能力 |

## 脚本说明

| 脚本 | 用途 |
|------|------|
| `nav_capability_test.py` | 小脑手动能力测试，不经过 LLM，适合先确认导航链路是否正常 |
| `run_flash_e1e2e3_nav.py` | 使用 flash 模型收集 N 组 E1/E2/E3 数据 |
| `run_pro_e3_nav.py` | 使用 pro 模型收集 E3 静态规划数据 |
| `run_e4_nav.py` | 人工 API 导航基线，不调用 LLM |

## 实验版本

| 版本 | 运行内容 | 重复次数默认值 | 输出 |
|------|----------|----------------|------|
| E1 | LLM 生成导航代码并执行，`adjust_policy=2` | 5 | `N1-N4/outputs/E1_flash.csv` |
| E2 | LLM 生成导航代码并执行，对比 `adjust_policy=0/1` | 每种 3 | `N1-N4/outputs/E2_flash.csv` |
| E3 | 只生成代码，不启动机器人 | 3 | `N1-N4/outputs/E3_flash.csv`、`E3_pro.csv` |
| E4 | 人工写定 API 序列执行 | 3 | `N1-N4/e4_nav_results.csv` |

N 组没有抓取动作，因此 E1/E2 的判定主要看导航调用是否完成、终点误差是否在阈值内，以及多航点任务是否按顺序完成。

## 常用命令

先启动仿真、建图定位、TEB 和场景抽取：

```bash
./scripts/run_gazebo.sh
./scripts/run_rtab.sh
./scripts/run_teb_compare.sh
python3 scripts/get_scene.py --deploy
```

手动能力测试：

```bash
python3 experiments/N1-N4/nav_capability_test.py all
```

E3 静态规划：

```bash
python3 experiments/N1-N4/run_flash_e1e2e3_nav.py --e3-only
python3 experiments/N1-N4/run_pro_e3_nav.py --repeats 3
```

E1/E2 执行实验：

```bash
python3 experiments/N1-N4/run_flash_e1e2e3_nav.py --execute
```

E4 人工基线：

```bash
python3 experiments/N1-N4/run_e4_nav.py --all --repeats 3
```
