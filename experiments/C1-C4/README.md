# C1-C4 复合任务实验

C 组用于检验大脑在长程复合任务中的规划、执行、自验证和恢复能力。目录中保留 C1-C4 脚本，方便复现实验和局部能力检查；主复合评估聚焦 C3/C4 no-helper，因为 C1/C2 的桌面放置、两物体之间放置能力已由 G3/G4 覆盖。

## 任务定义

| 任务 | 内容 | 判定重点 |
|------|------|----------|
| C1 | 从 BalconyTable 抓取 Coke 罐，放到 NightStand | 局部能力保留脚本：桌面抓取、跨区域搬运、目标桌面放置 |
| C2 | 从 NightStand_01_002 抓取红色方块，放到蓝色和黄色方块之间 | 局部能力保留脚本：桌面抓取、两物体之间放置 |
| C3 | 将地面红、蓝、黄三个方块依次放入垃圾桶 | 主复合任务：多物体循环、重复抓取与放置、失败后继续处理下一物体 |
| C4 | 围绕沙发巡逻，发现红色方块后抓取放入垃圾桶，再回到中断位置完成巡逻 | 主复合任务：巡逻中断、恢复路线、长程状态管理 |

## 脚本说明

| 脚本 | 用途 |
|------|------|
| `composite_capability_test.py` | C 组人工任务执行器，提供复合任务 reset、判定和手动流程 |
| `run_e1_composite.py` | C 组 E1 主实验，默认 `adjust_policy=2` |
| `run_e2_composite.py` | C 组 E2 闭环消融，对比 `adjust_policy=0/1` |
| `run_e3_composite.py` | C 组 E3 静态规划，比较模型、温度、RAG 和 helper 设置 |
| `run_e4_composite.py` | C 组 E4 人工 API 基线 |
| `run_e1e2e3_composite_pipeline.py` | 按顺序运行 E1/E2/E3，避免 Gazebo、机械臂、图像和日志状态污染 |
| `run_c3c4_batch_no_helper.sh` | C3/C4 no-helper 批量实验入口 |
| `run_c3_red_only.py`、`red_to_coffeetable.py`、`inplace_pickup_test.py` | 单项调试与局部能力验证脚本；演示入口见 `scripts/demo_read_to_trash.py` |

## 实验版本

| 版本 | 运行内容 | 重复次数默认值 | 输出 |
|------|----------|----------------|------|
| E1 | LLM 规划 + JudgeLLM/VLM + Gazebo 执行，`adjust_policy=2` | 5 | `C1-C4/outputs/C-E1.csv`、`C-E1-appendix.csv` |
| E2 | LLM 规划 + JudgeLLM/VLM + Gazebo 执行，`adjust_policy=0/1` | 每种 3 | `C1-C4/outputs/C-E2.csv`、`C-E2-appendix.csv` |
| E3 | 只生成代码，不执行机器人 | 3 | `C1-C4/outputs/C-E3.csv`、`C-E3-appendix.csv` |
| E4 | 人工 API 序列执行，不调用 LLM/VLM | 3 | `C1-C4/outputs/C-E4.csv`、`C-E4-appendix.csv` |

## helper 与 no-helper 区别

| 模式 | 含义 | 适合回答的问题 |
|------|------|----------------|
| helper | prompt 中保留 C1-C4 的专用任务 helper 或示例，模型更容易调用稳定封装 | 系统在工程优化后的完整表现如何 |
| no-helper | 使用 `--no-helper-prompt` 移除 C1-C4 专用 helper 提示，模型必须从 `move/pick/put` 等基础原语组合任务 | 大模型是否真的能完成长程分解与组合 |

在结果说明中，helper 版本更接近工程部署形态，no-helper 版本更能体现大模型规划、RAG 和自验证闭环的贡献。C3/C4 是主复合评估任务，因为它们需要循环、状态保持和中断恢复。

## RAG 与模型版本

| 设置 | 含义 |
|------|------|
| RAG on | 从历史成功任务中检索相似示例，作为参考片段注入 prompt |
| RAG off | 不注入历史成功任务，只依赖基础 prompt |
| flash | 速度和成本优先，适合大批量重复实验 |
| pro | 规划能力优先，适合 E3 静态规划对比 |
| temperature 0.0 | 输出更稳定 |
| temperature 0.4/0.8 | 输出更多样，可观察复杂任务下的稳定性变化 |

## 常用命令

先启动仿真、定位、导航、场景抽取和机械臂服务：

```bash
./scripts/run_gazebo.sh
./scripts/run_rtab.sh
./scripts/run_teb_compare.sh
python3 scripts/get_scene.py --deploy
python3 small_brain/scripts/arm_task_server.py
```

E1/E2/E3 顺序执行：

```bash
python3 experiments/C1-C4/run_e1e2e3_composite_pipeline.py \
  --tasks C1 C2 C3 C4 \
  --order e1 e2 e3 \
  --e1-repeats 5 \
  --e2-repeats 3 \
  --e3-repeats 3 \
  --e3-models flash pro \
  --e3-temperatures 0.0 0.4 0.8 \
  --start-writer \
  --continue-on-error
```

no-helper 版本：

```bash
python3 experiments/C1-C4/run_e1e2e3_composite_pipeline.py \
  --tasks C3 C4 \
  --order e1 e2 e3 \
  --no-helper-prompt \
  --start-writer \
  --continue-on-error
```

E4 人工基线：

```bash
python3 experiments/C1-C4/run_e4_composite.py --all --repeats 3
```

C1 red_block 变体：

```bash
python3 experiments/C1-C4/run_e1_composite.py --tasks C1 --c1-block-variant --start-writer
```
