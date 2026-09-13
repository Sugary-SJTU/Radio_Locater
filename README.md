# 无线电干扰源自动定位与清除（Linux）

2026 年数学建模竞赛 B 题项目。已实现问题 1、2 的数值模型，问题 3 的两种在线
策略，以及问题 3、4 共用的附件兼容 HTTP 模拟器。问题 4 自动策略尚未实现。

项目统一使用 `pathlib` 构造路径，不依赖盘符、反斜杠或当前启动目录。绘图固定使用
无界面 `Agg` 后端，字体缓存写入 `/tmp/radio_locator_matplotlib_<uid>`，可在普通
终端、SSH和无桌面服务器中运行。

## 项目结构

```text
.
├── main.py / run.py                  # 等价Linux入口，可直接执行
├── environment.yml / pyproject.toml  # mcm环境、依赖和命令注册
├── config/                           # 题面常量、路径、接口和问题3 YAML参数
├── problems/
│   ├── problem1/                     # 交会定位、直径、反例和绘图
│   ├── problem2/                     # 第二检测点选址和概率积分
│   ├── problem3/                     # 两种在线策略、日志、统计和轨迹图
│   └── problem4/                     # 问题4配置预留
├── src/radio_locator/
│   ├── cli.py                        # 统一CLI
│   ├── client.py                     # 四个附件HTTP端点的严格客户端
│   ├── local_simulator.py            # 问题3/4本地模拟器
│   ├── geometry.py                   # 公共几何算法
│   └── runtime.py                    # Linux无界面绘图适配
├── scripts/attachment_protocol_demo.py
├── reference/                        # 题目与附件
├── paper/                            # 当前论文
├── res/                              # 图像、表格、日志和本地真值
└── tests/                            # 回归测试
```

`config/constants.py` 只保存题面常量；问题 3 的算法参数集中在
`config/problem3.yaml`。`res/simulator/` 中的真值只允许在本地仿真结束后作图，
策略执行期间不得读取。

## Linux环境

当前项目环境名为 `mcm`。进入项目根目录后执行：

```bash
cd /home/Lilywhite/PIG/CUMCM/Code
micromamba activate mcm
python -m pip install -e .
```

如果当前 shell 尚未初始化 micromamba，可先运行：

```bash
eval "$(micromamba shell hook --shell bash)"
micromamba activate mcm
```

也可以不激活环境，直接执行：

```bash
micromamba run -n mcm python run.py --help
```

根目录的 `main.py` 和 `run.py` 会自动加入 `src/`，所以从项目根目录直接运行时
不再要求手工设置 `PYTHONPATH`。安装后也可以使用 `radio-locator` 命令。

若需重新创建环境：

```bash
micromamba create -f environment.yml
micromamba activate mcm
python -m pip install -e .
```

系统没有中文字体时建议安装 Noto CJK；程序也会自动尝试思源黑体、文泉驿和霞鹜
文楷：

```bash
# Debian / Ubuntu，可选
sudo apt install fonts-noto-cjk
```

## 问题1和问题2

```bash
python main.py problem1
python main.py problem2
python main.py problem2 --x -900 --y -500 --bearing 31.363757
```

图片写入 `res/figures/`，数值结果写入 `res/tables/`。

## 问题3本地联调

终端一启动本地模拟器：

```bash
cd /home/Lilywhite/PIG/CUMCM/Code
micromamba activate mcm
python main.py simulator \
  --problem 3 \
  --seed 1 \
  --robot-id demo \
  --host 127.0.0.1 \
  --port 2026
```

终端二运行策略：

```bash
cd /home/Lilywhite/PIG/CUMCM/Code
micromamba activate mcm
python run.py problem3 \
  --strategy belief_mpc \
  --host 127.0.0.1 \
  --port 2026 \
  --robot-id demo \
  --seed 1 \
  --truth-file res/simulator/problem3_seed1.json
```

另一策略为：

```bash
python run.py problem3 \
  --strategy robust_polygon_rolling \
  --host 127.0.0.1 \
  --port 2026 \
  --robot-id demo \
  --seed 1 \
  --truth-file res/simulator/problem3_seed1.json
```

路径必须使用 Linux 的 `/`，不要复制 Windows 盘符或 `res\simulator\...`。停止服务
在终端一按 `Ctrl+C`。本地问题4场景可用 `--problem 4`，但目前没有问题4策略。

附件固定操作序列：

```bash
python scripts/attachment_protocol_demo.py \
  --base-url http://127.0.0.1:2026 \
  --robot-id demo
```

虚拟时刻应依次为 `0、105、111、194、199、199`。

## 问题3策略与输出

`robust_polygon_rolling` 使用正多边形保证覆盖骨架，发现频道后滚动插入定位和清除
任务。`belief_mpc` 对每频道维护不存在原子和 `(空间网格点, 固定接收半径)` 的联合
权重 `P(Z_j,I_j,R_j|H_t)`。初始空间网格为150 m，收到示向后细化到30 m，接近
清除时细化到8 m；空间子粒子继承父粒子的半径，连续检测不会重新抽取 `R_j`。
none、near和示向角区间均进入条件贝叶斯更新。10～16个源的总数约束只用公共
log-odds偏移校正存在概率，不会强行指定具体频道。

候选先按 `η=(IG+覆盖增量+定位进度)/即时耗时` 粗筛，最终仍以
`Q=τ+E[V_hat]` 比较。`τ` 严格包含移动、每次5 s检测、换频1 s及清除3/5 s；
`V_hat` 同时计算MST/最少动作下界和“安全覆盖路线+最近邻+2-opt”可执行上界。
观测场景复制信念并虚拟贝叶斯更新，低概率角区间合并。批量动作不增加协议，而是
在同一点依次调用附件原生 `/measure`。

两种策略均保留1000 m保证覆盖路线。自适应动作只有比安全动作至少节省
`replan_time_margin` 秒且仍可恢复覆盖时才采用；候选为空、数值异常或连续无有效
观测时回到覆盖骨架。清除以连续示向区域最小覆盖圆 `ρ≤20 m` 或near为保证条件；
0.99概率清除还要求8 m细网格，失败后必须先获得新观测，禁止原地重复清除。

常用参数：

```bash
python run.py problem3 --help
```

- `--grid-step`：保证覆盖集合的离散网格步长；
- `--particles`：每频道粒子数；
- `--candidate-limit`：MPC候选动作上限；
- `--candidate-top-k`：效率粗筛后进入精确评价的动作数；
- `--horizon`、`--beam-width`：MPC搜索规模；
- `--coarse-grid-size`、`--fine-grid-size`、`--radius-grid-size`：联合网格分辨率；
- `--prune-threshold`：活跃计算集阈值，低概率质量仍可恢复；
- `--replan-margin`、`--clear-probability`：重规划迟滞与概率清除阈值；
- `--ablation A|B|C|D|E`：选择消融版本；
- `--fixed-polygon`：使用指定正多边形，不搜索参数；
- `--truth-file`：仅供本地运行结束后的复盘绘图。

输出目录：

- `res/logs/problem3/`：每一步原始请求、响应、耗时、状态、互信息和Q值；
- `res/tables/problem3/`：单局汇总与多边形搜索表；
- `res/figures/problem3/`：机器狗轨迹、动作结果和清除位置特写；
- `res/simulator/`：固定种子的本地场景真值。

每局结束时终端会先输出简明演练结果，再输出完整JSON。例如：

```text
=== 问题3第 1 局结果 ===
策略：belief_mpc
已清除/总数量：15/15（100.00%）
总虚拟耗时：5559.812 s (01:32:39.812)
虚拟耗时分解：移动 4903.812 s；检测 485.000 s；切频 96.000 s；清除 75.000 s
程序实际耗时：5.434 s
移动距离：24519.059 m；measure=97；切频=96；清除失败=0
```

本地演练传入 `--truth-file` 后使用事后真值给出精确总数量；没有真值但20个频道均
已完成时，也可由 `cleared/absent` 终态推断总数。尚未搜索完成且没有真值时会明确
显示“总数未知（题面上限16）”，不会把16误报成实际数量。轨迹图标题同步标注虚拟
耗时和已清除数/总数。

### A--E消融

- A：只按信息、覆盖、定位收益与时间的比值排序；
- B：加入预计剩余完成时间，使用单步评价；
- C：加入2～4步滚动时域；
- D：在C上加入分层自适应网格；
- E：在D上加入安全覆盖回退和重规划迟滞（完整模型）。

每个版本必须在同一seed的新模拟器实例上运行。例如：

```bash
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 \
  --robot-id demo --seed 21 --ablation E \
  --truth-file res/simulator/problem3_seed21.json
```

完成A--E后可汇总运行结果：

```bash
python scripts/summarize_problem3_ablation.py \
  res/tables/problem3/belief_mpc_seed21_run1_*.json
```

输出为 `res/tables/problem3/ablation_comparison.csv`。比较时先检查清除率，再比较总
虚拟时间；不能用较短时间掩盖未完成清除的版本。

## 官方端口与三次正式测试

复制环境变量模板并填写真实队号：

```bash
cp .env.example .env
# 人工编辑.env后加载；不要提交.env
source .env
```

单次运行：

```bash
python run.py problem3 \
  --strategy belief_mpc \
  --host 127.0.0.1 \
  --port 2026 \
  --robot-id "$CUMCM_ROBOT_ID"
```

正式三次测试：

```bash
python run.py problem3 \
  --strategy belief_mpc \
  --host 127.0.0.1 \
  --port 2026 \
  --robot-id "$CUMCM_ROBOT_ID" \
  --formal \
  --runs 3 \
  --next-run-wait 300 \
  --case-code A001 \
  --case-code A002 \
  --case-code A003 \
  --official-log "官方日志1.dat" \
  --official-log "官方日志2.dat" \
  --official-log "官方日志3.dat"
```

每局结束后仍需按附件要求在模拟器界面启动下一局。程序保留官方日志原文件名，且
不会在端口失败后使用伪数据。传输状态不明确时立即停止，不自动重复可能推进虚拟
时间的动作。官方接口不提供真值时不得传 `--truth-file`。

## 测试与静态检查

```bash
python -m pytest -q
ruff check .
python -m compileall -q config problems src tests main.py run.py
```

若暂时不安装项目，Pytest也会读取 `pyproject.toml` 中的 `pythonpath=[".","src"]`。
常见Linux问题如下：

- `Address already in use`：换 `--port`，或用 `ss -ltnp` 检查端口；
- `Permission denied`：用 `python run.py ...`，或确认入口具有执行权限；
- 中文显示方框：安装中文字体后删除 `/tmp/radio_locator_matplotlib_$(id -u)` 重建缓存；
- 无桌面/SSH绘图失败：不要改成GUI后端，项目默认使用 `Agg`；
- `ModuleNotFoundError`：确认使用的是 `mcm` 环境，并执行 `python -m pip install -e .`。
