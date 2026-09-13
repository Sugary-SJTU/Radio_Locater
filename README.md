# 无线电干扰源自动定位与清除

2026 年数学建模竞赛 B 题合并项目。数值模型和问题 4 以
`Radio_Locater/master` 为准，论文绘图、分题图片目录及相关展示功能来自
`Radio_Locater_p`。问题 4 已实现定向保证发现、联合定位和顺路清除策略。

## 目录与职责

```text
.
├── main.py / run.py                  # 等价命令行入口
├── environment.yml / pyproject.toml  # Conda 环境、Python 依赖与命令注册
├── config/                           # 题面常量、接口配置、路径和问题3参数
├── problems/
│   ├── problem1/                     # 交会定位、区域直径、覆盖判定与绘图
│   ├── problem2/                     # 第二检测点选址、候选区域与绘图
│   ├── problem3/                     # 在线搜索、定位、清除、日志与统计
│   └── problem4/                     # 定向保证网格、混合源定位清除与运行入口
├── src/radio_locator/
│   ├── cli.py                        # problem1、problem2、problem3、simulator 命令
│   ├── client.py                     # 官方模拟器四个 HTTP 接口客户端
│   ├── local_simulator.py             # 本地问题3/4模拟器
│   ├── geometry.py                   # 凸几何、半平面裁剪、旋转卡壳
│   └── runtime.py                    # 跨平台绘图缓存、字体和无界面后端适配
├── scripts/attachment_protocol_demo.py # 附件接口与计时示例
├── reference/                        # B题原文、附件1和附件2
├── paper/                            # 当前方案论文
├── res/
│   ├── figures/
│   │   ├── png/                       # PNG 输出镜像
│   │   │   ├── pic1/ ... pic4/       # 问题 1--4 的论文与模型说明图
│   │   │   └── problem3/、problem4/   # 各次机器狗运行的独立图组
│   │   ├── pdf/                       # 与 png/ 同结构的 PDF 输出镜像
│   │   └── source/                    # SVG、Markdown 等图形源文件
│   ├── tables/                       # 结果表、参数搜索和正式测试汇总
│   ├── logs/                         # 程序动作 JSONL 日志
│   └── simulator/                    # 本地模拟器真值，仅供复现核查
└── tests/                            # 回归测试
```

`config/constants.py` 只保存题面常量；问题 3 的可调算法参数位于 `config/problem3.yaml`。运行生成的内容都在 `res/`；不要让策略程序读取 `res/simulator/` 中的本地真值。

## 安装与检查

在 Linux shell 中：

```bash
micromamba activate mcm
cd /home/Lilywhite/PIG/CUMCM/Radio_Locater
python -m pip install -e .
python -m pytest -q
```

若 shell 尚未初始化 micromamba，可将上述每条 `python` 命令替换为
`micromamba run -n mcm python`。`pip install -e .` 后无需设置 `PYTHONPATH`，可用
`python main.py ...`、`python run.py ...` 或 `radio-locator ...` 运行；两个根入口在未安装前也可直接使用。
其中 `run.py` 会在命令结束或中断退出时额外打印从程序启动到退出的实际墙钟运行时间；
该数值用于衡量算法在电脑上的执行时间，与模拟器依据移动、检测、换频和清除动作累计的
虚拟时间相互独立。

## 问题 1、2

```bash
# 问题1：交会定位验证、区域直径和论文插图
python main.py problem1

# 问题2：默认首个检测点的第二检测点选址
python main.py problem2

# 自定义首个检测点与示向度
python main.py problem2 --x -900 --y -500 --bearing 31.363757
```

每幅图同时写入 `res/figures/png/` 和 `res/figures/pdf/` 的镜像路径。问题 1--4
论文图使用 `pic1/`--`pic4/`；机器狗每次运行在
`problem3/<run-id>/` 或 `problem4/<run-id>/` 下单独保存轨迹、清除细节和耗时图。
数据表写入 `res/tables/`。

## 问题 3 本地联调

先开一个终端启动本地服务（不要关闭此窗口）：

```bash
micromamba activate mcm
cd /home/Lilywhite/PIG/CUMCM/Radio_Locater
python main.py simulator --problem 3 --seed 1 --robot-id demo
```

再开第二个终端运行策略：

```bash
micromamba activate mcm
cd /home/Lilywhite/PIG/CUMCM/Radio_Locater
python run.py problem3 --strategy dynamic_coverage_route_aligned_tour \
  --host 127.0.0.1 --port 2026 --robot-id demo --seed 1 \
  --truth-file res/simulator/problem3_seed1.json
```

`--truth-file` 只用于程序退出后的统计和复盘绘图；策略运行期间不会读取它，官方客户端
测试不要传入真值。

### 问题 3 全部策略

命令行当前注册了 8 个策略：

| 策略名 | 核心方法 | 适用场景 |
|---|---|---|
| `robust_polygon_rolling` | 原点加半径 1200 m 正六边形保证覆盖；整站扫描后滚动插入定位，19.8 m 安全清除，末段 Held--Karp 和 28 m 网格兜底 | 结构清楚、强调保守可解释性的基线 |
| `belief_mpc` | 搜索 6--9 边覆盖骨架，以粒子信念、互信息和有限时域 Beam Search 滚动选择检测动作，同时始终保留可恢复的保证路线 | 概率规划和参数研究对照，计算量最大 |
| `cooperative_bearing_tour` | 原点加半径 1200 m 正六边形；同一覆盖测站同时为多个已发现频道积累交会示向，覆盖结束后按开放路线集中定位清除 | 共享测站思想的消融基线 |
| `integrated_bearing_tour` | 半径 1150 m 正六边形与原点构成保证骨架，把覆盖点和粗定位目标放入同一个动态开放旅行商问题；末段可做侧向共享补测 | 联合覆盖、定位和清除的基础高效方案 |
| `distance_optimized_bearing_tour` | 继承联合巡回动作规则；任务超过 10 个时使用多起点构造、2-opt 和单点重插入，10 个以内使用 Held--Karp | 单纯检验路线搜索改进收益 |
| `route_aligned_bearing_tour` | 起点先扫描，再依据在线示向在 12 个等价朝向中选择七边形 1050 m 覆盖骨架；评分兼顾接收概率与交会角 | 缩短覆盖路线并提高交会质量 |
| `safe_clear_route_aligned_tour` | 在路线对齐策略上，利用定位圆剩余安全半径把清除点向“当前位置—下一任务”路径移动 | 保持严格清除判据并进一步减少绕行 |
| `dynamic_coverage_route_aligned_tour` | 在安全清除策略上，发现数达到题设上限 16 后立即判定其他频道不存在并结束剩余覆盖 | 当前完整优化链；建议优先用于正式演练 |

后五个策略构成逐级可比的优化链：
`integrated_bearing_tour` → `distance_optimized_bearing_tour` →
`route_aligned_bearing_tour` → `safe_clear_route_aligned_tour` →
`dynamic_coverage_route_aligned_tour`。每个子策略只增加一类优化，便于配对消融。

任一策略都用同一命令格式运行：

```bash
python run.py problem3 --strategy <上表策略名> \
  --host 127.0.0.1 --port 2026 --robot-id demo --seed 1
```

联合巡回相关参数可用 `--tour-radius`、`--endgame` 和 `--channel-order` 覆写；路线
对齐策略另支持 `--route-aligned-sides`、`--route-aligned-radius`。`belief_mpc` 常用
`--grid-step`、`--particles`、`--candidate-limit`、`--horizon` 和 `--beam-width`。
完整默认值位于 `config/problem3.yaml` 和 `problems/problem3/config.py`。

用附件的固定操作序列验证四个接口和计时规则：

```bash
python scripts/attachment_protocol_demo.py --robot-id demo
```

它会执行 `/enter → /measure → /measure → /clear → /measure → /exit`，虚拟时刻应为 `0, 105, 111, 194, 199, 199`。仅在本地模拟器或官方演练中使用，不能用于正式测试。

停止本地服务请在服务窗口按 `Ctrl+C`。

## 问题 4 定向/全向混合源

问题四采用“中心 + 12点内环 + 12点外环”的同心环三角剖分；内环半径取 925 m（在
1000 m 接收半径的几何约束内缩短巡回路径）。每个剖分三角形最长边
不超过最低接收半径 1000 m，且外环正十二边形外切目标圆，因此任意源位置附近的测站
凸包都包含源位置；无论定向半平面朝向如何，至少一个测站必然收到信号。

本地联调先启动问题四模拟器：

```bash
python main.py simulator --problem 4 --seed 0 --robot-id demo --port 2026
```

再在另一终端运行策略：

```bash
python run.py problem4 --host 127.0.0.1 --port 2026 --robot-id demo --seed 0 \
  --truth-file res/simulator/problem4_seed0.json
```

### 问题 4 全部策略

| 策略名 | 核心方法 | 完备性与用途 |
|---|---|---|
| `guaranteed_directional_lattice` | 默认策略；25 站同心环（也可配置为 27 站平移三角格），保证扫描途中机会式交会并顺路插入安全清除，末段先清已定位频道再处理其余频道 | 具有定向源发现保证，保守正式基线 |
| `optimized_guaranteed_lattice` | 保留相同保证网格，把已定位和待定位频道统一纳入 Held--Karp 精确开放路径收尾，并使用路线对齐安全清除点 | 具有相同发现保证，通常比默认收尾更快；建议优先演练 |
| `problem4_fast` | 复现父项目：半径 1000 m 七边形加半径 2246 m 外侧六点，使用对定向源不严格成立的阴性圆裁剪，只做有限次主动定位且无网格兜底 | 不保证发现或清除全部源，仅作高速原方法对照 |
| `legacy_outer_probe_fast` | 保留相同 13 站父项目路线，但禁用错误的定向阴性裁剪，并为已发现频道启用安全定位清除兜底 | 仍无定向发现保证；用于衡量修补原路线的代价 |

运行非默认策略时显式传入名称，例如：

```bash
python run.py problem4 --strategy optimized_guaranteed_lattice \
  --host 127.0.0.1 --port 2026 --robot-id demo --seed 0 \
  --truth-file res/simulator/problem4_seed0.json
```

问题 4 还支持 `--grid-spacing`、`--bearing-limit` 和 `--clear-insertion` 调参。

策略运行期间不读取真值；`--truth-file` 仅在退出后统计全向/定向源数量并绘制复盘图。
单局输出写入 `res/logs/problem4/`、`res/tables/problem4/` 和
`res/figures/{png,pdf}/problem4/<run-id>/`。

复现保证路线、原始高速路线和安全兜底路线的同种子比较，或扩大单一策略审计样本：

```bash
python scripts/benchmark_problem4.py --seed-start 0 --seed-count 10
python scripts/benchmark_problem4.py --seed-start 0 --seed-count 10 --only optimized_guaranteed_lattice
```

优化保证策略采用 925 m 内环后，在 seed 0--9 上为 10/10 全源清除，平均虚拟耗时
7457.14 s、平均移动 26274.69 m。父项目高速策略虽然用时较短，但速度优势来自不完备
的定向阴性裁剪和提前停止，不能与全清除策略只按耗时比较。对比数据和图分别位于
`res/tables/problem4/problem4_strategy_comparison.json` 与
`res/figures/{png,pdf}/pic4/`。

## 官方客户端演练与正式测试

先在官方模拟器完成登录，并进入“问题 3 演练测试”或“问题 3 正式测试”。点击开始后等待倒计时结束，且界面明确显示接口已就绪，再运行程序。

```bash
export CUMCM_ROBOT_ID="你的参赛队号"
export CUMCM_BASE_URL="http://127.0.0.1:2026"
python run.py problem3 --strategy dynamic_coverage_route_aligned_tour \
  --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
```

不要复制 shell 提示符，也不要在策略名的下划线前添加 Markdown 转义符。

程序会串行调用 `/enter`、`/measure`、`/clear` 和 `/exit`。每局结束时终端打印总耗时、清除与
干扰源数量、单源平均耗时、移动距离、动作次数，以及移动、换频、检测、清除和其他耗时。
完整数据始终写入汇总 JSON；若还需在终端打印完整 JSON，可加 `--print-json`。
`accepted=false` 表示该动作未执行：检查官方客户端是否已就绪、当前登录队号是否一致、或当前局是否已经进入过。传输中断时程序会停止，不会自动重发可能已执行的动作。

正式测试前在 GUI 中记录案例编码；每局结束后从官方模拟器导出加密日志，保持原文件名。程序生成的明文动作日志、汇总表和机器狗轨迹图分别在 `res/logs/problem3/`、`res/tables/problem3/`、`res/figures/{png,pdf}/problem3/<run-id>/`；论文插图另存入 `res/figures/{png,pdf}/pic3/`。

连续编排三次正式测试：

```bash
python run.py problem3 --strategy dynamic_coverage_route_aligned_tour \
  --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID" \
  --formal --runs 3 --next-run-wait 300 \
  --case-code A001 --case-code A002 --case-code A003 \
  --official-log "官方日志1.dat" --official-log "官方日志2.dat" \
  --official-log "官方日志3.dat"
```

每局结束后仍需在官方 GUI 手动启动下一局；`--next-run-wait` 是程序等待接口重新开放的
最长秒数。正式汇总追加至 `res/tables/problem3/formal_runs.csv`。

## 常用命令

```bash
python run.py problem3 --help
python run.py problem4 --help
python -m pytest -q
ruff check .
```

## 策略配对与调参复现

问题 3 的联合巡回优化链可按相同种子逐级配对；以下命令默认只做 10 局快速检查，
需要扩大样本时再增加 `--count`：

```bash
# integrated 与 distance_optimized
python scripts/benchmark_distance_tour.py --start 0 --count 10

# route_aligned、safe_clear、dynamic_coverage 的逐级比较
python scripts/benchmark_distance_tour.py --candidate route_aligned_bearing_tour --start 0 --count 10
python scripts/benchmark_distance_tour.py --baseline route_aligned_bearing_tour \
  --candidate safe_clear_route_aligned_tour --start 0 --count 10
python scripts/benchmark_distance_tour.py --baseline safe_clear_route_aligned_tour \
  --candidate dynamic_coverage_route_aligned_tour --start 0 --count 10

# 路线对齐多边形的粗调与细调
python scripts/tune_route_aligned_tour.py --start 0 --count 10
python scripts/tune_route_aligned_tour.py --stage fine --start 0 --count 10
```

当前问题 3 路线对齐默认值是七边形 1050 m；联合巡回基线仍独立使用原点加六边形
1150 m。末段默认 `probe`：剩余覆盖节点不超过 2 个时，把粗定位目标加入排路并做一次
侧向共享补测；候选任务不超过 10 个时使用 Held--Karp 精确开放路线。历史完整样本、
速度/精度对照和方法说明见 `问题三与问题四策略与表现整理.md`。
