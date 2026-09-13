# 无线电干扰源自动定位与清除

2026 年数学建模竞赛 B 题合并项目。数值模型和问题 4 以
`Radio_Locater/master` 为准，论文绘图、分题图片目录及相关展示功能来自
`Radio_Locater_p`；问题 4 的自动搜索策略仍待实现。

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
│   └── problem4/                     # 定向/全向混合问题的配置预留
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
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 --robot-id demo --seed 1 --truth-file res/simulator/problem3_seed1.json
```

也可将 `belief_mpc` 改为 `robust_polygon_rolling`。`--truth-file` 只用于本地结束后的复盘绘图；官方客户端测试绝不能传入真值。

用附件的固定操作序列验证四个接口和计时规则：

```bash
python scripts/attachment_protocol_demo.py --robot-id demo
```

它会执行 `/enter → /measure → /measure → /clear → /measure → /exit`，虚拟时刻应为 `0, 105, 111, 194, 199, 199`。仅在本地模拟器或官方演练中使用，不能用于正式测试。

停止本地服务请在服务窗口按 `Ctrl+C`。本地问题 4 场景可用 `--problem 4` 启动；当前复用问题 3
策略接口进行联调，运行时传入对应问题 4 的 `--truth-file`，程序会自动统计全向/定向源数量，
并将图组写入 `problem4/<run-id>/`。

## 官方客户端演练与正式测试

先在官方模拟器完成登录，并进入“问题 3 演练测试”或“问题 3 正式测试”。点击开始后等待倒计时结束，且界面明确显示接口已就绪，再运行程序。

```bash
export CUMCM_ROBOT_ID="你的参赛队号"
export CUMCM_BASE_URL="http://127.0.0.1:2026"
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
```

不要复制 shell 提示符，也不要输入 Markdown 转义后的 `belief\_mpc`；命令中应使用普通下划线 `belief_mpc`。

程序会串行调用 `/enter`、`/measure`、`/clear` 和 `/exit`。每局结束时终端打印总耗时、清除与
干扰源数量、单源平均耗时、移动距离、动作次数，以及移动、换频、检测、清除和其他耗时。
完整数据始终写入汇总 JSON；若还需在终端打印完整 JSON，可加 `--print-json`。
`accepted=false` 表示该动作未执行：检查官方客户端是否已就绪、当前登录队号是否一致、或当前局是否已经进入过。传输中断时程序会停止，不会自动重发可能已执行的动作。

正式测试前在 GUI 中记录案例编码；每局结束后从官方模拟器导出加密日志，保持原文件名。程序生成的明文动作日志、汇总表和机器狗轨迹图分别在 `res/logs/problem3/`、`res/tables/problem3/`、`res/figures/{png,pdf}/problem3/<run-id>/`；论文插图另存入 `res/figures/{png,pdf}/pic3/`。

连续编排三次正式测试：

```bash
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID" --formal --runs 3 --next-run-wait 300 --case-code A001 --case-code A002 --case-code A003 --official-log "官方日志1.dat" --official-log "官方日志2.dat" --official-log "官方日志3.dat"
```

每局结束后仍需在官方 GUI 手动启动下一局；`--next-run-wait` 是程序等待接口重新开放的最长秒数。正式汇总追加至 `res/tables/problem3_formal_runs.csv`。

## 常用命令

```bash
python run.py problem3 --help
python -m pytest -q
ruff check .
```

问题 3 的两套策略彼此独立：`robust_polygon_rolling` 固定采用原点加半径 1200 m 的正六边形，结合滚动定位插入、19.8 m 安全清除阈值、Held--Karp 末端清除路径和 28 m 网格兜底；`belief_mpc` 继续使用候选正多边形参数搜索和有限时域 Beam Search。常用调参为 `--grid-step`、`--particles`、`--candidate-limit`、`--horizon` 和 `--beam-width`，两套策略的固定参数也可在 `config/problem3.yaml` 中分别修改。正式测试应先在演练中确认参数有效。

### 更快的问题3联合路线策略

新增 `integrated_bearing_tour`：覆盖测站同时为多个频道交会定位，覆盖点与已定位到150m以内的目标一起进行最近邻+2-opt滚动开放路线规划。实际清除仍遵循19.8m判据。26个本地场景均全清，平均3737秒；具体对照与限制见《问题三思路与具体方法》第13节。

```bash
python run.py problem3 --strategy integrated_bearing_tour --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
```

距离优先的实验变体 `distance_optimized_bearing_tour` 保持相同覆盖、检测、定位和清除
规则；当联合任务超过 10 个时，改用多起点构造、2-opt 和单点重插入联合搜索。
原策略仍完整保留，可用相同种子执行配对比较：

```bash
python run.py problem3 --strategy distance_optimized_bearing_tour --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
python scripts/benchmark_distance_tour.py --start 0 --count 120
```

进一步的实验变体 `route_aligned_bearing_tour` 会先完成起点扫描，再在一个多边形
对称周期内比较 12 个等价朝向。评分综合后验可接收概率与交会角的平方正弦，抑制
近共线的低质量交会；定位和安全清除规则不变。可用同一批种子与原策略配对：

```bash
python run.py problem3 --strategy route_aligned_bearing_tour --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
python scripts/benchmark_distance_tour.py --candidate route_aligned_bearing_tour --start 0 --count 120
python scripts/tune_route_aligned_tour.py --start 0 --count 30
python scripts/tune_route_aligned_tour.py --stage fine --start 0 --count 30
```

初始六边形 1150 m 版本在固定种子 0--119 的配对模拟中，两者均 120/120 全清
且无清除失败；该变体平均
移动距离由 12924.40 m 降至 12819.04 m（减少 0.815%），平均总耗时由
3528.49 s 降至 3505.13 s（减少 0.662%），62 局更快、7 局相同、51 局更慢。
该结果用于分离“在线选角”本身的收益。

覆盖多边形网格调优进一步比较了 6--9 边的 13 组可行参数。120 局复核后，
`route_aligned_bearing_tour` 独立采用七边形 1050 m：平均移动 12488.94 m，
较原六边形联合策略减少 435.46 m（3.369%）；平均总耗时 3479.69 s，减少
48.80 s（1.383%）。可用 `--route-aligned-sides` 与
`--route-aligned-radius` 覆写，新默认不会改变原 `integrated_bearing_tour`。
七边形 1025--1075 m 的 5 m 步长细化仍以 1050 m 最优；排除用于选参的前30局后，
seed 30--119 留出集平均仍少走 379.39 m（2.952%）、节省 33.70 s
（0.960%），54局更快、36局更慢。

`safe_clear_route_aligned_tour` 在上述策略上进一步利用定位圆的安全余量：若定位
圆半径为 $\rho$，只允许清除点在圆心周围 $19.8-\rho$ m 内移动，并在其中最小化
“当前位置--清除点--下一任务”的折线路径。三角不等式保证任意可能源到新清除点
仍不超过 19.8 m。固定种子 0--119 中保持 120/120 全清、零清除失败，相对
`route_aligned_bearing_tour` 平均少走 57.11 m、节省 13.73 s，90局更快、30局
更慢。运行方式：

```bash
python run.py problem3 --strategy safe_clear_route_aligned_tour --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
python scripts/benchmark_distance_tour.py --baseline route_aligned_bearing_tour --candidate safe_clear_route_aligned_tour --start 0 --count 120
```

`dynamic_coverage_route_aligned_tour` 再利用题设“干扰源至多16个”的确定上限：当
16个不同频道已经处于已发现、已定位或已清除状态时，剩余未知频道必然不存在，立即
结束剩余全覆盖骨架并转入目标处理。未达到上限时完全保留七边形覆盖路线。120个固定
种子中相对安全清除策略为20局更快、100局相同、0局更慢，平均再少走33.80 m、
节省16.50 s，仍为120/120全清且零清除失败：

```bash
python run.py problem3 --strategy dynamic_coverage_route_aligned_tour --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
python scripts/benchmark_distance_tour.py --baseline safe_clear_route_aligned_tour --candidate dynamic_coverage_route_aligned_tour --start 0 --count 120
```

沿用原模拟器队号；本地demo可将机器人编号改为`demo`。另有`cooperative_bearing_tour`作为共享测站后集中清除的对照方案。两种旧策略仍可使用。

2026-09-12参数更新：联合巡回默认`tour_polygon_radius_m: 1150.0`，仍保留原点扫描。60场景配对中平均总时间由3800.28秒降至3699.96秒；51局更快、9局更慢。新增`--tour-radius 1200`可恢复旧半径，`--channel-order alternating`可试验相邻非空扫描批次升降序交替，默认`legacy`优先当前频道。交替顺序在本地试验中平均增加约2秒，故未启用为默认。完整覆盖证明与分组数据见方法文档第14节。

末段绕行修订：默认`tour_endgame_mode: probe`，在最多剩2个覆盖节点时让已发现的粗定位目标参加排路，并先做一次侧向共享补测。候选节点≤10时使用Held–Karp精确开放路线。`--endgame legacy`可恢复原模式，`--endgame exact`只启用小规模精确排路。120个配对场景中新默认均值3528.87秒，旧模式3717.74秒，83局改善；不保证每局变快。P5真实历史状态复盘见方法文档第15节。
