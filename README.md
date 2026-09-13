## Radio_Locater!!
### 一定要

这个项目是 2026 年CUMCM B 题的实现代码，它完成了解题、作图、本地模拟、接口客户端API的使命。我们在项目提供统一命令行，本地HTTP的模拟器、官方接口客户端，以及打印动作日志、统计表和论文图表等等等……

#important
- 因为该项目原生在非windows环境开发，为了最终兼容windows，大量使用vibecoding进行了修改与重构，请务必保证按照文档步骤正确安装和运行
- 友情提示：conda / mamba / micromamba 管理环境
- 以下为项目使用说明，在运行代码前值得一读。

## Overview

| 模块       | 功能                                                        | 主要输出                             |
| ---------- | ----------------------------------------------------------- | ------------------------------------ |
| 问题 1     | 多站示向交会、可行域裁剪、区域直径、几何边界情况验证        | 验证表与定位示意图                   |
| 问题 2     | 根据首个检测点和示向度生成、筛选并评价第二检测点            | 候选点得分表与选址图                 |
| 问题 3     | 连接模拟器执行搜索、测向定位、路径规划和安全清除            | 动作日志、运行汇总、轨迹/耗时/清除图 |
| 问题 4     | 面向全向/定向混合源进行保证发现、联合定位和优化清除         | 保证网格、运行汇总和对比图           |
| 本地模拟器 | 复现附件规定的 `/enter`、`/measure`、`/clear`、`/exit` 接口 | 可重复的随机案例和事后真值           |

## 下面是项目结构↓

```text
.
├── run.py / main.py                  # 命令行入口 ；报告墙钟时间
├── environment.yml / pyproject.toml  # Conda 系环境与 Python 包配置
├── config/                           # configurations：Path and parameters and constants
├── problems/
│   ├── problem1/                     # 交会定位与几何验证
│   ├── problem2/                     # 第二检测点选址
│   ├── problem3/                     # 全向源在线策略、状态和统计
│   └── problem4/                     # 混合源保证扫描和清除策略
├── src/radio_locator/
│   ├── cli.py                        # 命令行解析文件
│   ├── client.py                     # 官方 or 本地模拟器 HTTP 客户端
│   ├── local_simulator.py            # 问题 3、4 本地模拟器
│   ├── geometry.py                   # 几何算法
│   └── runtime.py                    # Windows字体、缓存与无ui绘图适配
├── scripts/                          # 协议
├── tests/                            # 回归测试脚本
├── paper/ / reference/               # 论文和题目附件
└── res/
    ├── figures/{png,pdf,source}/      # 论文图和运行图（不提交）
    ├── tables/                        # 数值结果和实验汇总（不提交）
    ├── logs/                          # 每局动作日志（不提交）
    └── simulator/                     # 真值（不提交）
```

题面物理常量集中在 `config/constants.py`，问题 3 的可调参数集中在 `config/problem3.yaml`，问题 4 的附加参数位于 `problems/problem4/config.py`。在线策略不会读取 `res/simulator/` 中的真值。

论文附录源码模板位于 `paper/appendix_code_template.tex`。在 `paper` 目录使用 XeLaTeX 编译时，它会自动载入当前项目的目录说明和全部 66 个源码/配置文件。


## Windows 环境复现（必读！！）

推荐使用 64 位 Windows 10/11、Miniconda 或 Anaconda，以及 PowerShell。Python 支持 3.11 至 3.14。

打开 Anaconda Prompt 或已经配置好 Conda 的 PowerShell，进入项目目录。路径可替换为自己的实际位置

```powershell
cd "C:\path\to\Radio_Locater"
conda env create -f environment.yml
conda activate mcm
python -m pip install -e .
```

如果名为 `mcm` 的环境已经存在，用以下命令同步依赖

```powershell
conda env update -n mcm -f environment.yml --prune
conda activate mcm
python -m pip install -e .
```

可编辑安装完成后，可使用 `python run.py ...`、`python main.py ...` 或 `radio-locator ...`。

如果旧环境在 NumPy/Matplotlib 绘图时出现 Windows 胎里异常，先试试上面的 `conda env update ... --prune`；还失败的话请新建干净环境，不要把其他 Python 安装目录手工追加到该环境的   PATH里！！




## 问题 1、2

```powershell
python run.py problem1
python run.py problem2
python run.py problem2 --x -900 --y -500 --bearing 31.363757
```

问题 1 生成定位与几何验证结果；问题 2 的第三条命令自定义首个检测点 `(x, y)` 和示向度。示向角以正东为 `0°`，逆时针为正。

## 本地模拟问题 3

在第一个 PowerShell 窗口启动服务，并保持窗口运行：

```powershell
conda activate mcm
cd "C:\path\to\Radio_Locater"
python run.py simulator --problem 3 --seed 1 --robot-id demo --host 127.0.0.1 --port 2026
```

在第二个 PowerShell 窗口运行推荐策略：

```powershell
conda activate mcm
cd "C:\path\to\Radio_Locater"
python run.py problem3 --strategy dynamic_coverage_route_aligned_tour --host 127.0.0.1 --port 2026 --robot-id demo --seed 1 --truth-file res/simulator/problem3_seed1.json
```

`--truth-file` 仅在策略退出后用于统计和复盘绘图，正式测试不要传入。停止本地服务时在第一个窗口按 `Ctrl+C`。

### 问题 3 策略

当前命令行的 6 个策略：

| 策略名                                | 方法与用途                                                                                    |
| ------------------------------------- | --------------------------------------------------------------------------------------------- |
| `robust_polygon_rolling`              | 原点加正六边形进行全局保证覆盖，发现目标后滚动插入定位和安全清除；结构清楚的保守基线。        |
| `belief_mpc`                          | 使用粒子信念、候选动作和有限时域 Beam Search 决策，同时保留可恢复覆盖骨架；适合概率规划研究。 |
| `integrated_bearing_tour`             | 将覆盖站、粗定位点和清除任务统一放入动态开放旅行商路径；联合优化基础方案。                    |
| `distance_optimized_bearing_tour`     | 在联合巡回上加入多起点构造、2-opt、单点重插入和小规模 Held--Karp 精确求解。                   |
| `safe_clear_route_aligned_tour`       | 在线选择较优覆盖方向，并在定位圆内沿后续路线移动安全清除点，以减少绕行。                      |
| `dynamic_coverage_route_aligned_tour` | **当前方案！！**在安全清除策略上，当已发现源数达到题设上限时提前终止无效覆盖；推荐用于演练。  |

运行其他策略时只需替换 `--strategy`：

```powershell
python run.py problem3 --strategy integrated_bearing_tour --host 127.0.0.1 --port 2026 --robot-id demo --seed 1
```

联合巡回参数可用 `--tour-radius`、`--endgame`、`--channel-order` 覆写；路线对齐参数使用 `--route-aligned-sides` 和 `--route-aligned-radius`；`belief_mpc` 还支持 `--grid-step`、`--particles`、`--candidate-limit`、`--horizon` 和 `--beam-width`。完整参数见：

```powershell
python run.py problem3 --help
```



## 本地模拟问题 4

问题 4 默认采用 中心 + 12 点内环 + 12 点外环 的同心环三角剖分。内环半径为 925 m；剖分最长边不超过最低接收半径 1000 m，外环覆盖目标圆边界，从而为未知朝向的定向源提供几何发现保证。

在第一个 PowerShell 窗口启动问题 4 模拟器

```powershell
python run.py simulator --problem 4 --seed 0 --robot-id demo --host 127.0.0.1 --port 2026
```

在第二个窗口运行推荐的优化保证策略

```powershell
python run.py problem4 --strategy optimized_guaranteed_lattice --host 127.0.0.1 --port 2026 --robot-id demo --seed 0 --truth-file res/simulator/problem4_seed0.json
```

3 个策略：

| 策略名                           | 方法与保证                                                                                                 |
| -------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| `guaranteed_directional_lattice` | 保证网格扫描，途中积累交会示向并顺路清除，末段集中处理剩余频道；保守基线。                                 |
| `optimized_guaranteed_lattice`   | **当前方案！** 保持相同发现保证，统一规划待定位与待清除任务，并采用路线对齐安全清除点；                    |
| `legacy_outer_probe_fast`        | 历史 13 站外侧补测路线，禁用错误的定向阴性裁剪并增加安全定位清除兜底；不具备全域定向发现保证，仅用于对照。 |

`--grid-spacing`、`--bearing-limit` 和 `--clear-insertion` 可覆盖问题 4 参数。快速复核采用 10 个种子即可：

```powershell
python scripts/benchmark_problem4.py --seed-start 0 --seed-count 10 --only optimized_guaranteed_lattice
```

## 官方客户端演练与正式运行

先在官方模拟器登录并启动对应问题，等待倒计时结束且界面显示接口就绪。PowerShell 中可通过环境变量保存连接信息：

```powershell
$env:CUMCM_ROBOT_ID = "你的参赛队号"
$env:CUMCM_BASE_URL = "http://127.0.0.1:2026"
python run.py problem3 --strategy dynamic_coverage_route_aligned_tour --robot-id $env:CUMCM_ROBOT_ID
```

程序串行调用 `/enter`、`/measure`、`/clear` 和 `/exit`。每局结束后会报告模拟器总耗时、清除数量、移动距离和各类动作耗时，随后 `run.py` 输出本机程序实际运行时间。完整数据写入 JSON；添加 `--print-json` 可同时打印到终端。

正式问题 3 可编排三局；每局还需在官方 GUI 中手动启动：

```powershell
python run.py problem3 --strategy dynamic_coverage_route_aligned_tour --robot-id $env:CUMCM_ROBOT_ID --formal --runs 3 --next-run-wait 300 --case-code A001 --case-code A002 --case-code A003 --official-log "官方日志1.dat" --official-log "官方日志2.dat" --official-log "官方日志3.dat"
```

网络状态不明确时客户端不会自动重发可能已经执行的动作。若收到 `accepted=false`，检查官方界面是否就绪、参赛队号是否一致，以及当前案例是否已被其他进程进入。

## 输出位置

- `res/logs/problem3/`、`res/logs/problem4/`：逐动作 JSONL 日志。
- `res/tables/problem3/`、`res/tables/problem4/`：单局汇总、参数搜索和审计结果。
- `res/figures/{png,pdf}/problem3/<run-id>/`、`problem4/<run-id>/`：每局轨迹、清除细节和耗时图。
- `res/figures/{png,pdf}/pic1` 至 `pic4`：精选论文插图。
- `res/simulator/`：真值，供运行后核查。


## 协议与回归检查

本地模拟器运行时，可用附件固定动作序列检查四个接口和计时规则：

```powershell
python scripts/attachment_protocol_demo.py --robot-id demo
```

此物仅供在本地或官方演练

检查命令

```powershell
python -m pytest -q
ruff check problems src tests run.py
python run.py problem3 --help
python run.py problem4 --help
```

