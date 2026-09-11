# 2026 高教社杯数学建模竞赛 B 题

题目：无线电干扰源的快速自动定位与清除。

本仓库已经实现问题 1、2 的数值算法和论文插图、问题 3 的两种在线策略，并提供
问题 3、4 共用的附件兼容本地模拟器。问题 4 的自动搜索清除策略仍为后续工作。

## 精简后的结构

```text
.
├── main.py / run.py              # 两个等价的根运行入口
├── pyproject.toml                # Python 包、依赖、命令行入口及工具配置
├── environment.yml              # Conda 环境配置
├── .env.example                 # 模拟器环境变量示例，不保存真实队号
├── config/
│   ├── constants.py             # 题面和附件明确给出的公共常量
│   ├── paths.py                 # 题目文件及结果目录的绝对路径
│   ├── local_simulator.py       # 本地模拟器服务与协议限制参数
│   ├── problem3.yaml            # 问题3集中算法参数
│   └── simulator.py             # 模拟器地址、端点和响应状态常量
├── problems/
│   ├── problem1/
│   │   ├── config.py            # 问题 1 参数与输出文件名
│   │   ├── model.py             # 交会定位、直径与覆盖判定
│   │   ├── examples.py          # 三组可复现检测数据
│   │   ├── plotting.py          # 问题 1 的四类论文图
│   │   └── main.py              # 问题 1 验证和绘图入口
│   ├── problem2/
│   │   ├── config.py            # 问题 2 参数与输出文件名
│   │   ├── model.py             # 粗筛、直径增益和局部细化
│   │   ├── plotting.py          # 粗筛原理及候选区域图
│   │   └── main.py              # 问题 2 选点和绘图入口
│   ├── problem3/
│   │   ├── config.py            # 参数类型、策略名和输出路径
│   │   ├── coverage.py          # 正多边形覆盖证明、搜索和路径优化
│   │   ├── belief.py            # (Z,I,R)联合粒子信念和互信息
│   │   ├── shared.py            # U集合、状态机、执行器、日志和定位任务
│   │   ├── strategies.py        # rolling与belief MPC在线控制器
│   │   ├── plotting.py          # 单局运动轨迹、事件时间轴和清除特写
│   │   └── main.py              # 端口运行、三次测试与结果汇总
│   └── problem4/
│       ├── config.py            # 问题 4 测试参数与日志路径
│       └── main.py              # 问题 4 策略预留入口
├── src/radio_locator/
│   ├── __init__.py              # 公共包的公开接口
│   ├── cli.py                   # 数值问题和本地模拟器统一命令行入口
│   ├── geometry.py              # 半平面裁剪和旋转卡壳公共算法
│   ├── local_simulator.py       # 问题 3/4 场景、物理状态机和 HTTP 服务
│   └── client.py                # 模拟器连接参数容器，当前不发送请求
├── scripts/
│   └── attachment_protocol_demo.py # 原样复现附件完整计时示例
├── paper/                        # 当前方案论文 PDF
├── reference/
│   ├── B题.pdf                  # 题目原文
│   └── 附件/                    # 模拟器使用说明与通信协议
└── res/
    ├── figures/                 # 各问题生成的图片
    ├── tables/                  # 计算结果和正式测试汇总表
    ├── simulator/               # 固定种子生成的本地案例真值
    └── logs/                    # 问题 3、4 的程序动作日志
```

删除了旧模板遗留的种植策略包、重复的 `run.py`、重复的 `config/problemN.py` 转发层，
以及当前没有读写需求的 `data`、`models`、`notebooks` 空目录。

## 文件职责

| 文件 | 功能 | 当前是否包含算法 |
|---|---|---|
| `main.py` | 将根目录运行方式转发给 `radio_locator.cli` | 否 |
| `config/constants.py` | 保存区域半径、频道范围、测向误差、移动速度、动作耗时和测试时限 | 否 |
| `config/paths.py` | 从文件位置推导项目根目录、题目附件路径和输出目录 | 否 |
| `config/simulator.py` | 读取环境变量，定义四个 HTTP 端点和协议结果码 | 否 |
| `config/local_simulator.py` | 定义本地监听、种子、请求大小和幂等缓存限制 | 否 |
| `problems/problem1/config.py` | 定义交会定位误差、计算容差和问题 1 输出路径 | 否 |
| `problems/problem1/model.py` | 示向度半平面求交、区域直径和直径圆覆盖判定 | 是 |
| `problems/problem1/examples.py` | 定义良好交会、弱交会和三站反例数据 | 验证数据 |
| `problems/problem1/plotting.py` | 绘制三站交会、旋转卡壳、验证组和反例 | 绘图 |
| `problems/problem1/main.py` | 运行问题 1 验证并导出 CSV 和 PNG | 是 |
| `problems/problem2/config.py` | 定义第二检测点问题的角度、区域和输出路径 | 否 |
| `problems/problem2/model.py` | 构造位置后验、粗筛候选点并按期望直径增益细化 | 是 |
| `problems/problem2/plotting.py` | 绘制四项粗筛原理和最终候选区域 | 绘图 |
| `problems/problem2/main.py` | 运行问题 2 选点并导出 CSV 和 PNG | 是 |
| `config/problem3.yaml` | 集中保存网格、粒子、MPC和正多边形默认参数 | 配置 |
| `problems/problem3/config.py` | 定义问题3参数类型、策略名和日志路径 | 否 |
| `problems/problem3/coverage.py` | 覆盖闭式/数值验证、参数搜索、插入代价和2-opt | 是 |
| `problems/problem3/belief.py` | 保持固定接收半径的联合粒子信念、none更新和互信息 | 是 |
| `problems/problem3/shared.py` | 管理U、定位区域、最小覆盖圆、真实动作和JSONL日志 | 是 |
| `problems/problem3/strategies.py` | 实现滚动正多边形与有限时域belief MPC | 是 |
| `problems/problem3/plotting.py` | 从动作日志绘制轨迹、动作结果和清除位置特写 | 绘图 |
| `problems/problem3/main.py` | 串联客户端、策略、三次正式测试和统计输出 | 是 |
| `problems/problem4/config.py` | 在问题 3 配置基础上增加定向源覆盖角 | 否 |
| `problems/problem4/main.py` | 预留混合干扰源搜索定位清除入口 | 否 |
| `src/radio_locator/client.py` | 严格调用四个端点；传输歧义时禁止自动重复动作 | 是 |
| `src/radio_locator/local_simulator.py` | 生成问题 3/4 案例并实现附件 HTTP、物理和计时规则 | 是 |
| `src/radio_locator/geometry.py` | 公共凸几何、半平面裁剪及旋转卡壳实现 | 是 |
| `src/radio_locator/cli.py` | 提供问题 1、2及问题 3/4 本地模拟器的统一入口 | 入口 |
| `pyproject.toml` | 声明项目元数据、运行依赖、包目录和 Ruff/Pytest 配置 | 不适用 |
| `environment.yml` | 创建可复现的 Conda 开发环境 | 不适用 |
| `.env.example` | 展示本机模拟器连接变量的写法 | 不适用 |

各 Python 源文件顶部均有模块说明；预留入口同时写明未来输入、输出和当前边界。

## 参数来源约定

- `config/constants.py` 只保存题目或附件明确规定的数值。
- 各题 `config.py` 只组合本题需要的公共参数并定义结果路径。
- 后续自行选择的网格间距、收敛阈值、搜索半径等算法超参数，应单独标注为
  “建模假设”或“算法参数”，不得混入题面常量。
- 问题 3、4 的运行状态，例如当前位置、当前频道、已清除目标，不写入配置文件。

## 环境与运行

```bash
micromamba activate mcm
export PYTHONPATH="src:."
```

生成问题 1、2 的验证表和全部图片：

```bash
python main.py problem1
python main.py problem2
# 自定义第一检测点 (x, y) 和实测示向度：
python main.py problem2 --x -900 --y -500 --bearing 31.363757
```

若已通过 `pip install -e .` 安装项目，也可以运行 `radio-locator problem1` 和
`radio-locator problem2`。

## 问题 3、4 本地模拟器

启动问题 3 全向源案例：

```bash
python main.py simulator --problem 3 --seed 2026 --robot-id demo
```

启动问题 4 全向/定向混合源案例：

```bash
python main.py simulator --problem 4 --seed 2026 --robot-id demo
```

服务默认监听 `http://127.0.0.1:2026`。同一题号和同一 `--seed` 会逐项复现源数量、
频道、坐标、接收半径、类型和定向方向；每局生成 10--16 个频道互异的源。问题 3
全部为全向源，问题 4 保证全向源和定向源均至少有一个。真值默认写到
`res/simulator/problemN_seedSEED.json`，只供本地核查，策略程序不应读取。

另开终端运行附件原始操作序列：

```bash
python scripts/attachment_protocol_demo.py --robot-id demo
```

该序列的虚拟时刻应依次为 `0、105、111、194、199、199`。本地服务实现精确路径
`/enter`、`/measure`、`/clear`、`/exit`，并复现移动、频道切换、检测、清除计时，
三类测量结果、两类清除结果、定向覆盖、固定地点测向误差、严格字段检查、请求体
限制、HTTP 400/404/405/409/413/415/429 和 `request_id` 幂等语义。

## 问题 3：两种在线策略

每个频道都维护 `channel_status[j]`、离散未保证覆盖集合 `U[j]` 以及联合信念
`belief[j]=(Z_j,I_j,R_j)`。存在粒子初始化时只抽取一次接收半径，后续观测只更新
权重，因此不会把同一源在不同位置的检测半径错误地当作独立随机数。`none` 同时
更新粒子权重，并从 `U[j]` 中排除检测点周围的 1000 m 圆；只有 `U[j]` 为空时才
能证明该频道 `absent`。`near` 会在记录测量响应后立即于当前位置调用 `/clear`。

`robust_polygon_rolling` 以正多边形为可恢复的保底路线。对不扫描原点的方案使用

```text
d_max = max(rho, sqrt(1800²+rho²-2*1800*rho*cos(pi/n)))
```

作连续覆盖证明，并以高分辨率网格复核。程序会比较 6、7、8、9 边、多组半径、
旋转角和是否扫描原点，按 `route_length/5 + 5*N_measure + N_switch` 选择可行方案；
`--fixed-polygon` 则保留七边形、半径 1000 m 基准。发现频道后按插入额外路程
`dist(Pa,Q)+dist(Q,Pb)-dist(Pa,Pb)` 决定立即定位还是延后，同时始终保留剩余骨架。

`belief_mpc` 每轮从保底节点、各频道 `U[j]` 代表点、问题2定位点和保证清除点生成
有限候选。互信息只更新被测频道，且 `information/time` 只用于预排序；最终以
`Q=tau+E[V_hat]` 最小化动作时间与剩余路线、检测、切频、清除和定位时间。默认
采用有限时域 beam search，仅执行首动作，收到真实观测后重算。定位区域必须通过
最小覆盖圆半径 `rho<=20 m` 才会建立保证清除任务，不能用“直径不超过40 m”替代。

先启动附件模拟器或本地模拟器，再在另一终端运行：

```bash
python run.py problem3 --strategy robust_polygon_rolling \
  --host 127.0.0.1 --port 2026 --robot-id demo --seed 1 \
  --truth-file res/simulator/problem3_seed1.json
python run.py problem3 --strategy belief_mpc \
  --host 127.0.0.1 --port 2026 --robot-id demo --seed 1 \
  --truth-file res/simulator/problem3_seed1.json
```

默认参数在 `config/problem3.yaml`。常用覆盖项包括 `--grid-step`、`--particles`、
`--candidate-limit`、`--horizon`、`--beam-width`、`--p0`、`--g0`、
`--polygon-sides`、`--polygon-radius`、`--polygon-rotation`、
`--robustness-margin` 和 `--scan-origin`。题面固定的保证半径只能为 1000 m。

正式测试使用一个进程连续编排三局。每局结束后在附件 GUI 启动下一案例；若服务会
短暂重启，可用 `--next-run-wait` 等待端口恢复。案例号和附件生成的加密日志路径按
局重复传入；程序只保存其原文件名，不移动或改名原日志：

```bash
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 \
  --robot-id "$CUMCM_ROBOT_ID" --formal --runs 3 --next-run-wait 300 \
  --case-code A001 --case-code A002 --case-code A003 \
  --official-log 模拟器原日志1.dat --official-log 模拟器原日志2.dat \
  --official-log 模拟器原日志3.dat
```

程序动作日志位于 `res/logs/problem3/*.jsonl`，逐步保存虚拟时间、位置、频道、动作、
耗时分解、原始请求/响应、20频道状态、检测概率、互信息、Q值、剩余时间估计和选取
理由。单局统计及多边形参数搜索表位于 `res/tables/problem3/`；正式三局另追加到
`formal_runs.csv`。每局结束还会在 `res/figures/problem3/` 生成主运动图；若通过
`--truth-file` 显式提供事后真值，还会绘制干扰源及独立清除特写。真值只在 `/exit`
后读取，绝不参与策略。主图使用灰、橙、紫、绿、红分别表示无信号、示向度、near、
清除成功和清除失败；重合动作在右侧时间轴展开，清除特写另图排版，避免遮住主图。

正式模拟器接口不返回源坐标时，不能虚构真值；此时主图仍包含完整轨迹和全部行为，
但明确标注“未提供可读真值”。连接失败会直接报错，绝不使用伪数据；动作超时属于执行状态不明，
客户端不自动重发，也不继续发新动作。只有人工确认服务器按相同 `request_id` 幂等
后，才可显式调用客户端的 `retry_last_ambiguous()`。

## 问题 1 实现与反例

每个示向度 `theta` 被转换为 `[theta-1°, theta+1°]` 的两个半平面约束，再与
半径 1800 m 的圆域正多边形近似求交。得到逆时针凸多边形后，旋转卡壳以 `O(m)`
时间求最大顶点距；同时用 `O(m²)` 暴力枚举作验证。

第三组测试是由题意中的三次测向真实交会产生的反例：

| 检测点 | 坐标 / m | 示向度 |
|---|---:|---:|
| S1 | (0, 0) | 22.856939° |
| S2 | (850, -850) | 65.446094° |
| S3 | (280, 1350) | 329.255358° |

交汇四边形顶点约为 `(1504.120, 650.510)`、`(1484.254, 604.940)`、
`(1493.638, 599.135)`、`(1552.621, 622.795)`。其直径约为 `70.660 m`；以直径
端点中点为圆心、`35.330 m` 为半径作圆时，另一个顶点位于圆外约 `4.010 m`，
因此题目所述直径圆不一定覆盖定位区域。

## 问题 2 实现

根据论文第 6 节，首次示向度区域离散为位置假设，并按未知接收半径服从
`U(1000,1500)` 得到的 `p(d)` 加权。第二检测点采用两阶段搜索：

1. 全域粗筛：排除第一检测点 5 m 邻域，并检查 1500 m 可达性、检测概率不低于
   0.8、加权交会角指标不低于 0.5。
2. 局部细化：先在粗筛可行点上求目标函数最优点，再在其附近计算细网格的期望
   对数直径缩减；在目标值相差不超过 5% 的近优点中选择离第一检测点最近者。

`problem2 --x X --y Y --bearing THETA` 可指定第一检测点位置与示向度。程序会输出
近优点集的总坐标包围范围、连通分量凸包及最佳第二检测点。这里的“区域”是细网格
离散结果的概括，不是连续优化得到的精确边界；其精度由 `REFINE_STEP_M` 控制。

当粗网格近优集合在首次扇形两侧形成多个连通分量时，程序分别保留每个分量的
最高分点作为细化中心，避免仅细化单侧峰值。结果同时区分“期望增益最大点”和
式 (59) 的“5%近优集合内距 `M1` 最近的最终点”，二者不要求重合。

### 与论文第 6 节的对应关系

当前实现采用论文给出的“期望对数直径增益”路线：式 (14) 构造首次可行域，式
(17)--(23) 构造离散后验与条件检测概率，式 (24)--(30) 完成粗筛，式 (31)--(38)
对首次扇形内**全部**离散位置概率加权求和，式 (52)--(54) 按“粗网格目标最优点
附近再细化”搜索，最后按式 (57)--(59) 取 5% 近优集合中离第一检测点最近的位置。
因此算法流程和目标函数与论文这一条路线一致；论文另列的互信息目标式 (39)--(51)
是可替代路线，当前程序没有同时实现它。

数值结果仍依赖离散近似：首次区域位置网格为 45 m，候选粗网格为 150 m，细网格
为 40 m、细化半径为 240 m；圆域用 720 边形近似。论文未明确给出测向误差的概率
密度，程序把 `-1°、0°、1°` 作为等权样本近似式 (37) 中的误差期望；初始候选集
`C^(0)` 取半径 1800 m 的目标圆域。这些均集中写在 `problems/problem2/config.py`，
人工复核或灵敏度分析时可直接调整。

选点完成后的直径分布验证另用 20 m 密网格，不改变上述选点结果。每个密网格位置
分别计算三个测向误差下的 `S2` 直径；柱状图按 `π_k q_2k` 加权，饼图按直径区间
汇总同一概率质量。

图中使用绿色实色表示 `0–1000 m` 必然接收区，橙色斜线环表示 `1000–1500 m`
概率过渡区，两者不会使用相同样式。

## 生成结果

运行问题 1 后生成：

- `res/tables/problem1_validation.csv`：三组输入、交汇顶点、两种直径算法结果和覆盖判定。
- `res/figures/problem1_three_station_intersection.png`：三站扇形细实线边界、交会全景、
  虚线边界特写及直径圆。
- `res/figures/problem1_rotating_calipers.png`：无坐标轴的旋转卡壳解析图。
- `res/figures/problem1_validation_cases.png`：三组测试的全局扇形布局，以及带虚线边界、
  直径和直径圆的逐组特写。
- `res/figures/problem1_diameter_circle_counterexample.png`：三站反例全景和局部覆盖判定。

运行问题 2 后生成：

- `res/tables/problem2_candidate_scores.csv`：粗筛和细化阶段的全部指标。
- `res/tables/problem2_excellent_regions.json`：输入参数、5% 近优区域范围、各连通分量
  凸包和最佳第二检测点。
- `res/tables/problem2_selected_m2_diameters.csv`：固定最终 `M2` 后，对 `S1` 中每个
  干扰源网格点及三个测向误差样本形成的 `S2` 多边形直径和积分权重。
- `res/figures/problem2_screening_principles.png`：四项粗筛原理；集合示意不显示坐标，
  接收概率与交会角质量保留必要的定量坐标。
- `res/figures/problem2_candidate_region.png`：后验位置、候选区、细化点及最终选择。
- `res/figures/problem2_selected_m2_diameter_histogram.png`：固定最终 `M2` 后，第二次
  测向所得多边形直径的检测概率加权柱状图和按数值区间汇总的饼图。

## 模拟器连接配置

真实参赛队号不能写入源文件或提交到仓库。运行问题 3、4 前设置：

```bash
export CUMCM_ROBOT_ID="<参赛队号>"
```

模拟器默认地址为 `http://127.0.0.1:2026`。仅在模拟器修改端口后覆盖：

```bash
export CUMCM_BASE_URL="http://127.0.0.1:<新端口>"
export CUMCM_HTTP_TIMEOUT_S="5"
```

## 推荐人工检查顺序

1. 对照题目和附件检查 `config/constants.py`。
2. 检查 `config/paths.py` 中输入与输出位置。
3. 检查 `config/simulator.py` 是否严格匹配附件 2 的协议字段。
4. 逐题检查 `problems/problemN/config.py`，确认没有混入其他问题参数。
5. 实现算法后，再检查各题 `main.py` 和 `src/radio_locator` 公共组件。

问题 3、4 正式测试前，还必须人工确认 `CUMCM_ROBOT_ID`、模拟器端口、动作串行性、
`request_id` 幂等重试，以及日志记录均正确。

## 测试与静态检查

```bash
PYTHONPATH=src:. MPLCONFIGDIR=/tmp/cumcm-mpl python -m pytest -q
python -m compileall -q config problems src tests main.py run.py
python -m ruff check config problems src tests main.py run.py
```

若当前 `mcm` 环境没有安装 `pytest` 或 `ruff`，对应命令会明确报缺少模块；不要把它
解释为测试通过。可先按 `environment.yml` 补齐开发工具，再重新运行。问题3测试覆盖
解析/数值覆盖、完整参数搜索、none粒子更新、固定 `R_j`、单频道隔离、动作计时、
clear频道不变、near立即清除、absent证明、最小覆盖圆及MPC保底路线不丢失。
