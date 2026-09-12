# 无线电干扰源自动定位与清除

2026 年数学建模竞赛 B 题项目。已实现问题 1、2 的数值模型，问题 3 的两种在线策略，及问题 3、4 共用的本地 HTTP 模拟器。问题 4 的自动搜索策略仍待实现。

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
│   │   ├── pic1/ ... pic4/           # 问题 1--4 的论文与模型说明图
│   │   └── problem3/、problem4/       # 机器狗运行回放轨迹（固定原路径）
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

问题 1、2 的图片分别写入 `res/figures/pic1/`、`res/figures/pic2/`；问题 3、4
后续论文图片分别使用 `pic3/`、`pic4/`。机器狗运行测试的轨迹图仍固定写入
`res/figures/problem3/`、`res/figures/problem4/`，数据表写入 `res/tables/`。

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

停止本地服务请在服务窗口按 `Ctrl+C`。本地问题 4 场景可用 `--problem 4` 启动，但项目尚未实现问题 4 的自动策略。

## 官方客户端演练与正式测试

先在官方模拟器完成登录，并进入“问题 3 演练测试”或“问题 3 正式测试”。点击开始后等待倒计时结束，且界面明确显示接口已就绪，再运行程序。

```bash
export CUMCM_ROBOT_ID="你的参赛队号"
export CUMCM_BASE_URL="http://127.0.0.1:2026"
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 --robot-id "$CUMCM_ROBOT_ID"
```

不要复制 shell 提示符，也不要输入 Markdown 转义后的 `belief\_mpc`；命令中应使用普通下划线 `belief_mpc`。

程序会串行调用 `/enter`、`/measure`、`/clear` 和 `/exit`。`accepted=false` 表示该动作未执行：检查官方客户端是否已就绪、当前登录队号是否一致、或当前局是否已经进入过。传输中断时程序会停止，不会自动重发可能已执行的动作。

正式测试前在 GUI 中记录案例编码；每局结束后从官方模拟器导出加密日志，保持原文件名。程序生成的明文动作日志、汇总表和机器狗轨迹图分别在 `res/logs/problem3/`、`res/tables/problem3/`、`res/figures/problem3/`；论文插图另存入 `res/figures/pic3/`。

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

沿用原模拟器队号；本地demo可将机器人编号改为`demo`。另有`cooperative_bearing_tour`作为共享测站后集中清除的对照方案。两种旧策略仍可使用。

2026-09-12参数更新：联合巡回默认`tour_polygon_radius_m: 1150.0`，仍保留原点扫描。60场景配对中平均总时间由3800.28秒降至3699.96秒；51局更快、9局更慢。新增`--tour-radius 1200`可恢复旧半径，`--channel-order alternating`可试验相邻非空扫描批次升降序交替，默认`legacy`优先当前频道。交替顺序在本地试验中平均增加约2秒，故未启用为默认。完整覆盖证明与分组数据见方法文档第14节。

末段绕行修订：默认`tour_endgame_mode: probe`，在最多剩2个覆盖节点时让已发现的粗定位目标参加排路，并先做一次侧向共享补测。候选节点≤10时使用Held–Karp精确开放路线。`--endgame legacy`可恢复原模式，`--endgame exact`只启用小规模精确排路。120个配对场景中新默认均值3528.87秒，旧模式3717.74秒，83局改善；不保证每局变快。P5真实历史状态复盘见方法文档第15节。
