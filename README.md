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
│   └── runtime.py                    # Windows 绘图缓存、字体和无界面后端适配
├── scripts/attachment_protocol_demo.py # 附件接口与计时示例
├── reference/                        # B题原文、附件1和附件2
├── paper/                            # 当前方案论文
├── res/
│   ├── figures/                      # 生成的图
│   ├── tables/                       # 结果表、参数搜索和正式测试汇总
│   ├── logs/                         # 程序动作 JSONL 日志
│   └── simulator/                    # 本地模拟器真值，仅供复现核查
└── tests/                            # 回归测试
```

`config/constants.py` 只保存题面常量；问题 3 的可调算法参数位于 `config/problem3.yaml`。运行生成的内容都在 `res/`；不要让策略程序读取 `res/simulator/` 中的本地真值。

## 安装与检查

在 PowerShell 中：

```powershell
conda activate mcm
cd C:\Users\85375\Desktop\Code\code\Radio_Locater
python -m pip install -e .
python -m pytest -q
```

`pip install -e .` 后无需设置 `PYTHONPATH`，可用 `python main.py ...`、`python run.py ...` 或 `radio-locator ...` 运行。

## 问题 1、2

```powershell
# 问题1：交会定位验证、区域直径和论文插图
python main.py problem1

# 问题2：默认首个检测点的第二检测点选址
python main.py problem2

# 自定义首个检测点与示向度
python main.py problem2 --x -900 --y -500 --bearing 31.363757
```

图片写入 `res/figures/`，数据表写入 `res/tables/`。

## 问题 3 本地联调

先开一个 PowerShell 启动本地服务（不要关闭此窗口）：

```powershell
conda activate mcm
cd C:\Users\85375\Desktop\Code\code\Radio_Locater
python main.py simulator --problem 3 --seed 1 --robot-id demo
```

再开第二个 PowerShell 运行策略：

```powershell
conda activate mcm
cd C:\Users\85375\Desktop\Code\code\Radio_Locater
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 --robot-id demo --seed 1 --truth-file res\simulator\problem3_seed1.json
```

也可将 `belief_mpc` 改为 `robust_polygon_rolling`。`--truth-file` 只用于本地结束后的复盘绘图；官方客户端测试绝不能传入真值。

用附件的固定操作序列验证四个接口和计时规则：

```powershell
python scripts\attachment_protocol_demo.py --robot-id demo
```

它会执行 `/enter → /measure → /measure → /clear → /measure → /exit`，虚拟时刻应为 `0, 105, 111, 194, 199, 199`。仅在本地模拟器或官方演练中使用，不能用于正式测试。

停止本地服务请在服务窗口按 `Ctrl+C`。本地问题 4 场景可用 `--problem 4` 启动，但项目尚未实现问题 4 的自动策略。

## 官方客户端演练与正式测试

先在官方模拟器完成登录，并进入“问题 3 演练测试”或“问题 3 正式测试”。点击开始后等待倒计时结束，且界面明确显示接口已就绪，再运行程序。

```powershell
$env:CUMCM_ROBOT_ID = "你的参赛队号"
$env:CUMCM_BASE_URL = "http://127.0.0.1:2026"
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 --robot-id $env:CUMCM_ROBOT_ID
```

不要复制 PowerShell 提示符 `(mcm) PS ...>`，也不要输入 Markdown 转义后的 `belief\_mpc`；命令中应使用普通下划线 `belief_mpc`。

程序会串行调用 `/enter`、`/measure`、`/clear` 和 `/exit`。`accepted=false` 表示该动作未执行：检查官方客户端是否已就绪、当前登录队号是否一致、或当前局是否已经进入过。传输中断时程序会停止，不会自动重发可能已执行的动作。

正式测试前在 GUI 中记录案例编码；每局结束后从官方模拟器导出加密日志，保持原文件名。程序生成的明文动作日志、汇总表和轨迹图分别在 `res/logs/problem3/`、`res/tables/problem3/`、`res/figures/problem3/`。

连续编排三次正式测试：

```powershell
python run.py problem3 --strategy belief_mpc --host 127.0.0.1 --port 2026 --robot-id $env:CUMCM_ROBOT_ID --formal --runs 3 --next-run-wait 300 --case-code A001 --case-code A002 --case-code A003 --official-log "官方日志1.dat" --official-log "官方日志2.dat" --official-log "官方日志3.dat"
```

每局结束后仍需在官方 GUI 手动启动下一局；`--next-run-wait` 是程序等待接口重新开放的最长秒数。正式汇总追加至 `res/tables/problem3_formal_runs.csv`。

## 常用命令

```powershell
python run.py problem3 --help
python -m pytest -q
ruff check .
```

问题 3 常用调参为 `--grid-step`、`--particles`、`--candidate-limit`、`--horizon`、`--beam-width` 和 `--fixed-polygon`。正式测试应先在演练中确认默认或调整后的参数有效。
