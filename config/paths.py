"""项目输入与输出目录定义。

所有路径均由当前文件位置推导，避免依赖启动命令所在目录。
本模块不创建目录，也不读写文件；具体算法负责在输出前创建所需目录。
"""

from pathlib import Path

# ``__file__`` 指向 <项目根目录>/config/paths.py。resolve() 将其规范化为
# 绝对路径，parents[1] 再向上两级得到项目根目录。这样程序不依赖启动目录。
PROJECT_ROOT = Path(__file__).resolve().parents[1]

# 只读参考资料：题目正文和两份模拟器附件。
REFERENCE_DIR = PROJECT_ROOT / "reference"
ATTACHMENTS_DIR = REFERENCE_DIR / "附件"
PROBLEM_PDF = REFERENCE_DIR / "B题.pdf"
# 附件 1 说明机器狗行为、模拟器操作流程和虚拟时间计算。
SIMULATOR_GUIDE = ATTACHMENTS_DIR / "附件1.docx"
# 附件 2 规定 HTTP+JSON 请求、响应、错误码和幂等要求。
API_GUIDE = ATTACHMENTS_DIR / "附件2.docx"
# 当前建模方案论文；问题 1、2 的算法实现以其中第 5、6 节为依据。
PAPER_PDF = PROJECT_ROOT / "paper" / "无线电干扰源的快速自动定位与清除.pdf"

# 可再生成结果统一放在 res 下，避免散落到配置或算法源码目录。
RES_DIR = PROJECT_ROOT / "res"
FIGURES_DIR = RES_DIR / "figures"
TABLES_DIR = RES_DIR / "tables"
# 这里保存程序自己的明文动作日志，不代替模拟器导出的加密正式日志。
LOGS_DIR = RES_DIR / "logs"
