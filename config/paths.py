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
# PNG 与 PDF 使用镜像目录；绘图函数一次调用会同时写出两种格式。
PNG_FIGURES_DIR = FIGURES_DIR / "png"
PDF_FIGURES_DIR = FIGURES_DIR / "pdf"
# 论文与模型说明图按题号分目录保存；具体绘图函数在保存时创建这些目录。
PROBLEM1_FIGURES_DIR = PNG_FIGURES_DIR / "pic1"
PROBLEM2_FIGURES_DIR = PNG_FIGURES_DIR / "pic2"
PROBLEM3_FIGURES_DIR = PNG_FIGURES_DIR / "pic3"
PROBLEM4_FIGURES_DIR = PNG_FIGURES_DIR / "pic4"
# 联调时的机器狗轨迹属于运行回放资料，沿用既有专用目录，不与论文插图混放。
PROBLEM3_TRAJECTORY_FIGURES_DIR = PNG_FIGURES_DIR / "problem3"
PROBLEM4_TRAJECTORY_FIGURES_DIR = PNG_FIGURES_DIR / "problem4"
TABLES_DIR = RES_DIR / "tables"
# 这里保存程序自己的明文动作日志，不代替模拟器导出的加密正式日志。
LOGS_DIR = RES_DIR / "logs"
# 本地仿真案例真值保存于此，仅供演练和复现，不应被问题 3、4 策略读取。
SIMULATOR_CASES_DIR = RES_DIR / "simulator"


def paired_pdf_path(png_path: Path) -> Path:
    """返回 PNG 输出在 PDF 镜像目录中的对应路径。"""

    try:
        relative = png_path.resolve().relative_to(PNG_FIGURES_DIR.resolve())
    except ValueError:
        # 测试或外部调用可传入任意临时路径，此时将 PDF 放在 PNG 同目录。
        return png_path.with_suffix(".pdf")
    return (PDF_FIGURES_DIR / relative).with_suffix(".pdf")
