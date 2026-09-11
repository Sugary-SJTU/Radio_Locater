"""项目根入口。

本文件只把 ``python main.py`` 转发到统一命令行入口，不包含模型、配置或业务逻辑。
安装环境后也可以直接运行 ``radio-locator``，两种方式行为一致。
"""

# 业务入口保存在 src 布局下的可安装包中；根文件只保留这一处转发，避免出现两套逻辑。
from radio_locator.cli import main


# 直接执行 ``python main.py`` 时调用入口；被测试或其他模块导入时不会自动运行。
if __name__ == "__main__":
    main()
