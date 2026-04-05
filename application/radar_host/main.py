"""雷达上位机监测平台 — 程序入口。"""
from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from app.main_window import RadarMonitorWindow


def main() -> int:
    # 创建应用程序实例
    app = QApplication(sys.argv)
    app.setApplicationName("Radar Monitor")

    # 创建并显示主窗口
    window = RadarMonitorWindow()
    window.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
