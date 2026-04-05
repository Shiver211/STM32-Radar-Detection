from __future__ import annotations

import sys

from PySide6.QtWidgets import QApplication

from app.main_window import RadarMonitorWindow


def main() -> int:
    app = QApplication(sys.argv)
    app.setApplicationName("Radar Monitor")

    window = RadarMonitorWindow()
    window.show()

    return app.exec()


if __name__ == "__main__":
    raise SystemExit(main())
