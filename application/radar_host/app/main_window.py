"""雷达上位机主窗口：包含串口控制、实时曲线、历史记录等界面。"""
from __future__ import annotations

from datetime import datetime
import math
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import QDateTime, QTimer, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
    QLineEdit,
    QMainWindow,
    QMessageBox,
    QPlainTextEdit,
    QPushButton,
    QStackedWidget,
    QTableWidget,
    QTableWidgetItem,
    QVBoxLayout,
    QWidget,
)

from .models import MeasurementSession, TelemetryFrame
from .serial_client import SerialClient, parse_telemetry_line
from .storage import MeasurementStorage


class RadarMonitorWindow(QMainWindow):
    """主窗口：整合串口通信、波形绘制、测量控制和历史记录查询。"""

    def __init__(self) -> None:
        super().__init__()

        # 启用全局抗锯齿，让曲线更平滑
        pg.setConfigOptions(antialias=True)

        self.setWindowTitle("Radar 上位机监测平台")
        self.resize(1380, 860)

        # 核心组件：串口客户端、数据库存储、两个测量会话（心率/呼吸）
        self._serial_client = SerialClient()
        project_root = Path(__file__).resolve().parents[1]
        self._storage = MeasurementStorage(project_root / "data" / "measurement_history.db")

        self._heart_session = MeasurementSession(measure_type="HEART")
        self._breath_session = MeasurementSession(measure_type="BREATH")

        # 波形数据缓冲区
        self._max_points = 600                # 最多保留 600 个采样点
        self._scroll_window_points = 240      # 窗口内显示 240 个点
        self._x_values: list[float] = []
        self._heart_curve_values: list[float] = []
        self._breath_curve_values: list[float] = []
        self._heart_phase_curve_values: list[float] = []
        self._breath_phase_curve_values: list[float] = []
        self._sample_index = 0

        # 曲线平滑参数：alpha 越小越平滑，越大越跟手
        self._value_curve_smooth_alpha = 0.20
        self._phase_curve_smooth_alpha = 0.25

        # 平滑前值缓存
        self._heart_value_smooth_prev: float | None = None
        self._breath_value_smooth_prev: float | None = None
        self._heart_phase_smooth_prev: float | None = None
        self._breath_phase_smooth_prev: float | None = None

        self._current_heart_avg_text = "--"
        self._current_breath_avg_text = "--"

        # 调试计数器
        self._debug_rx_count = 0    # 收到的总行数
        self._debug_ok_count = 0    # 解析成功数
        self._debug_fail_count = 0  # 解析失败数

        # 构建界面
        self._build_ui()
        self._apply_styles()

        # 定时轮询串口数据（每 25ms 一次）
        self._serial_poll_timer = QTimer(self)
        self._serial_poll_timer.setInterval(25)
        self._serial_poll_timer.timeout.connect(self._poll_serial)
        self._serial_poll_timer.start()

        # 定时扫描可用串口（每 1.5s 一次）
        self._port_scan_timer = QTimer(self)
        self._port_scan_timer.setInterval(1500)
        self._port_scan_timer.timeout.connect(self._refresh_ports_keep_selection)
        self._port_scan_timer.start()

        self._refresh_ports_keep_selection()

    def closeEvent(self, event) -> None:
        """窗口关闭时断开串口。"""
        self._serial_client.close()
        super().closeEvent(event)

    # ───────────────────── 界面构建 ─────────────────────

    def _build_ui(self) -> None:
        """搭建整体布局：左侧控制面板 + 右侧波形图 + 底部历史表格。"""
        central = QWidget(self)
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(16, 14, 16, 16)
        root.setSpacing(12)

        # 顶部标题栏
        title_layout = QHBoxLayout()
        self._title_label = QLabel("信号监测")
        self._subtitle_label = QLabel("Heart & Breath Monitor")
        title_layout.addWidget(self._title_label)
        title_layout.addStretch(1)
        title_layout.addWidget(self._subtitle_label)
        root.addLayout(title_layout)

        # 主网格：左上控制面板，右上波形面板，下方历史面板
        main_grid = QGridLayout()
        main_grid.setHorizontalSpacing(12)
        main_grid.setVerticalSpacing(12)
        root.addLayout(main_grid, 1)

        self._control_group = self._build_control_panel()
        self._plot_group = self._build_plot_panel()
        self._history_group = self._build_history_panel()

        main_grid.addWidget(self._control_group, 0, 0)
        main_grid.addWidget(self._plot_group, 0, 1)
        main_grid.addWidget(self._history_group, 1, 0, 1, 2)
        main_grid.setColumnStretch(0, 0)
        main_grid.setColumnStretch(1, 1)
        main_grid.setRowStretch(0, 1)
        main_grid.setRowStretch(1, 0)

    def _build_control_panel(self) -> QGroupBox:
        """左侧控制面板：串口选择、连接/断开、测量启停、调试日志。"""
        group = QGroupBox("设备与测量控制")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(12, 16, 12, 12)
        layout.setSpacing(10)

        # 串口选择
        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)

        self._port_combo = QComboBox()
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["115200", "57600", "38400", "9600"])
        self._baud_combo.setCurrentText("115200")

        form.addRow("串口", self._port_combo)
        form.addRow("波特率", self._baud_combo)
        layout.addLayout(form)

        # 连接/断开按钮
        button_row = QHBoxLayout()
        self._refresh_btn = QPushButton("刷新串口")
        self._connect_btn = QPushButton("连接设备")
        self._disconnect_btn = QPushButton("断开连接")
        button_row.addWidget(self._refresh_btn)
        button_row.addWidget(self._connect_btn)
        button_row.addWidget(self._disconnect_btn)
        layout.addLayout(button_row)

        self._status_label = QLabel("状态：未连接")
        layout.addWidget(self._status_label)

        # 测量控制区
        measure_group = QGroupBox("测量项目")
        measure_layout = QGridLayout(measure_group)

        self._heart_start_btn = QPushButton("开始心率测量")
        self._heart_stop_btn = QPushButton("停止心率测量")
        self._breath_start_btn = QPushButton("开始呼吸测量")
        self._breath_stop_btn = QPushButton("停止呼吸测量")

        self._heart_avg_label = QLabel("心率平均值：--")
        self._breath_avg_label = QLabel("呼吸平均值：--")

        measure_layout.addWidget(self._heart_start_btn, 0, 0)
        measure_layout.addWidget(self._heart_stop_btn, 0, 1)
        measure_layout.addWidget(self._heart_avg_label, 1, 0, 1, 2)
        measure_layout.addWidget(self._breath_start_btn, 2, 0)
        measure_layout.addWidget(self._breath_stop_btn, 2, 1)
        measure_layout.addWidget(self._breath_avg_label, 3, 0, 1, 2)

        layout.addWidget(measure_group)

        # 调试日志区
        debug_group = QGroupBox("调试日志")
        debug_layout = QVBoxLayout(debug_group)

        debug_stats_row = QHBoxLayout()
        self._debug_rx_label = QLabel("RX: 0")
        self._debug_ok_label = QLabel("解析成功: 0")
        self._debug_fail_label = QLabel("解析失败: 0")
        debug_stats_row.addWidget(self._debug_rx_label)
        debug_stats_row.addWidget(self._debug_ok_label)
        debug_stats_row.addWidget(self._debug_fail_label)
        debug_stats_row.addStretch(1)

        debug_btn_row = QHBoxLayout()
        self._debug_clear_btn = QPushButton("清空日志")
        debug_btn_row.addStretch(1)
        debug_btn_row.addWidget(self._debug_clear_btn)

        self._debug_log = QPlainTextEdit()
        self._debug_log.setReadOnly(True)
        self._debug_log.setPlaceholderText("串口收发日志会显示在这里")
        self._debug_log.setMinimumHeight(180)
        self._debug_log.setMaximumBlockCount(600)

        debug_layout.addLayout(debug_stats_row)
        debug_layout.addLayout(debug_btn_row)
        debug_layout.addWidget(self._debug_log)

        layout.addWidget(debug_group)
        layout.addStretch(1)

        # 绑定按钮事件
        self._refresh_btn.clicked.connect(self._refresh_ports_keep_selection)
        self._connect_btn.clicked.connect(self._connect_serial)
        self._disconnect_btn.clicked.connect(self._disconnect_serial)
        self._heart_start_btn.clicked.connect(self._start_heart_measurement)
        self._heart_stop_btn.clicked.connect(self._stop_heart_measurement)
        self._breath_start_btn.clicked.connect(self._start_breath_measurement)
        self._breath_stop_btn.clicked.connect(self._stop_breath_measurement)
        self._debug_clear_btn.clicked.connect(self._clear_debug_log)

        self._update_debug_stats()

        return group

    def _build_plot_panel(self) -> QGroupBox:
        """右侧波形面板：数值曲线和相位曲线两个页面可切换。"""
        group = QGroupBox("实时曲线")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 14, 10, 10)

        # 页面切换按钮
        switch_row = QHBoxLayout()
        self._switch_value_page_btn = QPushButton("数值曲线页")
        self._switch_phase_page_btn = QPushButton("相位曲线页")
        self._clear_plot_btn = QPushButton("清空曲线")
        self._switch_value_page_btn.setCheckable(True)
        self._switch_phase_page_btn.setCheckable(True)
        switch_row.addWidget(self._switch_value_page_btn)
        switch_row.addWidget(self._switch_phase_page_btn)
        switch_row.addWidget(self._clear_plot_btn)
        switch_row.addStretch(1)
        layout.addLayout(switch_row)

        # 两个页面用堆叠控件管理，同一时间只显示一个
        self._plot_stack = QStackedWidget()

        # 数值曲线页
        self._value_plot_widget = pg.PlotWidget()
        self._value_plot_widget.setBackground((248, 248, 245))
        self._value_plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self._value_plot_widget.setLabel("left", "数值")
        self._value_plot_widget.setLabel("bottom", "采样点")
        self._value_heart_curve = self._value_plot_widget.plot(
            pen=pg.mkPen(color=(204, 77, 74), width=2),
            name="心率曲线",
        )
        self._value_breath_curve = self._value_plot_widget.plot(
            pen=pg.mkPen(color=(35, 119, 92), width=2),
            name="呼吸曲线",
        )
        value_legend = self._value_plot_widget.addLegend(offset=(12, 12))
        value_legend.addItem(self._value_heart_curve, "心率")
        value_legend.addItem(self._value_breath_curve, "呼吸")

        value_page = QWidget()
        value_layout = QVBoxLayout(value_page)
        value_layout.setContentsMargins(0, 0, 0, 0)
        value_layout.addWidget(self._value_plot_widget)

        # 相位曲线页
        self._phase_plot_widget = pg.PlotWidget()
        self._phase_plot_widget.setBackground((248, 248, 245))
        self._phase_plot_widget.showGrid(x=True, y=True, alpha=0.25)
        self._phase_plot_widget.setLabel("left", "相位")
        self._phase_plot_widget.setLabel("bottom", "采样点")
        self._phase_heart_curve = self._phase_plot_widget.plot(
            pen=pg.mkPen(color=(204, 77, 74), width=2),
            name="心率相位",
        )
        self._phase_breath_curve = self._phase_plot_widget.plot(
            pen=pg.mkPen(color=(35, 119, 92), width=2),
            name="呼吸相位",
        )
        phase_legend = self._phase_plot_widget.addLegend(offset=(12, 12))
        phase_legend.addItem(self._phase_heart_curve, "心率相位")
        phase_legend.addItem(self._phase_breath_curve, "呼吸相位")

        phase_page = QWidget()
        phase_layout = QVBoxLayout(phase_page)
        phase_layout.setContentsMargins(0, 0, 0, 0)
        phase_layout.addWidget(self._phase_plot_widget)

        self._plot_stack.addWidget(value_page)
        self._plot_stack.addWidget(phase_page)
        layout.addWidget(self._plot_stack)

        self._switch_value_page_btn.clicked.connect(self._show_value_plot_page)
        self._switch_phase_page_btn.clicked.connect(self._show_phase_plot_page)
        self._clear_plot_btn.clicked.connect(self._clear_plot_data)
        self._show_value_plot_page()

        # 底部实时数值显示
        info_row = QHBoxLayout()
        self._realtime_heart_label = QLabel("心率当前值：--")
        self._realtime_breath_label = QLabel("呼吸当前值：--")
        self._distance_label = QLabel("距离：--")
        info_row.addWidget(self._realtime_heart_label)
        info_row.addWidget(self._realtime_breath_label)
        info_row.addWidget(self._distance_label)
        info_row.addStretch(1)
        layout.addLayout(info_row)

        return group

    def _build_history_panel(self) -> QGroupBox:
        """底部历史记录面板：按时间/类型查询已保存的测量记录。"""
        group = QGroupBox("历史记录查询")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 14, 10, 10)

        # 查询条件栏
        filter_row = QHBoxLayout()

        now_dt = QDateTime.currentDateTime()
        self._query_start_dt = QLineEdit(now_dt.addDays(-7).toString("yyyy-MM-dd"))
        self._query_end_dt = QLineEdit(now_dt.toString("yyyy-MM-dd"))
        self._query_start_dt.setPlaceholderText("yyyy-MM-dd")
        self._query_end_dt.setPlaceholderText("yyyy-MM-dd")

        self._query_type_combo = QComboBox()
        self._query_type_combo.addItems(["ALL", "HEART", "BREATH"])

        self._query_btn = QPushButton("按时间查询")

        filter_row.addWidget(QLabel("起始"))
        filter_row.addWidget(self._query_start_dt)
        filter_row.addWidget(QLabel("结束"))
        filter_row.addWidget(self._query_end_dt)
        filter_row.addWidget(QLabel("类型"))
        filter_row.addWidget(self._query_type_combo)
        filter_row.addWidget(self._query_btn)
        filter_row.addStretch(1)

        layout.addLayout(filter_row)

        # 结果表格
        self._history_table = QTableWidget(0, 7)
        self._history_table.setHorizontalHeaderLabels(
            ["ID", "类型", "开始时间", "结束时间", "平均值", "样本数", "串口"]
        )
        self._history_table.verticalHeader().setVisible(False)
        self._history_table.setSelectionBehavior(QTableWidget.SelectRows)
        self._history_table.setEditTriggers(QTableWidget.NoEditTriggers)
        self._history_table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)

        layout.addWidget(self._history_table)

        self._query_btn.clicked.connect(self._query_history)

        return group

    def _apply_styles(self) -> None:
        """应用全局样式：配色、字体、圆角等。"""
        self.setStyleSheet(
            """
            QWidget {
                background-color: #f4f1ea;
                color: #222222;
                font-family: 'Microsoft YaHei UI';
                font-size: 14px;
            }
            QMainWindow {
                background: qlineargradient(
                    x1:0, y1:0, x2:1, y2:1,
                    stop:0 #f8f5ee,
                    stop:0.55 #ece6da,
                    stop:1 #e2d8c8
                );
            }
            QGroupBox {
                border: 1px solid #d5cab8;
                border-radius: 14px;
                margin-top: 8px;
                padding-top: 12px;
                background-color: rgba(255, 255, 255, 0.78);
                font-weight: 600;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 12px;
                padding: 0 6px;
                color: #3a2f1f;
            }
            QPushButton {
                background-color: #c89b3c;
                border: none;
                border-radius: 9px;
                padding: 8px 12px;
                color: #ffffff;
                font-weight: 600;
            }
            QPushButton:hover {
                background-color: #b48931;
            }
            QPushButton:pressed {
                background-color: #946d23;
            }
            QComboBox, QLineEdit {
                background-color: #ffffff;
                border: 1px solid #cabba6;
                border-radius: 8px;
                padding: 4px 8px;
                min-height: 28px;
            }
            QLabel {
                background: transparent;
            }
            QTableWidget {
                background-color: #fffcf7;
                border: 1px solid #d5cab8;
                border-radius: 10px;
                gridline-color: #eadfce;
            }
            QHeaderView::section {
                background-color: #efdfc6;
                color: #2a1f11;
                border: none;
                padding: 6px;
                font-weight: 700;
            }
            """
        )

        title_font = QFont("Microsoft YaHei UI", 24, QFont.Bold)
        sub_font = QFont("Microsoft YaHei UI", 12)
        self._title_label.setFont(title_font)
        self._subtitle_label.setFont(sub_font)
        self._subtitle_label.setStyleSheet("color: #6f5a3f;")

    # ───────────────────── 串口操作 ─────────────────────

    def _refresh_ports_keep_selection(self) -> None:
        """刷新串口列表，尽量保持当前选中的端口不变。"""
        previous = self._port_combo.currentData()
        ports = self._serial_client.list_available_ports()

        self._port_combo.blockSignals(True)
        self._port_combo.clear()

        for port_name, desc in ports:
            self._port_combo.addItem(f"{port_name} | {desc}", port_name)

        if ports:
            idx = 0
            for i in range(self._port_combo.count()):
                if self._port_combo.itemData(i) == previous:
                    idx = i
                    break
            self._port_combo.setCurrentIndex(idx)
        self._port_combo.blockSignals(False)

        if not ports:
            self._status_label.setText("状态：未检测到串口设备")
        elif not self._serial_client.is_open:
            self._status_label.setText(f"状态：发现 {len(ports)} 个串口设备")

    def _connect_serial(self) -> None:
        """连接选中的串口。"""
        port = self._port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "连接失败", "请选择可用串口")
            return

        try:
            self._serial_client.open(port=str(port), baudrate=int(self._baud_combo.currentText()))
        except Exception as exc:
            QMessageBox.critical(self, "连接失败", f"无法连接串口：{exc}")
            return

        self._status_label.setText(f"状态：已连接 {self._serial_client.port_name}")
        self._append_debug_line(f"串口已连接：{self._serial_client.port_name} @ {self._baud_combo.currentText()}")

    def _disconnect_serial(self) -> None:
        """断开串口连接。"""
        self._append_debug_line("串口断开")
        self._serial_client.close()
        self._status_label.setText("状态：未连接")

    # ───────────────────── 测量控制 ─────────────────────

    def _start_heart_measurement(self) -> None:
        """开始心率测量。"""
        if not self._serial_client.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口设备")
            return

        self._heart_value_smooth_prev = None
        self._heart_phase_smooth_prev = None
        self._heart_session.start()
        self._status_label.setText("状态：心率测量中")

    def _stop_heart_measurement(self) -> None:
        """停止心率测量并保存结果。"""
        self._finalize_measurement("HEART")

    def _start_breath_measurement(self) -> None:
        """开始呼吸测量。"""
        if not self._serial_client.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口设备")
            return

        self._breath_value_smooth_prev = None
        self._breath_phase_smooth_prev = None
        self._breath_session.start()
        self._status_label.setText("状态：呼吸测量中")

    def _stop_breath_measurement(self) -> None:
        """停止呼吸测量并保存结果。"""
        self._finalize_measurement("BREATH")

    def _finalize_measurement(self, measure_type: str) -> None:
        """结束一次测量：计算平均值、保存到数据库、更新界面。"""
        session = self._heart_session if measure_type == "HEART" else self._breath_session
        result = session.stop()
        if result is None:
            return

        avg_text = "--" if result.average_value is None else f"{result.average_value:.2f}"

        if measure_type == "HEART":
            self._current_heart_avg_text = avg_text
            self._heart_avg_label.setText(f"心率平均值：{avg_text}")
            self._heart_value_smooth_prev = None
            self._heart_phase_smooth_prev = None
        else:
            self._current_breath_avg_text = avg_text
            self._breath_avg_label.setText(f"呼吸平均值：{avg_text}")
            self._breath_value_smooth_prev = None
            self._breath_phase_smooth_prev = None

        # 存入数据库
        self._storage.save_record(
            measure_type=result.measure_type,
            start_time=result.start_time.strftime("%Y-%m-%d %H:%M:%S"),
            end_time=result.end_time.strftime("%Y-%m-%d %H:%M:%S"),
            average_value=result.average_value,
            sample_count=result.sample_count,
            port_name=self._serial_client.port_name or "DISCONNECTED",
        )

        self._status_label.setText(
            f"状态：{measure_type} 测量结束，平均值 {avg_text}，样本 {result.sample_count}"
        )

    # ───────────────────── 数据接收与处理 ─────────────────────

    def _poll_serial(self) -> None:
        """定时检查串口，读取新数据并逐行处理。"""
        if not self._serial_client.is_open:
            return

        lines = self._serial_client.read_lines()
        if not lines:
            return

        for line in lines:
            self._debug_rx_count += 1
            self._append_debug_line(f"RX< {line}")
            self._process_line(line)

        self._update_debug_stats()

    def _process_line(self, line: str) -> None:
        """解析一行数据，成功则更新波形和数值。"""
        frame = parse_telemetry_line(line)
        if frame is None:
            self._debug_fail_count += 1
            return

        self._debug_ok_count += 1
        self._apply_frame(frame)

    def _apply_frame(self, frame: TelemetryFrame) -> None:
        """将一帧数据应用到曲线、标签和测量会话。"""
        heart_display_enabled = self._heart_session.active and frame.heart_enabled
        breath_display_enabled = self._breath_session.active and frame.breath_enabled

        # 两个测量都没开，只显示距离
        if (not heart_display_enabled) and (not breath_display_enabled):
            self._heart_value_smooth_prev = None
            self._heart_phase_smooth_prev = None
            self._breath_value_smooth_prev = None
            self._breath_phase_smooth_prev = None
            self._realtime_heart_label.setText("心率当前值：--")
            self._realtime_breath_label.setText("呼吸当前值：--")
            dist_text = f"{frame.distance_cm:.2f} cm" if frame.range_valid else "无效"
            self._distance_label.setText(f"距离：{dist_text}")
            return

        # 根据测量状态决定显示值（未开启的显示为空）
        if heart_display_enabled:
            heart_plot_value = max(frame.heart_rate, 0.0)
            heart_phase_plot_value = frame.heart_phase
        else:
            heart_plot_value = float("nan")
            heart_phase_plot_value = float("nan")
            self._heart_value_smooth_prev = None
            self._heart_phase_smooth_prev = None

        if breath_display_enabled:
            breath_plot_value = max(frame.breath_rate, 0.0)
            breath_phase_plot_value = frame.breath_phase
        else:
            breath_plot_value = float("nan")
            breath_phase_plot_value = float("nan")
            self._breath_value_smooth_prev = None
            self._breath_phase_smooth_prev = None

        # 平滑处理：让曲线过渡更自然
        heart_plot_value, self._heart_value_smooth_prev = self._smooth_point(
            heart_plot_value,
            self._heart_value_smooth_prev,
            self._value_curve_smooth_alpha,
        )
        breath_plot_value, self._breath_value_smooth_prev = self._smooth_point(
            breath_plot_value,
            self._breath_value_smooth_prev,
            self._value_curve_smooth_alpha,
        )
        heart_phase_plot_value, self._heart_phase_smooth_prev = self._smooth_point(
            heart_phase_plot_value,
            self._heart_phase_smooth_prev,
            self._phase_curve_smooth_alpha,
        )
        breath_phase_plot_value, self._breath_phase_smooth_prev = self._smooth_point(
            breath_phase_plot_value,
            self._breath_phase_smooth_prev,
            self._phase_curve_smooth_alpha,
        )

        # 追加到数据缓冲区
        self._sample_index += 1
        self._x_values.append(float(self._sample_index))
        self._heart_curve_values.append(heart_plot_value)
        self._breath_curve_values.append(breath_plot_value)
        self._heart_phase_curve_values.append(heart_phase_plot_value)
        self._breath_phase_curve_values.append(breath_phase_plot_value)

        # 超出上限则丢弃最旧的数据
        if len(self._x_values) > self._max_points:
            self._x_values = self._x_values[-self._max_points :]
            self._heart_curve_values = self._heart_curve_values[-self._max_points :]
            self._breath_curve_values = self._breath_curve_values[-self._max_points :]
            self._heart_phase_curve_values = self._heart_phase_curve_values[-self._max_points :]
            self._breath_phase_curve_values = self._breath_phase_curve_values[-self._max_points :]

        # 刷新四条曲线
        self._value_heart_curve.setData(self._x_values, self._heart_curve_values, connect="finite")
        self._value_breath_curve.setData(self._x_values, self._breath_curve_values, connect="finite")
        self._phase_heart_curve.setData(self._x_values, self._heart_phase_curve_values, connect="finite")
        self._phase_breath_curve.setData(self._x_values, self._breath_phase_curve_values, connect="finite")
        self._update_plot_ranges()

        # 更新实时数值标签
        if heart_display_enabled:
            self._realtime_heart_label.setText(f"心率当前值：{frame.heart_rate:.2f}")
        else:
            self._realtime_heart_label.setText("心率当前值：--")

        if breath_display_enabled:
            self._realtime_breath_label.setText(f"呼吸当前值：{frame.breath_rate:.2f}")
        else:
            self._realtime_breath_label.setText("呼吸当前值：--")

        dist_text = f"{frame.distance_cm:.2f} cm" if frame.range_valid else "无效"
        self._distance_label.setText(f"距离：{dist_text}")

        # 采集样本到测量会话
        if heart_display_enabled and frame.heart_rate > 0:
            self._heart_session.add_sample(frame.heart_rate)

        if breath_display_enabled and frame.breath_rate > 0:
            self._breath_session.add_sample(frame.breath_rate)

    # ───────────────────── 历史记录 ─────────────────────

    def _query_history(self) -> None:
        """根据用户选择的时间范围和类型查询历史记录。"""
        start_text = self._query_start_dt.text().strip()
        end_text = self._query_end_dt.text().strip()

        try:
            start_date = datetime.strptime(start_text, "%Y-%m-%d").date()
            end_date = datetime.strptime(end_text, "%Y-%m-%d").date()
        except ValueError:
            QMessageBox.warning(self, "查询失败", "日期格式应为 yyyy-MM-dd")
            return

        if end_date < start_date:
            QMessageBox.warning(self, "查询失败", "结束时间不能早于起始时间")
            return

        start_time_text = f"{start_date:%Y-%m-%d} 00:00:00"
        end_time_text = f"{end_date:%Y-%m-%d} 23:59:59"

        measure_type = self._query_type_combo.currentText()
        rows = self._storage.query_records(
            start_time=start_time_text,
            end_time=end_time_text,
            measure_type=measure_type,
        )

        self._history_table.setRowCount(len(rows))
        for row_index, row_data in enumerate(rows):
            avg_text = "--" if row_data.average_value is None else f"{row_data.average_value:.2f}"
            cells = [
                str(row_data.row_id),
                row_data.measure_type,
                row_data.start_time,
                row_data.end_time,
                avg_text,
                str(row_data.sample_count),
                row_data.port_name,
            ]
            for col, text in enumerate(cells):
                item = QTableWidgetItem(text)
                if col in (0, 4, 5):
                    item.setTextAlignment(Qt.AlignCenter)
                self._history_table.setItem(row_index, col, item)

        self._status_label.setText(f"状态：查询到 {len(rows)} 条记录")

    def _force_stop_all_sessions(self) -> None:
        """强制停止所有正在进行的测量。"""
        self._finalize_measurement("HEART")
        self._finalize_measurement("BREATH")

    # ───────────────────── 辅助方法 ─────────────────────

    @staticmethod
    def _finite_values(values: list[float]) -> list[float]:
        """过滤出有效的数值（排除空值）。"""
        return [v for v in values if math.isfinite(v)]

    @staticmethod
    def _smooth_point(raw_value: float, prev_value: float | None, alpha: float) -> tuple[float, float | None]:
        """对单个数据点做平滑：新值 = alpha × 当前值 + (1-alpha) × 前值。"""
        if not math.isfinite(raw_value):
            return float("nan"), None

        if (prev_value is None) or (not math.isfinite(prev_value)):
            smoothed = raw_value
        else:
            smoothed = (alpha * raw_value) + ((1.0 - alpha) * prev_value)

        return smoothed, smoothed

    def _update_plot_ranges(self) -> None:
        """自动调整坐标轴的显示范围，让曲线始终在可视区域内。"""
        if not self._x_values:
            return

        right = self._x_values[-1]
        left = max(1.0, right - float(self._scroll_window_points) + 1.0)
        if right <= left:
            right = left + 1.0

        self._value_plot_widget.setXRange(left, right, padding=0.0)
        self._phase_plot_widget.setXRange(left, right, padding=0.0)

        begin = max(0, len(self._x_values) - self._scroll_window_points)

        # 数值曲线 Y 轴
        value_visible = self._finite_values(self._heart_curve_values[begin:])
        value_visible.extend(self._finite_values(self._breath_curve_values[begin:]))

        if value_visible:
            value_max = max(1.0, max(value_visible))
            self._value_plot_widget.setYRange(0.0, value_max * 1.15 + 0.5, padding=0.0)
        else:
            self._value_plot_widget.setYRange(0.0, 10.0, padding=0.0)

        # 相位曲线 Y 轴
        phase_visible = self._finite_values(self._heart_phase_curve_values[begin:])
        phase_visible.extend(self._finite_values(self._breath_phase_curve_values[begin:]))

        if phase_visible:
            phase_min = min(phase_visible)
            phase_max = max(phase_visible)
            if abs(phase_max - phase_min) < 1e-3:
                phase_min -= 1.0
                phase_max += 1.0
            phase_margin = (phase_max - phase_min) * 0.15
            self._phase_plot_widget.setYRange(phase_min - phase_margin, phase_max + phase_margin, padding=0.0)
        else:
            self._phase_plot_widget.setYRange(-1.0, 1.0, padding=0.0)

    def _show_value_plot_page(self) -> None:
        """切换到数值曲线页。"""
        self._plot_stack.setCurrentIndex(0)
        self._switch_value_page_btn.setChecked(True)
        self._switch_phase_page_btn.setChecked(False)

    def _show_phase_plot_page(self) -> None:
        """切换到相位曲线页。"""
        self._plot_stack.setCurrentIndex(1)
        self._switch_value_page_btn.setChecked(False)
        self._switch_phase_page_btn.setChecked(True)

    def _update_debug_stats(self) -> None:
        """更新调试计数器的显示。"""
        self._debug_rx_label.setText(f"RX: {self._debug_rx_count}")
        self._debug_ok_label.setText(f"解析成功: {self._debug_ok_count}")
        self._debug_fail_label.setText(f"解析失败: {self._debug_fail_count}")

    def _append_debug_line(self, text: str) -> None:
        """向日志窗口追加一行带时间戳的文本。"""
        ts = QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        self._debug_log.appendPlainText(f"[{ts}] {text}")

    def _clear_debug_log(self) -> None:
        """清空调试日志。"""
        self._debug_log.clear()
        self._append_debug_line("日志已清空")

    def _clear_plot_data(self) -> None:
        """清空所有曲线缓存，并把坐标轴恢复到初始显示范围。"""
        self._x_values.clear()
        self._heart_curve_values.clear()
        self._breath_curve_values.clear()
        self._heart_phase_curve_values.clear()
        self._breath_phase_curve_values.clear()
        self._sample_index = 0

        self._heart_value_smooth_prev = None
        self._breath_value_smooth_prev = None
        self._heart_phase_smooth_prev = None
        self._breath_phase_smooth_prev = None

        self._value_heart_curve.setData([], [])
        self._value_breath_curve.setData([], [])
        self._phase_heart_curve.setData([], [])
        self._phase_breath_curve.setData([], [])

        self._value_plot_widget.setXRange(1.0, float(self._scroll_window_points), padding=0.0)
        self._phase_plot_widget.setXRange(1.0, float(self._scroll_window_points), padding=0.0)
        self._value_plot_widget.setYRange(0.0, 10.0, padding=0.0)
        self._phase_plot_widget.setYRange(-1.0, 1.0, padding=0.0)

        self._realtime_heart_label.setText("心率当前值：--")
        self._realtime_breath_label.setText("呼吸当前值：--")
        self._distance_label.setText("距离：--")
        self._append_debug_line("曲线已清空")

    def keyPressEvent(self, event) -> None:
        """按键事件：按 Esc 停止所有测量。"""
        if event.key() == Qt.Key_Escape:
            self._force_stop_all_sessions()
            self._status_label.setText("状态：已停止全部测量")
            return
        super().keyPressEvent(event)
