from __future__ import annotations

import math
from pathlib import Path

import pyqtgraph as pg
from PySide6.QtCore import QDateTime, QTimer, Qt
from PySide6.QtGui import QFont
from PySide6.QtWidgets import (
    QComboBox,
    QDateTimeEdit,
    QFormLayout,
    QGridLayout,
    QGroupBox,
    QHBoxLayout,
    QHeaderView,
    QLabel,
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
    def __init__(self) -> None:
        super().__init__()

        # 启用全局抗锯齿，降低折线边缘毛刺。
        pg.setConfigOptions(antialias=True)

        self.setWindowTitle("Radar 上位机监测平台")
        self.resize(1380, 860)

        self._serial_client = SerialClient()
        project_root = Path(__file__).resolve().parents[1]
        self._storage = MeasurementStorage(project_root / "data" / "measurement_history.db")

        self._heart_session = MeasurementSession(measure_type="HEART")
        self._breath_session = MeasurementSession(measure_type="BREATH")

        self._max_points = 600
        self._scroll_window_points = 240
        self._x_values: list[float] = []
        self._heart_curve_values: list[float] = []
        self._breath_curve_values: list[float] = []
        self._heart_phase_curve_values: list[float] = []
        self._breath_phase_curve_values: list[float] = []
        self._sample_index = 0

        # 曲线平滑参数：alpha 越小越平滑，越大越跟手。
        self._value_curve_smooth_alpha = 0.20
        self._phase_curve_smooth_alpha = 0.25

        self._heart_value_smooth_prev: float | None = None
        self._breath_value_smooth_prev: float | None = None
        self._heart_phase_smooth_prev: float | None = None
        self._breath_phase_smooth_prev: float | None = None

        self._current_heart_avg_text = "--"
        self._current_breath_avg_text = "--"

        # 串口调试计数器：用于快速定位“未收到/未解析/未绘制”。
        self._debug_rx_count = 0
        self._debug_ok_count = 0
        self._debug_fail_count = 0

        self._build_ui()
        self._apply_styles()

        self._serial_poll_timer = QTimer(self)
        self._serial_poll_timer.setInterval(25)
        self._serial_poll_timer.timeout.connect(self._poll_serial)
        self._serial_poll_timer.start()

        self._port_scan_timer = QTimer(self)
        self._port_scan_timer.setInterval(1500)
        self._port_scan_timer.timeout.connect(self._refresh_ports_keep_selection)
        self._port_scan_timer.start()

        self._refresh_ports_keep_selection()

    def closeEvent(self, event) -> None:  # type: ignore[override]
        self._serial_client.close()
        super().closeEvent(event)

    def _build_ui(self) -> None:
        central = QWidget(self)
        self.setCentralWidget(central)

        root = QVBoxLayout(central)
        root.setContentsMargins(16, 14, 16, 16)
        root.setSpacing(12)

        title_layout = QHBoxLayout()
        self._title_label = QLabel("生理信号实时监测")
        self._subtitle_label = QLabel("Heart & Breath Monitor")
        title_layout.addWidget(self._title_label)
        title_layout.addStretch(1)
        title_layout.addWidget(self._subtitle_label)
        root.addLayout(title_layout)

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
        group = QGroupBox("设备与测量控制")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(12, 16, 12, 12)
        layout.setSpacing(10)

        form = QFormLayout()
        form.setLabelAlignment(Qt.AlignRight)

        self._port_combo = QComboBox()
        self._baud_combo = QComboBox()
        self._baud_combo.addItems(["115200", "57600", "38400", "9600"])
        self._baud_combo.setCurrentText("115200")

        form.addRow("串口", self._port_combo)
        form.addRow("波特率", self._baud_combo)
        layout.addLayout(form)

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

        measure_group = QGroupBox("独立测量控制")
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

        debug_group = QGroupBox("串口调试")
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
        group = QGroupBox("实时曲线")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 14, 10, 10)

        switch_row = QHBoxLayout()
        self._switch_value_page_btn = QPushButton("数值曲线页")
        self._switch_phase_page_btn = QPushButton("相位曲线页")
        self._switch_value_page_btn.setCheckable(True)
        self._switch_phase_page_btn.setCheckable(True)
        switch_row.addWidget(self._switch_value_page_btn)
        switch_row.addWidget(self._switch_phase_page_btn)
        switch_row.addStretch(1)
        layout.addLayout(switch_row)

        self._plot_stack = QStackedWidget()

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
        self._show_value_plot_page()

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
        group = QGroupBox("历史记录查询")
        layout = QVBoxLayout(group)
        layout.setContentsMargins(10, 14, 10, 10)

        filter_row = QHBoxLayout()

        now = QDateTime.currentDateTime()
        self._query_start_dt = QDateTimeEdit(now.addDays(-7))
        self._query_end_dt = QDateTimeEdit(now)
        self._query_start_dt.setDisplayFormat("yyyy-MM-dd HH:mm:ss")
        self._query_end_dt.setDisplayFormat("yyyy-MM-dd HH:mm:ss")
        self._query_start_dt.setCalendarPopup(True)
        self._query_end_dt.setCalendarPopup(True)

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
            QComboBox, QDateTimeEdit {
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

    def _refresh_ports_keep_selection(self) -> None:
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
        port = self._port_combo.currentData()
        if not port:
            QMessageBox.warning(self, "连接失败", "请选择可用串口")
            return

        try:
            self._serial_client.open(port=str(port), baudrate=int(self._baud_combo.currentText()))
        except Exception as exc:  # noqa: BLE001
            QMessageBox.critical(self, "连接失败", f"无法连接串口：{exc}")
            return

        self._status_label.setText(f"状态：已连接 {self._serial_client.port_name}")
        self._append_debug_line(f"串口已连接：{self._serial_client.port_name} @ {self._baud_combo.currentText()}")

    def _disconnect_serial(self) -> None:
        self._append_debug_line("串口断开")
        self._serial_client.close()
        self._status_label.setText("状态：未连接")

    def _start_heart_measurement(self) -> None:
        if not self._serial_client.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口设备")
            return

        self._heart_value_smooth_prev = None
        self._heart_phase_smooth_prev = None
        self._heart_session.start()
        self._status_label.setText("状态：心率测量中")

    def _stop_heart_measurement(self) -> None:
        self._finalize_measurement("HEART")

    def _start_breath_measurement(self) -> None:
        if not self._serial_client.is_open:
            QMessageBox.warning(self, "提示", "请先连接串口设备")
            return

        self._breath_value_smooth_prev = None
        self._breath_phase_smooth_prev = None
        self._breath_session.start()
        self._status_label.setText("状态：呼吸测量中")

    def _stop_breath_measurement(self) -> None:
        self._finalize_measurement("BREATH")

    def _finalize_measurement(self, measure_type: str) -> None:
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

    def _poll_serial(self) -> None:
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
        frame = parse_telemetry_line(line)
        if frame is None:
            self._debug_fail_count += 1
            return

        self._debug_ok_count += 1

        self._apply_frame(frame)

    def _apply_frame(self, frame: TelemetryFrame) -> None:
        heart_display_enabled = self._heart_session.active and frame.heart_enabled
        breath_display_enabled = self._breath_session.active and frame.breath_enabled

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

        self._sample_index += 1
        self._x_values.append(float(self._sample_index))
        self._heart_curve_values.append(heart_plot_value)
        self._breath_curve_values.append(breath_plot_value)
        self._heart_phase_curve_values.append(heart_phase_plot_value)
        self._breath_phase_curve_values.append(breath_phase_plot_value)

        if len(self._x_values) > self._max_points:
            self._x_values = self._x_values[-self._max_points :]
            self._heart_curve_values = self._heart_curve_values[-self._max_points :]
            self._breath_curve_values = self._breath_curve_values[-self._max_points :]
            self._heart_phase_curve_values = self._heart_phase_curve_values[-self._max_points :]
            self._breath_phase_curve_values = self._breath_phase_curve_values[-self._max_points :]

        self._value_heart_curve.setData(self._x_values, self._heart_curve_values, connect="finite")
        self._value_breath_curve.setData(self._x_values, self._breath_curve_values, connect="finite")
        self._phase_heart_curve.setData(self._x_values, self._heart_phase_curve_values, connect="finite")
        self._phase_breath_curve.setData(self._x_values, self._breath_phase_curve_values, connect="finite")
        self._update_plot_ranges()

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

        if heart_display_enabled and frame.heart_rate > 0:
            self._heart_session.add_sample(frame.heart_rate)

        if breath_display_enabled and frame.breath_rate > 0:
            self._breath_session.add_sample(frame.breath_rate)

    def _query_history(self) -> None:
        start_dt = self._query_start_dt.dateTime().toPython()
        end_dt = self._query_end_dt.dateTime().toPython()

        if end_dt < start_dt:
            QMessageBox.warning(self, "查询失败", "结束时间不能早于起始时间")
            return

        measure_type = self._query_type_combo.currentText()
        rows = self._storage.query_records(
            start_time=start_dt.strftime("%Y-%m-%d %H:%M:%S"),
            end_time=end_dt.strftime("%Y-%m-%d %H:%M:%S"),
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
        self._finalize_measurement("HEART")
        self._finalize_measurement("BREATH")

    @staticmethod
    def _finite_values(values: list[float]) -> list[float]:
        return [v for v in values if math.isfinite(v)]

    @staticmethod
    def _smooth_point(raw_value: float, prev_value: float | None, alpha: float) -> tuple[float, float | None]:
        if not math.isfinite(raw_value):
            return float("nan"), None

        if (prev_value is None) or (not math.isfinite(prev_value)):
            smoothed = raw_value
        else:
            smoothed = (alpha * raw_value) + ((1.0 - alpha) * prev_value)

        return smoothed, smoothed

    def _update_plot_ranges(self) -> None:
        if not self._x_values:
            return

        right = self._x_values[-1]
        left = max(1.0, right - float(self._scroll_window_points) + 1.0)
        if right <= left:
            right = left + 1.0

        self._value_plot_widget.setXRange(left, right, padding=0.0)
        self._phase_plot_widget.setXRange(left, right, padding=0.0)

        begin = max(0, len(self._x_values) - self._scroll_window_points)

        value_visible = self._finite_values(self._heart_curve_values[begin:])
        value_visible.extend(self._finite_values(self._breath_curve_values[begin:]))

        if value_visible:
            value_max = max(1.0, max(value_visible))
            self._value_plot_widget.setYRange(0.0, value_max * 1.15 + 0.5, padding=0.0)
        else:
            self._value_plot_widget.setYRange(0.0, 10.0, padding=0.0)

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
        self._plot_stack.setCurrentIndex(0)
        self._switch_value_page_btn.setChecked(True)
        self._switch_phase_page_btn.setChecked(False)

    def _show_phase_plot_page(self) -> None:
        self._plot_stack.setCurrentIndex(1)
        self._switch_value_page_btn.setChecked(False)
        self._switch_phase_page_btn.setChecked(True)

    def _update_debug_stats(self) -> None:
        self._debug_rx_label.setText(f"RX: {self._debug_rx_count}")
        self._debug_ok_label.setText(f"解析成功: {self._debug_ok_count}")
        self._debug_fail_label.setText(f"解析失败: {self._debug_fail_count}")

    def _append_debug_line(self, text: str) -> None:
        ts = QDateTime.currentDateTime().toString("HH:mm:ss.zzz")
        self._debug_log.appendPlainText(f"[{ts}] {text}")

    def _clear_debug_log(self) -> None:
        self._debug_log.clear()
        self._append_debug_line("日志已清空")

    def keyPressEvent(self, event) -> None:  # type: ignore[override]
        if event.key() == Qt.Key_Escape:
            self._force_stop_all_sessions()
            self._status_label.setText("状态：已停止全部测量")
            return
        super().keyPressEvent(event)
