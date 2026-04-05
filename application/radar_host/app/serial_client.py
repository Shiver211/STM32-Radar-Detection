from __future__ import annotations

import re
from typing import Optional

import serial
from serial.tools import list_ports

from .models import TelemetryFrame


LEGACY_SUMMARY_PATTERN = re.compile(
    r"心率[:：]\s*(\d+)\s+呼吸[:：]\s*(\d+)\s+位置[:：]\s*(\d+)\s+人数[:：]\s*(\d+)"
)


class SerialClient:
    def __init__(self) -> None:
        self._serial: Optional[serial.Serial] = None
        self._rx_buffer = bytearray()

    @property
    def is_open(self) -> bool:
        return self._serial is not None and self._serial.is_open

    @property
    def port_name(self) -> str:
        if not self.is_open:
            return ""
        return str(self._serial.port)

    @staticmethod
    def list_available_ports() -> list[tuple[str, str]]:
        ports = []
        for port in list_ports.comports():
            description = port.description or "Unknown"
            ports.append((port.device, description))
        ports.sort(key=lambda item: item[0])
        return ports

    def open(self, port: str, baudrate: int) -> None:
        self.close()
        self._serial = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=0,
            write_timeout=0.2,
        )
        self._rx_buffer.clear()

    def close(self) -> None:
        if self._serial is not None:
            try:
                if self._serial.is_open:
                    self._serial.close()
            finally:
                self._serial = None
                self._rx_buffer.clear()

    def read_lines(self) -> list[str]:
        if not self.is_open:
            return []

        try:
            waiting = int(self._serial.in_waiting)
            if waiting <= 0:
                return []
            chunk = self._serial.read(waiting)
        except serial.SerialException:
            return []

        if not chunk:
            return []

        # 兼容只使用 \r 作为行结束符的设备，统一转成 \n 再分行。
        self._rx_buffer.extend(chunk.replace(b"\r", b"\n"))
        lines: list[str] = []

        while True:
            new_line_index = self._rx_buffer.find(b"\n")
            if new_line_index < 0:
                break

            one_line = self._rx_buffer[:new_line_index]
            del self._rx_buffer[: new_line_index + 1]
            decoded = one_line.decode("utf-8", errors="ignore").strip()
            if decoded:
                lines.append(decoded)

        return lines


def parse_telemetry_line(line: str) -> Optional[TelemetryFrame]:
    def parse_int_field(raw: str, default: int = 0) -> int:
        text = raw.strip()
        if text == "":
            return default
        return int(text)

    def parse_float_field(raw: str, default: float = 0.0) -> float:
        text = raw.strip()
        if text == "":
            return default
        return float(text)

    parts = line.split(",")
    if len(parts) >= 12 and parts[0] == "RADAR":
        try:
            return TelemetryFrame(
                tick_ms=parse_int_field(parts[1], 0),
                heart_rate=parse_float_field(parts[2], 0.0),
                breath_rate=parse_float_field(parts[3], 0.0),
                heart_phase=parse_float_field(parts[4], 0.0),
                breath_phase=parse_float_field(parts[5], 0.0),
                distance_cm=parse_float_field(parts[6], 0.0),
                human_present=parts[7] == "1",
                online=parts[8] == "1",
                range_valid=parts[9] == "1",
                heart_enabled=parts[10] == "1",
                breath_enabled=parts[11] == "1",
            )
        except ValueError:
            return None

    # 兼容旧版下位机文本：
    # [Rader]:心率：72  呼吸：14  位置：53  人数：1
    match = LEGACY_SUMMARY_PATTERN.search(line)
    if match is None:
        return None

    try:
        heart_rate = float(match.group(1))
        breath_rate = float(match.group(2))
        distance_cm = float(match.group(3))
        people_count = int(match.group(4))

        return TelemetryFrame(
            tick_ms=0,
            heart_rate=heart_rate,
            breath_rate=breath_rate,
            heart_phase=0.0,
            breath_phase=0.0,
            distance_cm=distance_cm,
            human_present=people_count > 0,
            online=True,
            range_valid=distance_cm > 0,
            heart_enabled=heart_rate > 0,
            breath_enabled=breath_rate > 0,
        )
    except ValueError:
        return None
