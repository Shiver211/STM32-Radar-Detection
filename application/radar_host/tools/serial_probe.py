"""串口抓包调试工具：用于在命令行下测试串口数据收发和解析。"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import serial
from serial.tools import list_ports

# 兼容直接从 tools 目录运行脚本
PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from app.serial_client import parse_telemetry_line  # noqa: E402


def choose_port(explicit_port: str | None) -> str:
    """自动选择串口：指定了就用指定的，只有一个就自动选，多个则报错提示手动指定。"""
    ports = list(list_ports.comports())
    print("可用串口:")
    for p in ports:
        print(f"  - {p.device}: {p.description}")

    if explicit_port:
        return explicit_port

    if len(ports) == 1:
        return ports[0].device

    raise RuntimeError("未指定端口且检测到多个串口，请使用 --port 指定")


def main() -> int:
    parser = argparse.ArgumentParser(description="Radar 串口抓包调试工具")
    parser.add_argument("--port", type=str, default=None, help="串口号，例如 COM9")
    parser.add_argument("--baud", type=int, default=115200, help="波特率")
    parser.add_argument("--seconds", type=float, default=10.0, help="抓包时长（秒）")
    parser.add_argument(
        "--send",
        action="append",
        default=[],
        help="打开串口后额外发送的命令，可重复（默认不发送任何命令）",
    )
    args = parser.parse_args()

    try:
        port = choose_port(args.port)
    except Exception as exc:
        print(f"端口选择失败: {exc}")
        return 2

    print(f"\n打开串口: {port} @ {args.baud}")

    try:
        ser = serial.Serial(port=port, baudrate=args.baud, timeout=0.05, write_timeout=0.2)
    except Exception as exc:
        print(f"打开串口失败: {exc}")
        return 3

    tx_count = 0
    rx_count = 0
    parsed_ok = 0
    parsed_fail = 0
    buf = bytearray()

    try:
        send_list = list(args.send)

        # 发送用户指定的命令
        for cmd in send_list:
            payload = (cmd.strip() + "\n").encode("ascii", errors="ignore")
            ser.write(payload)
            tx_count += 1
            print(f"TX> {cmd}")

        # 在指定时间内持续读取数据
        end_time = time.time() + max(0.1, args.seconds)
        while time.time() < end_time:
            chunk = ser.read(512)
            if not chunk:
                continue

            buf.extend(chunk)
            while True:
                nl = buf.find(b"\n")
                if nl < 0:
                    break

                line_raw = bytes(buf[:nl])
                del buf[: nl + 1]
                line = line_raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                rx_count += 1
                print(f"RX< {line}")

                # 尝试解析
                frame = parse_telemetry_line(line)
                if frame is None:
                    parsed_fail += 1
                    continue

                parsed_ok += 1
                print(
                    "   -> PARSED",
                    f"HR={frame.heart_rate:.2f}",
                    f"BR={frame.breath_rate:.2f}",
                    f"H_EN={int(frame.heart_enabled)}",
                    f"B_EN={int(frame.breath_enabled)}",
                )

    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        try:
            ser.close()
        except Exception:
            pass

    # 输出统计结果和诊断建议
    print("\n调试统计:")
    print(f"  TX: {tx_count}")
    print(f"  RX: {rx_count}")
    print(f"  解析成功: {parsed_ok}")
    print(f"  解析失败: {parsed_fail}")

    if rx_count == 0:
        print("\n结论: 串口未收到任何数据。优先检查串口线、波特率、下位机是否在运行。")
    elif parsed_ok == 0:
        print("\n结论: 收到数据但格式未匹配，请贴出 RX 行进行协议适配。")
    else:
        print("\n结论: 数据链路正常，若界面无曲线应重点排查绘图逻辑与启停状态。")

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
