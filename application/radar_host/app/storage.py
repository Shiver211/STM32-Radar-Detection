"""数据存储：使用 SQLite 保存和查询历史测量记录。"""
from __future__ import annotations

import sqlite3
from dataclasses import dataclass
from pathlib import Path
from typing import Optional


@dataclass
class StoredMeasurement:
    """从数据库读取的一条历史记录。"""
    row_id: int
    measure_type: str
    start_time: str
    end_time: str
    average_value: Optional[float]
    sample_count: int
    port_name: str


class MeasurementStorage:
    """管理测量记录的数据库操作：建表、保存、查询。"""

    def __init__(self, db_path: Path) -> None:
        self._db_path = db_path
        self._db_path.parent.mkdir(parents=True, exist_ok=True)  # 确保目录存在
        self._initialize()

    def _initialize(self) -> None:
        """创建数据表（如果不存在）。"""
        with sqlite3.connect(self._db_path) as conn:
            conn.execute(
                """
                CREATE TABLE IF NOT EXISTS measurement_records (
                    id INTEGER PRIMARY KEY AUTOINCREMENT,
                    measure_type TEXT NOT NULL,
                    start_time TEXT NOT NULL,
                    end_time TEXT NOT NULL,
                    average_value REAL,
                    sample_count INTEGER NOT NULL,
                    port_name TEXT NOT NULL,
                    created_at TEXT NOT NULL DEFAULT (datetime('now', 'localtime'))
                )
                """
            )
            conn.commit()

    def save_record(
        self,
        measure_type: str,
        start_time: str,
        end_time: str,
        average_value: Optional[float],
        sample_count: int,
        port_name: str,
    ) -> None:
        """保存一条测量记录到数据库。"""
        with sqlite3.connect(self._db_path) as conn:
            conn.execute(
                """
                INSERT INTO measurement_records (
                    measure_type,
                    start_time,
                    end_time,
                    average_value,
                    sample_count,
                    port_name
                ) VALUES (?, ?, ?, ?, ?, ?)
                """,
                (measure_type, start_time, end_time, average_value, sample_count, port_name),
            )
            conn.commit()

    def query_records(
        self,
        start_time: str,
        end_time: str,
        measure_type: Optional[str] = None,
    ) -> list[StoredMeasurement]:
        """按时间范围和类型查询历史记录。"""
        sql = (
            "SELECT id, measure_type, start_time, end_time, average_value, sample_count, port_name "
            "FROM measurement_records "
            "WHERE start_time >= ? AND end_time <= ? "
        )
        params: list[object] = [start_time, end_time]

        # 指定了具体类型才加过滤条件
        if measure_type is not None and measure_type != "ALL":
            sql += "AND measure_type = ? "
            params.append(measure_type)

        sql += "ORDER BY start_time DESC"

        with sqlite3.connect(self._db_path) as conn:
            cursor = conn.execute(sql, params)
            rows = cursor.fetchall()

        return [
            StoredMeasurement(
                row_id=int(row[0]),
                measure_type=str(row[1]),
                start_time=str(row[2]),
                end_time=str(row[3]),
                average_value=None if row[4] is None else float(row[4]),
                sample_count=int(row[5]),
                port_name=str(row[6]),
            )
            for row in rows
        ]
