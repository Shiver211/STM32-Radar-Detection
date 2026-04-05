"""数据模型：定义遥测帧、测量会话等核心数据结构。"""
from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


@dataclass
class TelemetryFrame:
    """一条完整的雷达遥测数据，包含心率、呼吸、距离等信息。"""
    tick_ms: int           # 设备运行时间（毫秒）
    heart_rate: float      # 心率
    breath_rate: float     # 呼吸率
    heart_phase: float     # 心率相位（用于画波形）
    breath_phase: float    # 呼吸相位（用于画波形）
    distance_cm: float     # 目标距离（厘米）
    human_present: bool    # 是否有人
    online: bool           # 设备是否在线
    range_valid: bool      # 距离数据是否有效
    heart_enabled: bool    # 心率测量是否开启
    breath_enabled: bool   # 呼吸测量是否开启


@dataclass
class MeasurementStopResult:
    """一次测量结束后产生的汇总结果。"""
    measure_type: str          # 测量类型（HEART / BREATH）
    start_time: datetime       # 开始时间
    end_time: datetime         # 结束时间
    sample_count: int          # 采集到的样本数
    average_value: Optional[float]  # 平均值


@dataclass
class MeasurementSession:
    """一次测量会话：记录从"开始"到"停止"期间的所有样本。"""
    measure_type: str
    active: bool = False
    start_time: Optional[datetime] = None
    samples: list[float] = field(default_factory=list)

    def start(self, start_time: Optional[datetime] = None) -> None:
        """开始测量：清空旧数据，记录开始时间。"""
        self.active = True
        self.start_time = start_time or datetime.now()
        self.samples.clear()

    def add_sample(self, value: float) -> None:
        """添加一个样本值（仅在测量进行中有效）。"""
        if not self.active:
            return
        self.samples.append(value)

    def stop(self, end_time: Optional[datetime] = None) -> Optional[MeasurementStopResult]:
        """结束测量：计算平均值，返回结果。"""
        if (not self.active) or (self.start_time is None):
            return None

        final_time = end_time or datetime.now()
        sample_count = len(self.samples)
        average_value = (sum(self.samples) / sample_count) if sample_count > 0 else None

        result = MeasurementStopResult(
            measure_type=self.measure_type,
            start_time=self.start_time,
            end_time=final_time,
            sample_count=sample_count,
            average_value=average_value,
        )

        self.active = False
        self.start_time = None
        self.samples.clear()
        return result
