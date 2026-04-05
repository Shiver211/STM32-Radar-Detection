from __future__ import annotations

from dataclasses import dataclass, field
from datetime import datetime
from typing import Optional


@dataclass
class TelemetryFrame:
    tick_ms: int
    heart_rate: float
    breath_rate: float
    heart_phase: float
    breath_phase: float
    distance_cm: float
    human_present: bool
    online: bool
    range_valid: bool
    heart_enabled: bool
    breath_enabled: bool


@dataclass
class MeasurementStopResult:
    measure_type: str
    start_time: datetime
    end_time: datetime
    sample_count: int
    average_value: Optional[float]


@dataclass
class MeasurementSession:
    measure_type: str
    active: bool = False
    start_time: Optional[datetime] = None
    samples: list[float] = field(default_factory=list)

    def start(self, start_time: Optional[datetime] = None) -> None:
        self.active = True
        self.start_time = start_time or datetime.now()
        self.samples.clear()

    def add_sample(self, value: float) -> None:
        if not self.active:
            return
        self.samples.append(value)

    def stop(self, end_time: Optional[datetime] = None) -> Optional[MeasurementStopResult]:
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
