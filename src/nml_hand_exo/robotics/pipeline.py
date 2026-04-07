from __future__ import annotations

import time
import uuid
from dataclasses import dataclass, field
from enum import Enum
from typing import Any


class PipelineStage(str, Enum):
    NORMALIZE = "normalize"
    DIALOGUE = "dialogue"
    PLAN = "plan"
    POLICY = "policy"
    EXECUTE = "execute"


@dataclass
class PipelineEvent:
    stage: PipelineStage
    started_at_s: float
    ended_at_s: float
    data: dict[str, Any] = field(default_factory=dict)

    @property
    def latency_ms(self) -> float:
        return round((self.ended_at_s - self.started_at_s) * 1000.0, 2)


@dataclass
class PipelineTrace:
    correlation_id: str = field(default_factory=lambda: str(uuid.uuid4()))
    events: list[PipelineEvent] = field(default_factory=list)

    def add_event(self, stage: PipelineStage, started_at_s: float, data: dict[str, Any] | None = None) -> PipelineEvent:
        event = PipelineEvent(
            stage=stage,
            started_at_s=started_at_s,
            ended_at_s=time.perf_counter(),
            data=data or {},
        )
        self.events.append(event)
        return event

    def to_dict(self) -> dict[str, Any]:
        return {
            "correlation_id": self.correlation_id,
            "events": [
                {
                    "stage": event.stage.value,
                    "latency_ms": event.latency_ms,
                    "data": event.data,
                }
                for event in self.events
            ],
        }
