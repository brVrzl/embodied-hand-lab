from __future__ import annotations

from dataclasses import dataclass, field


@dataclass(slots=True)
class TaskTemplate:
    name: str
    description: str
    required_sensors: list[str]
    success_condition: str
    recommended_logging_fields: list[str] = field(default_factory=list)

