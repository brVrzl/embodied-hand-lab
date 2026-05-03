from __future__ import annotations

from pathlib import Path

from embodiment_core.config import load_yaml

from .models import TaskTemplate


def load_task_config(path: str | Path) -> TaskTemplate:
    data = load_yaml(path)
    return TaskTemplate(
        name=data["task_name"],
        description=data["description"],
        required_sensors=data.get("required_sensors", []),
        success_condition=data["success_condition"],
        recommended_logging_fields=data.get("recommended_logging_fields", []),
    )


def load_task_library(task_dir: str | Path) -> dict[str, TaskTemplate]:
    tasks: dict[str, TaskTemplate] = {}
    for path in sorted(Path(task_dir).glob("*.yaml")):
        task = load_task_config(path)
        tasks[task.name] = task
    return tasks

