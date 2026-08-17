from __future__ import annotations

import json
from pathlib import Path

import numpy as np
import pytest

from embodiment_core.act_contract import ActCheckpointContract


def _write_checkpoint(path: Path, *, image_prefix: str = "workspace", chunk: int = 60) -> None:
    path.mkdir()
    (path / "config.json").write_text(
        json.dumps(
            {
                "type": "act",
                "chunk_size": chunk,
                "input_features": {
                    f"observation.images.{image_prefix}": {"type": "VISUAL", "shape": [3, 240, 320]},
                    "observation.images.wrist": {"type": "VISUAL", "shape": [3, 240, 320]},
                    "observation.state": {"type": "STATE", "shape": [12]},
                },
                "output_features": {"action": {"type": "ACTION", "shape": [12]}},
            }
        ),
        encoding="utf-8",
    )


def test_contract_reads_dynamic_chunk_and_image_keys(tmp_path: Path) -> None:
    checkpoint = tmp_path / "checkpoint"
    _write_checkpoint(checkpoint, image_prefix="fixed", chunk=60)
    contract = ActCheckpointContract.from_checkpoint(checkpoint)
    assert contract.chunk_size == 60
    assert contract.image_keys == (
        "observation.images.fixed",
        "observation.images.wrist",
    )
    assert contract.workspace_image_key == "observation.images.fixed"
    assert contract.wrist_image_key == "observation.images.wrist"
    observation = {
        key: np.zeros((3, 240, 320), dtype=np.float32) for key in contract.image_keys
    }
    observation[contract.state_key] = np.zeros(12, dtype=np.float32)
    contract.validate_observation(observation)


def test_contract_rejects_wrong_action_shape_before_runtime(tmp_path: Path) -> None:
    checkpoint = tmp_path / "checkpoint"
    _write_checkpoint(checkpoint)
    document = json.loads((checkpoint / "config.json").read_text())
    document["output_features"]["action"]["shape"] = [16]
    (checkpoint / "config.json").write_text(json.dumps(document))
    with pytest.raises(ValueError, match="action shape"):
        ActCheckpointContract.from_checkpoint(checkpoint)
