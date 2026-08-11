"""Artifact-backed integration safeguards for the physical-bottle ACT validation.

Run this file inside the selected LeRobot environment with
``ACT_VALIDATION_EXPERIMENT`` set to the experiment directory.  The normal host
test environment skips it when PyTorch/LeRobot or the artifact path is absent.
"""

from __future__ import annotations

import json
import os
from pathlib import Path
import unittest

try:
    import torch
    from lerobot.configs import PreTrainedConfig
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    from lerobot.policies.act.modeling_act import ACTPolicy
    from lerobot.policies.factory import make_pre_post_processors
except ImportError:
    torch = None


EXPECTED_EPISODES = (65, 66, 67, 69, 70)
STATE_NAMES = (
    "jaka_joint_1",
    "jaka_joint_2",
    "jaka_joint_3",
    "jaka_joint_4",
    "jaka_joint_5",
    "jaka_joint_6",
    "rh56_index",
    "rh56_middle",
    "rh56_ring",
    "rh56_pinky",
    "rh56_thumb_close",
    "rh56_thumb_lateral",
)
ACTION_NAMES = (
    "jaka_target_1",
    "jaka_target_2",
    "jaka_target_3",
    "jaka_target_4",
    "jaka_target_5",
    "jaka_target_6",
    "rh56_target_index",
    "rh56_target_middle",
    "rh56_target_ring",
    "rh56_target_pinky",
    "rh56_target_thumb_close",
    "rh56_target_thumb_lateral",
)


@unittest.skipIf(torch is None, "requires the selected LeRobot/PyTorch environment")
class ActPhysicalBottleIntegrationTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        experiment_text = os.environ.get("ACT_VALIDATION_EXPERIMENT")
        if not experiment_text:
            raise unittest.SkipTest("ACT_VALIDATION_EXPERIMENT is not set")
        cls.experiment = Path(experiment_text).resolve()
        cls.view = cls.experiment / "lerobot_v3_view"
        cls.checkpoint = cls.experiment / "strong_run/checkpoints/002000/pretrained_model"
        if not cls.view.is_dir() or not cls.checkpoint.is_dir():
            raise unittest.SkipTest("ACT validation artifacts are not present")
        cls.provenance = json.loads(
            (cls.view / "meta/embodied_lab_provenance.json").read_text(encoding="utf-8")
        )
        cls.config = PreTrainedConfig.from_pretrained(cls.checkpoint)
        cls.device = "cuda" if torch.cuda.is_available() else "cpu"
        cls.config.device = cls.device
        cls.dataset = LeRobotDataset(
            "local/physical_bottle_5demo_act_validation",
            root=cls.view,
            delta_timestamps={"action": [i / 30 for i in range(cls.config.chunk_size)]},
            video_backend="pyav",
            return_uint8=True,
        )
        cls.policy = ACTPolicy.from_pretrained(cls.checkpoint, config=cls.config).to(cls.device)
        cls.policy.eval()
        cls.preprocessor, cls.postprocessor = make_pre_post_processors(
            policy_cfg=cls.config,
            pretrained_path=str(cls.checkpoint),
            preprocessor_overrides={"device_processor": {"device": cls.device}},
        )

    def _batch(self, index: int = 100):
        batch = torch.utils.data.default_collate([self.dataset[index]])
        for key in ("observation.images.workspace", "observation.images.wrist"):
            batch[key] = batch[key].float() / 255.0
        return batch

    def test_exact_state_and_action_ordering(self) -> None:
        self.assertEqual(tuple(self.provenance["state_order"]), STATE_NAMES)
        self.assertEqual(tuple(self.provenance["action_order"]), ACTION_NAMES)
        self.assertEqual(tuple(self.dataset.meta.features["observation.state"]["names"]), STATE_NAMES)
        self.assertEqual(tuple(self.dataset.meta.features["action"]["names"]), ACTION_NAMES)
        self.assertEqual(tuple(self.dataset[0]["observation.state"].shape), (12,))
        self.assertEqual(tuple(self.dataset[0]["action"].shape), (16, 12))

    def test_exclusions_and_chunk_boundaries(self) -> None:
        loaded = sorted(int(value) for value in self.dataset.hf_dataset.unique("source_episode_index"))
        self.assertEqual(loaded, list(EXPECTED_EPISODES))
        self.assertNotIn(68, loaded)
        self.assertNotIn(71, loaded)
        offset = 0
        for episode in self.provenance["episode_map"]:
            length = int(episode["length"])
            end_sample = self.dataset[offset + length - 1]
            self.assertEqual(int(end_sample["source_episode_index"]), episode["source_episode_index"])
            self.assertEqual(int(end_sample["segment_id"]), 0)
            self.assertEqual(int((~end_sample["action_is_pad"]).sum()), 1)
            expected = end_sample["action"][0].expand_as(end_sample["action"])
            self.assertTrue(torch.equal(end_sample["action"], expected))
            offset += length
        self.assertEqual(offset, len(self.dataset))

    def test_model_normalization_round_trip(self) -> None:
        batch = self._batch()
        raw_action = batch["action"].clone()
        transformed = self.preprocessor(batch)
        reconstructed = self.postprocessor(transformed["action"])
        self.assertLessEqual(torch.max(torch.abs(raw_action - reconstructed)).item(), 1e-5)

    def test_checkpoint_reload_preserves_config_normalization_and_output(self) -> None:
        self.assertEqual(self.config.chunk_size, 16)
        self.assertEqual(set(self.config.image_features), {
            "observation.images.workspace",
            "observation.images.wrist",
        })
        self.assertEqual(tuple(self.config.output_features), ("action",))
        self.assertTrue(
            (self.checkpoint / "policy_preprocessor_step_3_normalizer_processor.safetensors").is_file()
        )
        batch = self._batch()
        processed = self.preprocessor(batch)
        with torch.inference_mode():
            first = self.postprocessor(self.policy.predict_action_chunk(processed)).cpu()

        reloaded_config = PreTrainedConfig.from_pretrained(self.checkpoint)
        reloaded_config.device = self.device
        reloaded_policy = ACTPolicy.from_pretrained(
            self.checkpoint, config=reloaded_config
        ).to(self.device)
        reloaded_policy.eval()
        reloaded_pre, reloaded_post = make_pre_post_processors(
            policy_cfg=reloaded_config,
            pretrained_path=str(self.checkpoint),
            preprocessor_overrides={"device_processor": {"device": self.device}},
        )
        with torch.inference_mode():
            second = reloaded_post(reloaded_policy.predict_action_chunk(reloaded_pre(self._batch()))).cpu()
        self.assertEqual(tuple(first.shape), (1, 16, 12))
        self.assertTrue(torch.isfinite(first).all())
        self.assertTrue(torch.equal(first, second))


if __name__ == "__main__":
    unittest.main(verbosity=2)
