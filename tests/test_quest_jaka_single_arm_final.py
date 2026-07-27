from __future__ import annotations

import mujoco

from quest_jaka_sim import JakaMujocoSimulation, ReplayConfig
from quest_jaka_sim.simulation import build_viewer_mjcf


def test_final_single_arm_profile_preserves_distinct_periods_and_pi_limits(tmp_path) -> None:
    config = ReplayConfig.load(
        "configs/sim/quest_hts_jaka_mini2_live_demo.yaml",
        speed_profile="root_cause_fix",
    )
    assert config.output_contract.velocity_boundaries_rad_s == (3.141592653589793,) * 6
    assert config.output_contract.feasibility_acceleration_period_ns == 16_666_667
    assert config.output_contract.servo_period_ns == 8_000_000

    model_path = build_viewer_mjcf(
        config.mjcf_path,
        tmp_path / "single_arm_viewer.xml",
        arm_only=True,
    )
    model = mujoco.MjModel.from_xml_path(str(model_path))
    actuator_names = [mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i) for i in range(model.nu)]
    assert len(actuator_names) == 6
    assert all(name.startswith("jaka_joint_") for name in actuator_names)

    plant = JakaMujocoSimulation(config, mjcf_path=model_path)
    assert plant.hand_available is False
    assert len(plant.arm_actuator_ids) == 6
    plant.step(0.02)
    assert plant.data.time > 0.0
