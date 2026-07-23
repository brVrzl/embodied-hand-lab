# Documentation

This index separates current instructions from dated evidence. Start with the
status page, then use the topic-specific page. Historical reports retain the
claims and paths that were true when written; they do not override current code
or this index.

## Current project documentation

### Architecture

- [Overview](architecture/overview.md)
- [Shared target pipeline](architecture/shared_target_pipeline.md)
- [Simulation/hardware parity](architecture/simulation_hardware_parity.md)
- [Coordinate frames](architecture/coordinate_frames.md)

### Operation

- [Quest/JAKA MuJoCo simulation](operation/simulation_demo.md)
- [Quest host setup](operation/quest_setup.md)
- [Hardware prerequisites](operation/hardware_prerequisites.md)
- [JAKA arm teleoperation](operation/jaka_arm_teleoperation.md)
- [RH56 operation](operation/rh56_operation.md)
- [Troubleshooting](operation/troubleshooting.md)

### Safety and status

- [Safety model](safety/safety_model.md)
- [Physical test gates](safety/physical_test_gates.md)
- [Controller configuration boundary](safety/controller_configuration.md)
- [Incident response](safety/incident_response.md)
- [Current status](status/current_status.md)
- [Known limitations](status/known_limitations.md)
- [Validation matrix](status/validation_matrix.md)

### Development and reference

- [Repository layout](development/repository_layout.md)
- [Setup](development/setup.md)
- [Build](development/build.md)
- [Testing](development/testing.md)
- [Configuration](development/configuration.md)
- [Logging and replay](development/logging_and_replay.md)
- [Real-robot data collection schema](development/real_robot_data_collection.md)
- [Contribution workflow](development/contribution_workflow.md)
- [Repository consolidation audit](development/repository_consolidation_audit.md)
- [Command reference](reference/command_reference.md)
- [Configuration reference](reference/config_reference.md)
- [Log schemas](reference/log_schemas.md)
- [Glossary](reference/glossary.md)

## Other current project areas

- [Digital twin workspace](digital_twin/README.md)
- [Motion-input platform](motion_input/README.md)
- [RH56 pregrasp protocol](rh56_pregrasp_prediction_protocol.md)
- [Correll RH56DFX assessment](rh56dfx_correll_integration_assessment.md)
- [RGB-D readiness](d435_depth_pointcloud_readiness.md)
- [LeRobot data and workspace calibration](lerobot_data_and_workspace_calibration.md)
- [Literature and asset reviews](literature/)

Plans for the Jetson integration and tennis-ball digital twin remain plans, not
validation evidence.

## History and evidence

[The history index](history/README.md) classifies preserved physical gates,
incidents, raw measurements, handoffs, and superseded designs. Historical
evidence is intentionally not mixed into normal operator instructions.

## Documentation rules

- Keep one authoritative current page per topic.
- Use repository-relative paths and commands verified against current `--help`.
- State whether a result is offline, simulation, physical, failed, or
  unverified.
- Never rewrite raw evidence or a failed historical outcome.
- Move superseded material to history and update its index; delete only when
  complete duplication and absence of active references are proven.
