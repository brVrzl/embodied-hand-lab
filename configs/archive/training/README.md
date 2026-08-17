# Archived training configurations

These snapshots preserve reproducibility for completed historical runs and
intermediate validation experiments. They are not active launcher inputs.

- `physical_bottle_mixed*`: the completed mixed-quality v2 ACT/ACT+Force runs
  and their four-episode validation variant;
- `physical_bottle_nominal52_split.yaml`: the split file superseded by the
  `training` section in `configs/training/shared/physical_bottle.yaml`;
- `act_force_physical_bottle_nominal52.yaml`: the optional force view now
  represented by `views.act_force` in the canonical shared task config.

Do not use an archived file for a new training run without creating a reviewed
current configuration and updating the active launcher explicitly.
