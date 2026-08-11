# Physical bottle training dataset v1

This is an offline materialization report. Raw staging episodes were not modified.

- Master schema: `embodied_lab.physical_training.v1`
- Task: `place_bottle_on_cardboard_box` — Pick up the bottle and place it on the cardboard box.
- Included episodes: 5
- Master rows: 3268
- Cropped duration: 108.767 s
- Master image frames: 6536 (workspace + wrist)
- ACT+Force-valid rows: 3268
- Train/val/test episodes: `[65, 66, 67, 69, 70]` / `[]` / `[]`

## Per-episode materialization

| episode | raw duration | raw frames | crop start | crop end | crop frames | excluded | force valid | recovery/manual flag | status |
|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| 65 | 28.099999719 | 844 | 2317783532878211 | 2317806639073046 | 694 | 150 | 1.000 | False | included |
| 66 | 21.066666456 | 633 | 2317826920317031 | 2317842999217416 | 483 | 150 | 1.000 | False | included |
| 67 | 33.499999665 | 1006 | 2317854677503217 | 2317883215236451 | 857 | 149 | 1.000 | False | included |
| 68 | 73.499999265 | 2205 | 2317891856078210 | 2317960386182046 | 2055 | 2205 | 1.000 | True | excluded_by_manifest |
| 69 | 22.799999772 | 685 | 2317973570305038 | 2317991393559113 | 535 | 150 | 1.000 | False | included |
| 70 | 28.266666384 | 849 | 2318000627220865 | 2318023919074247 | 699 | 150 | 1.000 | False | included |
| 71 | 35.499999645 | 1066 | 2318035370392959 | 2318065882323032 | 916 | 1066 | 1.000 | True | excluded_by_manifest |

## Semantics

The master stores absolute/native 12-D actions. ACT reads images, state, and action; ACT+Force reads the same rows plus six raw FORCE_ACT counts, force age, and force validity. The initial openpi adapter reads images, state, task prompt, and absolute action, and does not read force.

Normalization files are computed only from the configured train episodes. Force statistics use only rows with `force_valid=true`.

A short final both-clutches-released debounce/reset tail is excluded using the persisted release timestamp. No fixed five-second subtraction or vision-based crop was used.
