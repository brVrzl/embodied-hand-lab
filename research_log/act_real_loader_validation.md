# Real ACT loader validation

Result: **PASS**

The actual training loader was `lerobot.datasets.lerobot_dataset.LeRobotDataset` from the selected LeRobot 0.6.2 environment. The repository's custom smoke adapter was not used as evidence for this check.

## Dataset and mapping

The immutable source was `data/training/physical_bottle_v1/master`: 3,268 frames from source episodes 65 (694), 66 (483), 67 (857), 69 (535), and 70 (699). Episodes 68 and 71 are absent. Every included source episode has one contiguous `segment_id=0`; source frame indices and 30 Hz timestamps are contiguous. Both source videos have exactly the corresponding number of 640x480 RGB frames.

A derived, disposable LeRobot v3 view was written under `outputs/act/physical_bottle_5demo_validation_20260811_150941/lerobot_v3_view`. It maps source episode IDs 65/66/67/69/70 to LeRobot's required contiguous internal IDs 0/1/2/3/4, retaining the mapping in `meta/embodied_lab_provenance.json`. This operation did not modify the master dataset.

| Source | ACT feature | Loaded representation |
|---|---|---|
| workspace video | `observation.images.workspace` | uint8 RGB, CHW `[3,240,320]` |
| wrist video | `observation.images.wrist` | uint8 RGB, CHW `[3,240,320]` |
| `observation.state` | `observation.state` | float `[12]` |
| `action` | `action` | float future chunk `[16,12]` |

`observation.force` is intentionally absent from the standard ACT view. Source provenance scalars are metadata and are not policy inputs.

## Image handling

Source frames were decoded with OpenCV, resized from 640x480 to 320x240 using `INTER_AREA`, explicitly changed from OpenCV BGR to RGB, and written through the real LeRobot video writer. No random image transform, crop, or augmentation was enabled. The ACT processor converts uint8 to `[0,1]` and applies ImageNet visual mean/std (`[0.485,0.456,0.406]`, `[0.229,0.224,0.225]`) because `dataset.use_imagenet_stats=true`.

The first frame of episode 65, dataset middle, last frame of episode 65, first frame of episode 66, and final dataset frame were decoded by the real loader and compared with their source frames. RGB-reference MSE was 8.37-19.34, while deliberately BGR-interpreted MSE was 193.91-1,368.57. Deliberately swapped-camera MSE was 5,661.67-9,728.12. The contact sheet at `outputs/act/physical_bottle_5demo_validation_20260811_150941/validation/loader_camera_contact_sheet.png` was also inspected: camera identity, color, and frame correspondence are correct.

## Chunk and boundary contract

The existing smoke horizon of 16 was reconciled with the real ACT configuration rather than assumed. LeRobot ACT accepts `chunk_size=16`, and the selected policy was configured with `chunk_size=16` and `n_action_steps=16`. Action delta timestamps are `0/30` through `15/30` seconds.

LeRobot clamps an action lookup at the current episode end, repeats the episode's last action, and sets `action_is_pad=True` for the repeated positions. The model's L1 target loss masks those positions according to this contract. Exhaustive checks over all 3,268 frames established that no valid target crosses an episode. Because every included episode has only segment 0, no valid target crosses a segment either. The last frame of an episode has one valid action and 15 masked repeated actions. Reset-tail rows were removed by the existing materialization and are not present here.

Representative real-loader checks:

| Sample | Source location | State | Action chunk | Valid actions |
|---|---:|---:|---:|---:|
| first | ep 65, frame 0 | `[12]` | `[16,12]` | 16 |
| middle | ep 67, frame 457 | `[12]` | `[16,12]` | 16 |
| last of ep 65 | ep 65, frame 693 | `[12]` | `[16,12]` | 1 |
| transition | ep 66, frame 0 | `[12]` | `[16,12]` | 16 |
| dataset last | ep 70, frame 698 | `[12]` | `[16,12]` | 1 |

A real DataLoader batch of four produced state `[4,12]` and action `[4,16,12]`. Training used LeRobot's `EpisodeAwareSampler` with shuffling, seed 1000, all episode frames eligible, and `drop_n_last_frames=0`.

## ACT+Force readiness

The same 3,268 master row indices expose `observation.force` with shape `[3268,6]`, without changing episodes, frames, cameras, state, action, or split. It remains excluded from the standard ACT view. No ACT+Force architecture or training was started.

Machine-readable evidence is in `outputs/act/physical_bottle_5demo_validation_20260811_150941/validation/loader_and_normalization_validation.json`.
