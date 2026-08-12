# Latest physical data audit

审计日期：2026-08-11（Asia/Shanghai）
Episode root：`data/raw_episodes`
配置：`configs/data_collection/physical_collection.yaml`，`collection_profile: production`，目标 30 Hz。

## 范围与结论

本次“最新 production 批次”按 `lerobot_episode_staging_v2` 和采集时间确定为
`episode_000051`–`episode_000064`。根目录中的 `episode_000065` 只有
`meta/rejected/episode_000065.json`，原因为
`outer_session_ended_before_episode_boundary`，没有 canonical rows 或视频，不能作为 episode
审计。`000000`–`000050` 是旧的 `lerobot_episode_staging_v1` 批次，不属于本次 updated
production 范围。

审计只读打开了 canonical JSONL、quality JSONL、episode metadata、MP4，并调用现有
`synchronize_staging_episode` 做离线同步检查；未连接硬件，未修改原始 episode 或代码。

**总体决定：C. COLLECTION ISSUE REMAINS。**

主要原因不是 JSON/MP4 损坏，而是采集完整性没有达到 30 Hz/casual provenance 的要求：

- 14 个 episode 的有效 canonical-row 频率为 19.81–27.62 Hz；每个 episode 都有大量
  canonical deadline 缺槽（38–753 个），不是 30 Hz 连续数据。
- `episode_000055`、`56`、`58`、`60`、`62`、`63`、`64` 出现 source timestamp
  晚于 canonical timestamp 的记录；其中 FORCE 在 `56`、`62`、`63`、`64` 中出现未来
  provenance，不能直接作为无条件有效的 ACT+Force 样本。
- FORCE 的 unique update rate 为 8.07–9.85 Hz，名义上接近 10 Hz，但最大 update gap
  为 292 ms–4.299 s；`episode_000062` 的 4.299 s gap 明显需要调查。
- recorder queue drop、ring-reference expiry、writer failure 均为 0；因此目前证据更像
  producer/deadline/source timestamp 质量问题，而不是 recorder 文件截断。

这些 episode 在**schema 层面**可以复用：ACT 忽略 `observation.force`，ACT+Force 使用同一
episode 的 force、age 和 validity。但在清理未来 provenance、缺槽和 aborted episode 之前，
不能宣称可直接用于 pilot training，也不建议继续按当前配置批量采集。

## Episode summary

表中 `missed` 为 `nominal_slot_index` 缺槽数 / metadata-only quality rows；camera gaps 为
`workspace / wrist` 的 RealSense frame-number gap events / 估计缺失 frame 数。timestamp
errors 只列 source timestamp 晚于 canonical 的数量；所有 episode 的 timestamp regression
计数均为 0。

| episode | duration (s) | canonical Hz | workspace FPS | wrist FPS | JAKA Hz | ANGLE Hz | FORCE Hz | FORCE causal valid % | missed | camera gaps | recorder drops | ring expiry | timestamp errors | loader | overall |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---:|---:|---|---|---|
| 51 | 6.233 | 21.34 | 20.1 | 21.3 | 21.0 | 11.9 | 8.4 | 100.00 | 54/0 | 22/61 / 17/54 | 0 | 0 | — | OK | REJECT FROM TRAINING |
| 52 | 13.167 | 21.95 | 21.6 | 21.9 | 21.9 | 11.4 | 8.3 | 100.00 | 106/1 | 29/110 / 24/106 | 0 | 0 | — | OK | REJECT FROM TRAINING |
| 53 | 20.767 | 22.01 | 21.9 | 22.0 | 21.6 | 11.2 | 8.1 | 100.00 | 166/2 | 21/167 / 20/166 | 0 | 0 | — | OK | REJECT FROM TRAINING |
| 54 | 25.067 | 22.34 | 22.3 | 22.3 | 22.0 | 12.8 | 8.9 | 100.00 | 192/0 | 87/192 / 87/192 | 0 | 0 | — | OK | REJECT FROM TRAINING |
| 55 | 19.533 | 24.27 | 23.8 | 24.3 | 24.3 | 13.2 | 9.0 | 100.00 | 112/2 | 61/121 / 53/112 | 0 | 0 | ANGLE 22 | OK | REJECT FROM TRAINING |
| 56 | 48.067 | 21.55 | 21.4 | 21.3 | 21.4 | 13.1 | 9.0 | 99.81 | 406/5 | 260/413 / 262/415 | 0 | 0 | CMD 4, ANGLE 68, FORCE 2 | OK | REJECT FROM TRAINING |
| 57 | 8.133 | 26.31 | 26.3 | 26.3 | 26.1 | 13.5 | 9.6 | 100.00 | 30/0 | 17/30 / 17/30 | 0 | 0 | — | OK | REJECT FROM TRAINING |
| 58 | 11.633 | 24.93 | 24.0 | 24.9 | 24.7 | 12.8 | 8.8 | 100.00 | 59/1 | 26/69 / 16/59 | 0 | 0 | ANGLE 1 | OK | REJECT FROM TRAINING |
| 59 | 28.267 | 25.61 | 25.4 | 25.5 | 25.2 | 12.9 | 8.8 | 100.00 | 124/4 | 17/128 / 17/127 | 0 | 0 | — | OK | INVESTIGATE |
| 60 | 27.000 | 24.89 | 24.9 | 24.9 | 24.7 | 14.4 | 9.9 | 100.00 | 138/0 | 125/138 / 125/138 | 0 | 0 | ANGLE 7 | OK | INVESTIGATE |
| 61 | 12.967 | 22.21 | 22.2 | 22.2 | 21.8 | 13.9 | 9.5 | 100.00 | 101/1 | 80/101 / 80/101 | 0 | 0 | — | OK | REJECT FROM TRAINING |
| 62 | 73.900 | 19.81 | 19.6 | 19.6 | 19.3 | 12.1 | 8.4 | 99.11 | 753/9 | 401/766 / 405/769 | 0 | 0 | CMD 1, ANGLE 58, FORCE 13 | OK | REJECT FROM TRAINING |
| 63 | 15.933 | 27.62 | 26.9 | 27.6 | 27.3 | 14.6 | 9.8 | 99.77 | 38/0 | 39/49 / 28/38 | 0 | 0 | ANGLE 7, FORCE 1 | OK | INVESTIGATE |
| 64 | 15.600 | 26.99 | 27.0 | 27.0 | 26.4 | 14.3 | 9.7 | 99.05 | 47/1 | 31/47 / 31/47 | 0 | 0 | FORCE 4 | OK | INVESTIGATE |

All 14 files had row count equal to metadata `num_frames`, contiguous `frame_index`, strictly
increasing canonical timestamps, no malformed JSON lines, and no exact duplicate canonical
payloads. `end_timestamp_ns` equals the last canonical timestamp and `duration_s` equals the
first-to-last duration. However, `canonical_metadata_duration_ns` is `0` in all 14 metadata files,
so that field is not a trustworthy episode-duration field.

The MP4s all opened and decoded exactly one frame per canonical row at 640×480. The container
declares 30 FPS, but it does not preserve skipped canonical time: for example, episode 60 has
27.000 s of canonical time but a 22.433 s / 673-frame MP4. Video playback frame index must not be
used as the temporal clock; the stored canonical/device timestamps must be used.

## ACT state/action schema

Every canonical row in episodes 51–64 has:

- `observation.state`: exactly 12 finite values, JAKA measured 6 + RH56 ANGLE_ACT position 6;
- `action`: exactly 12 finite values, JAKA target 6 + RH56 target 6;
- `observation.force`: exactly 6 finite, integer-valued samples consistent with raw RH56 counts;
- no NaN/Inf or dimension changes.

Episode 51 is a short pre-manipulation abort: all 12 action channels and all FORCE channels are
constant. The other episodes show non-zero hand position/target motion; representative RH56
position and target ranges are below, in the existing position units/radians:

| ep | measured position range H1..H6 | target range H1..H6 |
|---:|---|---|
| 51 | 0/0.001/0/0/0/0.001 | 0/0/0/0/0/0 |
| 52 | 0.147/0.224/0.287/0.234/0.088/0.529 | 0.144/0.232/0.295/0.231/0.094/0.525 |
| 53 | 0.087/0.073/0.112/0.084/0.086/0.544 | 0.088/0.068/0.113/0.086/0.092/0.565 |
| 54 | 0.173/0.161/0.246/0.227/0.001/0.389 | 0.174/0.162/0.249/0.224/0.008/0.405 |
| 55 | 0.481/0.512/0.518/0.520/0.392/0.945 | 0.483/0.512/0.521/0.516/0.400/0.942 |
| 56 | 0.412/0.417/0.397/0.335/0.261/0.694 | 0.429/0.476/0.452/0.345/0.288/0.673 |
| 57 | 0.182/0.208/0.224/0.135/0.035/0.433 | 0.188/0.213/0.230/0.137/0.039/0.440 |
| 58 | 0.160/0.161/0.195/0.130/0.057/0.213 | 0.161/0.153/0.202/0.125/0.053/0.211 |
| 59 | 0.339/0.415/0.403/0.337/0.244/0.360 | 0.391/0.439/0.425/0.338/0.275/0.375 |
| 60 | 0.378/0.414/0.266/0.278/0.270/0.352 | 0.379/0.452/0.274/0.283/0.284/0.355 |
| 61 | 0.415/0.441/0.323/0.322/0.222/0.241 | 0.427/0.443/0.318/0.328/0.213/0.256 |
| 62 | 0.403/0.560/0.585/0.602/0.301/0.435 | 0.419/0.569/0.599/0.609/0.319/0.439 |
| 63 | 0.376/0.396/0.274/0.290/0.124/0.324 | 0.377/0.396/0.282/0.297/0.129/0.325 |
| 64 | 0.376/0.415/0.416/0.325/0.241/0.405 | 0.376/0.415/0.417/0.335/0.250/0.418 |

## FORCE_ACT and RH56 timing

FORCE values are raw counts; no Newton conversion is present. Repeated force values across
canonical rows are expected zero-order hold. The table gives `FORCE age p50/p95/max` in ms and
the unique-source update interval p50/p95/max in ms. The latter is also the longest observed
stale/update gap; gaps over 200 ms are flagged by the audit.

| ep | JAKA obs Hz; age p50/p95/max ms; repeated rows | ANGLE Hz; age p50/p95/max ms | FORCE age p50/p95/max ms | FORCE interval p50/p95/max ms | causal force valid |
|---:|---|---|---|---|---:|
| 51 | 21.0; 20.2/38.9/57.6; 2 | 11.9; 60.7/94.2/94.2 | 43.8/109.2/110.7 | 100.1/299.2/301.9 | 100.00% |
| 52 | 21.9; 46.6/64.0/95.9; 3 | 11.4; 69.5/125.1/125.2 | 74.9/134.8/141.9 | 100.0/298.9/701.2 | 100.00% |
| 53 | 21.6; 51.0/64.4/71.6; 10 | 11.2; 70.6/125.5/125.5 | 75.6/109.3/142.7 | 100.2/298.5/700.3 | 100.00% |
| 54 | 22.0; 39.3/64.6/67.3; 10 | 12.8; 53.1/119.8/119.8 | 102.2/136.7/137.0 | 100.0/200.1/501.2 | 100.00% |
| 55 | 24.3; 16.6/32.3/60.3; 0 | 13.2; 33.4/95.4/128.7 | 41.0/105.1/140.3 | 100.1/104.4/1499.4 | 100.00% |
| 56 | 21.4; 18.1/47.4/64.7; 6 | 13.1; 31.8/89.3/122.7 | 38.8/103.4/136.4 | 100.1/103.9/1197.3 | 99.81% |
| 57 | 26.1; 18.0/32.7/44.6; 3 | 13.5; 38.3/71.9/105.2 | 53.0/86.9/120.7 | 100.0/105.8/292.1 | 100.00% |
| 58 | 24.7; 49.6/64.3/65.7; 4 | 12.8; 65.0/121.1/121.1 | 96.3/137.8/138.5 | 99.9/107.2/900.5 | 100.00% |
| 59 | 25.2; 33.0/47.7/63.6; 13 | 12.9; 46.2/76.0/109.3 | 57.9/92.6/126.3 | 100.1/106.8/899.1 | 100.00% |
| 60 | 24.7; 44.3/63.0/65.7; 7 | 14.4; 59.2/92.6/126.0 | 73.0/108.4/136.6 | 100.0/103.7/299.2 | 100.00% |
| 61 | 21.8; 21.6/56.3/64.3; 5 | 13.9; 36.8/92.6/125.8 | 42.7/107.7/136.7 | 100.0/101.5/799.5 | 100.00% |
| 62 | 19.3; 26.0/59.0/66.0; 38 | 12.1; 35.9/92.6/125.9 | 42.2/108.1/141.9 | 100.1/105.4/**4298.7** | 99.11% |
| 63 | 27.3; 19.9/47.5/62.2; 6 | 14.6; 31.3/60.0/93.4 | 42.3/76.9/110.3 | 100.0/105.1/298.0 | 99.77% |
| 64 | 26.4; 31.5/47.5/48.8; 10 | 14.3; 42.6/76.0/139.7 | 58.2/92.6/126.0 | 99.9/103.3/599.7 | 99.05% |

The direct `rh56_force_act_valid` field is true for the future-force rows in the affected files,
while `timing.source_validity.rh56_force_act` is false. For offline training, the latter causal
provenance mask must take precedence. No FORCE timestamp regression was found. The existing
synchronizer reports `future_force_samples_used: false` for all 14 timelines because its latest
selection is causal, but the raw canonical rows still contain the provenance violations listed in
the summary table.

FORCE min/max/range by channel (H1..H6, raw RH56 counts):

| ep | H1 | H2 | H3 | H4 | H5 | H6 |
|---:|---|---|---|---|---|---|
| 51 | -4..-4 (0) | 0..0 (0) | -24..-24 (0) | -25..-25 (0) | -2..-2 (0) | 24..24 (0) |
| 52 | -8..4 (12) | -6..5 (11) | -24..-7 (17) | -33..-17 (16) | -8..-2 (6) | 3..27 (24) |
| 53 | -2..1 (3) | -5..-3 (2) | -20..-17 (3) | -26..-23 (3) | -8..-5 (3) | 10..25 (15) |
| 54 | -5..2 (7) | -5..5 (10) | -18..-10 (8) | -41..-16 (25) | -10..-7 (3) | 0..15 (15) |
| 55 | -12..5 (17) | -13..10 (23) | -31..-11 (20) | -50..-16 (34) | -11..0 (11) | -6..32 (38) |
| 56 | -4..623 (627) | -5..639 (644) | -22..615 (637) | -42..108 (150) | -11..1086 (1097) | -334..339 (673) |
| 57 | 1..10 (9) | 2..6 (4) | -15..45 (60) | -30..-7 (23) | -10..46 (56) | -41..13 (54) |
| 58 | -2..10 (12) | -3..3 (6) | -15..-1 (14) | -32..44 (76) | -5..38 (43) | -48..-5 (43) |
| 59 | -5..608 (613) | -1..494 (495) | -27..467 (494) | -42..314 (356) | -5..883 (888) | -165..587 (752) |
| 60 | -5..380 (385) | -5..587 (592) | -21..468 (489) | -41..333 (374) | -4..411 (415) | -244..675 (919) |
| 61 | 6..371 (365) | 0..321 (321) | -27..47 (74) | -41..-21 (20) | 0..576 (576) | -117..208 (325) |
| 62 | -4..410 (414) | -7..649 (656) | -21..698 (719) | -44..187 (231) | -4..607 (611) | -322..446 (768) |
| 63 | -4..760 (764) | -7..573 (580) | -24..80 (104) | -35..-12 (23) | -2..779 (781) | -268..177 (445) |
| 64 | -5..335 (340) | -2..396 (398) | -20..175 (195) | -41..-19 (22) | -4..876 (880) | -22..89 (111) |

Large contact-like force variation is visible in episodes 56, 59, 60, 62, 63 and 64, with
hundreds of raw-count changes in several channels. Episodes 52–55, 57 and 58 show only small
changes; episode 51 shows none because it ended before manipulation. This is a signal sanity check,
not a contact label or force-in-Newtons claim.

## JAKA freshness and causal selection

The measured JAKA observation timestamp is present and monotonic in every row; no future JAKA
observation was selected. Effective selected observation rates are 19.3–27.3 Hz, below the
configured production compact-status target of approximately 31.25 Hz. Repeated JAKA observations
are therefore expected across canonical rows, but the large canonical slot gaps show that the
dataset did not receive a stable compact status stream at the desired cadence.

The causal angle/force timestamps are monotonic as sequences, but the source-validity failures
listed above are timestamp ordering failures, not timestamp regressions.

## Camera integrity

Both cameras use `rgb_timestamp_domain: global_time`. For every episode, selected host timestamps,
device timestamps and frame numbers were nondecreasing; no host/device/frame regression or ring
seqlock inconsistency was found. MP4 decoding succeeded for both roles. Repeated source frames are
reported below as `workspace / wrist`; they are expected canonical reuse, not automatically camera
loss.

| ep | W age p50/p95/max ms | R age p50/p95/max ms | repeated W/R | descriptor drops W/R | max bad/missing run W/R slots |
|---:|---|---|---:|---:|---:|
| 51 | 31.3/33.5/35.2 | 21.4/23.8/24.8 | 8/0 | 15/9 | 10/10 |
| 52 | 29.2/33.3/36.0 | 16.1/21.4/22.2 | 5/0 | 15/10 | 19/19 |
| 53 | 25.4/32.1/33.5 | 23.7/29.6/31.5 | 2/0 | 15/9 | 19/19 |
| 54 | 20.6/28.8/30.4 | 12.9/21.1/22.7 | 0/0 | 15/9 | 14/14 |
| 55 | 6.8/32.9/34.7 | 18.6/24.5/27.2 | 10/0 | 15/9 | 44/44 |
| 56 | 18.9/31.8/36.2 | 18.4/31.3/34.9 | 8/10 | 15/9 | 36/36 |
| 57 | 7.8/10.7/11.3 | 5.0/7.4/8.3 | 0/0 | 15/9 | 7/7 |
| 58 | 4.2/8.7/41.9 | 26.6/30.5/31.8 | 10/0 | 15/10 | 22/22 |
| 59 | 23.3/31.7/39.3 | 21.8/31.9/33.7 | 5/4 | 15/9 | 26/26 |
| 60 | 19.1/28.2/29.5 | 12.3/22.0/23.1 | 0/0 | 0/0 | 7/7 |
| 61 | 23.4/28.5/29.0 | 17.6/21.4/22.3 | 0/0 | 0/0 | 22/22 |
| 62 | 11.3/32.0/35.5 | 21.1/32.1/34.9 | 15/18 | 0/0 | 128/128 |
| 63 | 5.8/33.0/35.1 | 27.3/32.4/33.0 | 12/0 | 0/0 | 7/7 |
| 64 | 20.2/24.7/25.7 | 13.4/18.7/19.3 | 0/0 | 0/0 | 17/17 |

Frame-number gaps are much larger than recorder drops in most episodes because canonical samples
skip deadlines and therefore skip over already-captured camera frame numbers. They are valid
evidence of sparse canonical selection; without per-frame acquisition logs they cannot be
reinterpreted as the same number of physical RealSense capture losses. The separate descriptor
drop counters are retained above. Camera clock maps from the existing synchronizer were valid for
all episodes, with `global_time` domains, one pair per canonical row, and maximum linear-fit
residuals of approximately 2.4–8.2 ms. No image interpolation was used.

## Recorder and queue health

For all production episodes:

- `queue_full_count = 0`;
- `recorder_dropped_count = 0`;
- `ring_reference_expired_count = 0`;
- `writer_failed_count = 0`;
- both camera processes report alive and both ring consistency/reuse/expiry counters are 0;
- all quality events were persisted.

The metadata does not expose a recorder queue high-watermark, so that metric is **unavailable**, not
zero. The canonical deadline gaps must not be mislabeled as recorder drops.

The production episode metadata has no separate `jaka_state.jsonl` or `rh56_feedback.jsonl` audit
stream. This is consistent with the production de-duplication change; the canonical timing row is
the available source for offline synchronization.

## Quest / teleoperation regression evidence

The staging metadata says `quest_raw_datagram: unavailable` and does not persist per-datagram
right-hand/head validity. Therefore the required “head present when arm reference capture was
requested” assertion cannot be proved from the canonical dataset alone.

The sparse combined event logs provide partial evidence:

- The single-episode session logs for episodes 51–54 contain `ARM_REFERENCE_INPUT_INVALID`
  diagnostics (1, 2, 2 and 9 records respectively), with `arm_reference_capture: false` and
  `captured_head_yaw_rad: null` in those snapshots. This is a real teleoperation-quality warning,
  although the persisted event does not identify the overwritten HTS datagram as the cause.
- The single-session logs for 55, 57 and 58 contain no `ARM_REFERENCE_INPUT_INVALID`; 56 has one
  `HAND_REFERENCE_INPUT_INVALID` in a shared session log.
- The event log referenced by 59–64 is shared by multiple episodes, so its aggregate
  `RIGHT_WRIST_TRACKING_LOST`/controller fault counts cannot be attributed to individual episodes.
- Episode termination metadata still records producer/control liveness failures in 51–55 and 57–58.

Consequently, the new episodes do not provide sufficient per-episode evidence to certify the HTS
latest-only regression as absent. This audit did not alter the teleoperation path.

## Cross-modal temporal alignment and task-phase review

For each episode, the existing synchronizer constructed a 30 Hz timeline, mapped both RealSense
device clocks to host monotonic time, selected latest causal JAKA/ANGLE/FORCE observations, and
selected camera frames within 100 ms without image interpolation. It reported
`future_force_samples_used: false` for all 14 episodes. The row-level source-validity violations
listed earlier remain an independent reason to filter affected canonical rows before training.

Canonical data has no persisted semantic labels for “before grasp”, first bottle contact/closure,
lift, transport, placement, or release. I sampled q10/q30/q50/q70/q90 canonical moments in every
episode rather than inventing phase labels. The timing tables above report the actual age ranges
seen across all rows; the sampled moments show camera ages generally below 40 ms, JAKA ages up to
66 ms, ANGLE ages up to 140 ms, and FORCE ages up to 142 ms except where a source timestamp is
invalid. In episode 62, the 4.299 s FORCE update gap and 4.3 s canonical gap overlap the later
timeline, so phase-specific alignment there is not trustworthy.

## ACT vs ACT+Force readiness

The exact same collected rows have the required fields for both model inputs:

- ACT: images + 12-D `observation.state` + 12-D `action`;
- ACT+Force: the same fields plus 6-D raw-count `observation.force`, force age and causal
  validity.

No separate recollection is needed solely to compare the two model variants. However, the current
episodes are not clean enough for immediate training: exclude aborted episodes, metadata-only
quality slots, canonical deadline gaps, and every row with a false source-validity mask or future
source timestamp. After that filtering, the remaining complete episodes still require manual video
and task-success review; this report does not assign success labels or claim physical safety.

## Audit commands/evidence

Read-only checks used:

```text
existing episode_dataset.synchronization.synchronize_staging_episode
JSONL/metadata/schema/timestamp checks over data/raw_episodes
OpenCV full decode of every production workspace/wrist MP4
ffprobe stream/container inspection
read-only inspection of metadata recorder/camera diagnostics and combined event logs
```

Raw episodes were preserved unchanged.
