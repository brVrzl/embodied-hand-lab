# Thor ACT command-disabled shadow benchmark

Benchmark date: 2026-08-11 (Asia/Shanghai)

## Verdict

**ACT SHADOW: `PASS`.** The maintained LeRobot 0.6.2 ACT checkpoint completed 300 live, command-disabled queries at 30 Hz with finite native-space outputs of shape `[16, 12]`. No JAKA or RH56 write API was called.

This is an observation/inference timing result only. It is not evidence of physical task success and does not authorize robot control.

## Runtime and checkpoint

- Machine: NVIDIA Jetson AGX Thor (`thor`), CUDA-enabled LeRobot container `jaka-lerobot-dev:snapshot-before-raw-mount`.
- Checkpoint: `outputs/act/physical_bottle_5demo_validation_20260811_150941/strong_run/checkpoints/002000/pretrained_model`.
- Implementation: maintained LeRobot 0.6.2 `ACTPolicy`, using the checkpoint's saved preprocessors/postprocessors and normalization statistics.
- ACT configuration: prediction chunk `H=16`, saved `n_action_steps=16`, two RGB cameras, 12-D measured state, 12-D native/absolute action.
- GPU memory: peak PyTorch allocated 100,354,048 bytes; reserved 125,829,120 bytes.
- Active-window GPU utilization: mean 59.7%, peak 80%; GPU temperature mean 41.3 C, max 41.4 C; measured module power mean 6.07 W, max 6.30 W. No thermal-throttling indication was observed (`nvidia-smi dmon` did not expose a supported throttle-violation counter on this platform).
- Active-window CPU use: mean approximately 18.8% across reported cores; maximum one-second all-core mean approximately 21.4%.

## Strictly read-only live adapter

`tools/act_live_shadow.py` reads:

- workspace RealSense D435 serial `346522072675`;
- wrist RealSense D435 serial `346222072985`;
- JAKA measured joints from the read-only SDK diagnostic stream;
- RH56 `ANGLE_ACT`; optional `FORCE_ACT`, `CURRENT`, `STATUS`, and `ERROR` for logging.

Both cameras acquire `RGB8` at 640x480 and are resized once to 320x240 using OpenCV `INTER_AREA`, converted to CHW float32, and scaled by 1/255. A regression test proves this path is pixel-identical to the recorded BGR-to-RGB preprocessing path, preventing a second channel swap. State order is JAKA joints 1–6 followed by canonical RH56 normalized positions 1–6.

The process boundary is deliberate: the live process has only read-only device objects and sends observations over a Unix socket to `tools/act_shadow_model_worker.py`. Predictions are serialized to disk and never forwarded to either controller. A source safeguard test rejects forbidden motion/write call sites in the live adapter.

## Benchmark method and artifacts

- 20 unreported warm-up queries, then 300 measured policy queries.
- Requested query cadence: 30 Hz (**exploratory diagnostic value**, chosen to stress the full dataset-rate path).
- Result: 30.000003 queries/s.
- Primary artifact: `outputs/force_shadow_20260811/live_shadow_300q_final/shadow_benchmark.json`.
- Predictions: `outputs/force_shadow_20260811/live_shadow_300q_final/shadow_predictions.npz`.
- Model metrics: `outputs/force_shadow_20260811/live_runtime_2/model_worker_summary.json`.
- Resource logs: `outputs/force_shadow_20260811/live_runtime_2/nvidia_dmon.log` and `tegrastats.log`.

## Latency

All values are milliseconds.

| Stage | p50 | p90 | p95 | p99 | max |
|---|---:|---:|---:|---:|---:|
| observation acquisition | 0.160 | 2.598 | 3.042 | 4.243 | 7.399 |
| image preprocessing | 1.294 | 2.133 | 2.162 | 2.631 | 2.749 |
| state assembly | 0.034 | 0.040 | 0.042 | 0.046 | 0.052 |
| checkpoint preprocessing | 1.041 | 1.145 | 1.157 | 1.211 | 1.411 |
| ACT model inference | 14.842 | 15.080 | 15.168 | 15.910 | 18.048 |
| checkpoint postprocessing | 0.194 | 0.216 | 0.235 | 0.273 | 0.317 |
| model IPC round trip | 19.894 | 21.896 | 22.161 | 22.871 | 23.189 |
| total observation-to-native-action | 21.751 | 24.498 | 26.615 | 27.942 | 31.617 |

At a 30 Hz deadline of 33.333 ms, the measured total p99 margin is 5.391 ms and worst-observed margin is 1.717 ms. This supports measurement at 30 Hz but is a thin maximum-latency margin for a first controller rollout.

## Input freshness and capture health

| Input | p99 age at observation assembly | maximum age | stale count (>2 nominal periods) |
|---|---:|---:|---:|
| workspace image | 7.425 ms | 7.866 ms | 0 |
| wrist image | 6.746 ms | 7.273 ms | 0 |
| JAKA joints | 35.906 ms | 36.937 ms | 0 |
| RH56 angle | 66.258 ms | 67.725 ms | 0 |

Camera skew p99/max was 1.629/2.195 ms. Workspace captured 337 frames at 30.262 Hz with one detected late/gap event; wrist captured 360 at 30.159 Hz with none. JAKA produced 386 successful samples at 30 Hz, with no read failure/timeout and no active motion/collision/emergency-stop indication. RH56 recorded 189 angle reads and 126 reads of each 10 Hz auxiliary register, with zero writes. Repeated RH56 state timestamps on 149/300 queries are expected when a 30 Hz policy observes a 15 Hz source and are not concealed by interpolation.

## Prediction integrity

- Shape: `[300, 16, 12]`; finite-value failures: 0.
- Repeated inference on one fixed observation with deterministic settings was bit-identical; maximum absolute difference was 0.
- Native JAKA prediction ranges by channel:
  - min `[1.398222, -1.196650, -1.731821, -3.399659, -1.436705, 4.677517]`
  - max `[1.403777, -1.191881, -1.725714, -3.394265, -1.433977, 4.682651]`
- Native RH56 prediction ranges by channel:
  - min `[0.016162, 0.011056, 0.064595, 0.041959, 0.006077, 0.610273]`
  - max `[0.019510, 0.013992, 0.068783, 0.045930, 0.007302, 0.613102]`
- Authoritative hardware-limit excursions: 0 scalar predictions.
- Training-envelope excursions: 8.3333% of scalar predictions. This is entirely JAKA channel 4: all its shadow predictions were about 0.018–0.024 rad above the demonstration maximum of -3.417761 rad. Every other channel had zero excursions. No values were clamped.
- Mean absolute first-step change between consecutive queries ranged from 0.000072 to 0.000440 across channels; maximum per-channel changes ranged from 0.000231 to 0.001641. Mean chunk-overlap alignment MAE was 0.000282.

The JAKA-4 demonstration-envelope result requires attention during later rollout review, but it is within the configured hardware command limit and does not invalidate the command-disabled inference pipeline.
