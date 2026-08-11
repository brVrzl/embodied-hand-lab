# ACT five-demonstration overfit validation

Result: **PASS for the offline overfit objective**. This is not evidence of task success or generalization.

Both runs used the real LeRobot 0.6.2 ACT model, its real training loop, two ResNet18 camera branches, GPU forward/loss/backward/AdamW update, and its normal checkpoint infrastructure. ImageNet-pretrained backbone weights were disabled to keep the experiment offline and reproducible.

## Tiny pipeline run

| Setting | Value |
|---|---|
| Cameras | workspace + wrist, each RGB `[3,240,320]` |
| State / action | `[12]` / `[16,12]` |
| Architecture | ACT VAE, model dim 128, 4 heads, FFN 512, transformer encoder 2, decoder 1, VAE encoder 2, latent 16 |
| Backbone | ResNet18 per camera, no pretrained weights |
| Batch / steps | 8 / 300 |
| Optimizer | AdamW, policy and backbone LR `1e-4`, weight decay `1e-4`, gradient clip 10 |
| Precision / seed | fp32 / 1000, deterministic settings |
| Parameters | 12,305,772 trainable |
| GPU memory / throughput | 0.52 GB logged; 70 samples/s at final log |

The first logged total loss was 3.025 at step 10 (`L1=0.894`, `KL=2.130`). The final logged loss was 0.511 at step 300 (`L1=0.445`, `KL=0.066`); minimum was 0.496. A fresh checkpoint evaluation produced finite `[3268,16,12]` output and native-space MAE 0.076123. This established the complete training chain before the stronger run.

Tiny artifacts are under `outputs/act/physical_bottle_5demo_validation_20260811_150941/tiny_run`, `tiny_analysis`, and `tiny_evaluation`.

## Strong five-demo overfit

| Setting | Value |
|---|---|
| Architecture | ACT VAE, model dim 256, 8 heads, FFN 1024, transformer encoder 4, decoder 1, VAE encoder 4, latent 32 |
| Backbone | ResNet18 per camera, no pretrained weights |
| Chunk / executed action steps | 16 / 16 |
| Batch / steps | 16 / 2,000 |
| Optimizer | AdamW, policy and backbone LR `1e-4`, weight decay `1e-4`, gradient clip 10; no scheduler |
| VAE / regularization | KL weight 1.0, dropout 0.1 |
| Precision / seed | fp32 / 1000, deterministic settings |
| Parameters | 18,713,100 trainable |
| Runtime | 7 min 22 s |
| GPU memory / throughput | 1.17 GB logged; 76 samples/s at final log |

Loss decreased substantially: step 20 total 2.696 (`L1=0.890`, `KL=1.806`) to step 2,000 total 0.095 (`L1=0.089`, `KL=0.006`), with minimum 0.095. The full loss history and plot are `strong_analysis/loss_history.json` and `strong_analysis/loss_history.png` beneath the experiment directory.

## Training-trajectory inference

Fresh-process evaluation covered all 3,268 frames and 51,688 unpadded future action targets. Errors are after the checkpoint postprocessor has restored native absolute action units.

| Scope | MAE | MSE |
|---|---:|---:|
| all 12 channels | 0.0167138 | 0.000755601 |
| JAKA 1-6 | 0.0217788 | 0.000832365 |
| RH56 1-6 | 0.0116489 | 0.000678836 |

| Source episode | Total MAE | Total MSE | JAKA MAE | RH56 MAE |
|---:|---:|---:|---:|---:|
| 65 | 0.0170595 | 0.000790440 | 0.0224858 | 0.0116332 |
| 66 | 0.0179218 | 0.000785113 | 0.0238028 | 0.0120407 |
| 67 | 0.0156835 | 0.000722187 | 0.0195170 | 0.0118499 |
| 69 | 0.0162799 | 0.000697809 | 0.0190748 | 0.0134849 |
| 70 | 0.0171368 | 0.000785854 | 0.0245263 | 0.00974728 |

| Dimension | MAE | MSE |
|---|---:|---:|
| JAKA 1 | 0.0237032 | 0.000935816 |
| JAKA 2 | 0.0178738 | 0.000571639 |
| JAKA 3 | 0.0271334 | 0.00136142 |
| JAKA 4 | 0.0202930 | 0.000663942 |
| JAKA 5 | 0.0206978 | 0.000721808 |
| JAKA 6 | 0.0209714 | 0.000739534 |
| RH56 index | 0.0106627 | 0.000757876 |
| RH56 middle | 0.0116435 | 0.000901446 |
| RH56 ring | 0.0137451 | 0.000880941 |
| RH56 pinky | 0.0106170 | 0.000551074 |
| RH56 thumb-close | 0.0104009 | 0.000439024 |
| RH56 thumb-lateral | 0.0128241 | 0.000542582 |

MAE by horizon 0-15 was `[0.0191280, 0.0181874, 0.0173690, 0.0166697, 0.0161576, 0.0157708, 0.0155668, 0.0152901, 0.0152194, 0.0153206, 0.0155094, 0.0159421, 0.0165243, 0.0172944, 0.0182094, 0.0192591]`. Padded targets were excluded.

## Temporal behavior and action envelope

Separate six-channel JAKA and six-channel RH56 chunk plots were produced for relative-progress proxies named approach, grasp, lift, transport, placement, and release. The dataset has no ground-truth phase labels, so these names are qualitative inspection anchors only. The plots under `strong_evaluation` show the learned chunks tracking demonstrated direction, scale, ordering, and temporal shape; errors remain low in all five episodes. No sign inversion, channel permutation, gross scale mismatch, or output saturation was observed.

Predictions were not clamped. Their global envelope reveals small but real extrapolation:

| Dim | Train min | Train max | Pred min | Pred max | >5% range excursion |
|---|---:|---:|---:|---:|---|
| JAKA 1 | 0.557077 | 1.504913 | 0.513835 | 1.584322 | high |
| JAKA 2 | -1.376594 | -0.578616 | -1.411803 | -0.532818 | high |
| JAKA 3 | -2.154594 | -0.848742 | -2.222833 | -0.802701 | low |
| JAKA 4 | -4.188760 | -3.417762 | -4.239478 | -3.389218 | low |
| JAKA 5 | -1.596522 | -0.763582 | -1.603976 | -0.723382 | none |
| JAKA 6 | 4.423104 | 5.524969 | 4.465399 | 5.557484 | none |
| RH56 index | 0.000000 | 0.409860 | -0.013438 | 0.414535 | none |
| RH56 middle | 0.000000 | 0.447497 | -0.011051 | 0.466256 | none |
| RH56 ring | 0.000000 | 0.486776 | -0.016918 | 0.504647 | none |
| RH56 pinky | 0.000000 | 0.402500 | -0.007378 | 0.407941 | none |
| RH56 thumb-close | 0.000000 | 0.312500 | -0.019321 | 0.242714 | low |
| RH56 thumb-lateral | 0.423204 | 0.944051 | 0.400683 | 0.944468 | none |

Here “significant” is the analysis threshold of more than 5% of that channel's demonstrated range. Small negative RH56 predictions are also visible even where they do not cross that threshold. These observations do not invalidate offline serialization or the overfit result, but they must be handled by the existing future command-safety boundary before any physical rollout; no clamp was added to make the offline result look better.

## Checkpoint and determinism

The final checkpoint is `outputs/act/physical_bottle_5demo_validation_20260811_150941/strong_run/checkpoints/002000/pretrained_model`. It contains the policy config, training config, model safetensors, preprocessor config/stat tensors, and postprocessor config/stat tensors. In a fresh process, LeRobot reloaded all of these from disk, emitted finite `[3268,16,12]` native-space predictions, and required no hidden in-memory statistics. Two deterministic evaluations were bit-identical (`max_abs_diff=0.0`). A second independent integration-test reload also matched exactly.

Complete numeric results and arrays are in `strong_evaluation/evaluation_metrics.json` and `strong_evaluation/evaluation_arrays.npz`.

## Reproduction commands

Run these inside the pinned LeRobot container documented in `act_thor_environment.md`, from `/workspace/embodied_lab`. They create only ignored files beneath `outputs/`:

```bash
python tools/act_physical_bottle_validation.py build-view \
  --master data/training/physical_bottle_v1/master \
  --view outputs/act/physical_bottle_5demo_reproduction/lerobot_v3_view

python tools/act_physical_bottle_validation.py inspect-loader \
  --master data/training/physical_bottle_v1/master \
  --view outputs/act/physical_bottle_5demo_reproduction/lerobot_v3_view \
  --output outputs/act/physical_bottle_5demo_reproduction/validation

python -m lerobot.scripts.lerobot_train \
  --config_path=configs/training/act_physical_bottle_lerobot_tiny.json

python -m lerobot.scripts.lerobot_train \
  --config_path=configs/training/act_physical_bottle_lerobot_strong.json

python tools/act_physical_bottle_validation.py evaluate \
  --view outputs/act/physical_bottle_5demo_reproduction/lerobot_v3_view \
  --checkpoint outputs/act/physical_bottle_5demo_reproduction/strong_run/checkpoints/002000/pretrained_model \
  --output outputs/act/physical_bottle_5demo_reproduction/strong_evaluation
```

The two checked-in JSON files are directly parsed by LeRobot's `--config_path` mechanism and encode the exact tiny/strong architecture, normalization, optimizer, batch, seed, and step settings used in validation. The output root is deliberately changed from the completed run's timestamped directory so reproduction cannot overwrite its local evidence.
