# ACT pipeline validation summary

Final result: **TRAINING PIPELINE PASS**

The maintained LeRobot ACT path completed dataset loading, GPU forward/loss/backward/optimizer, substantial five-demo overfitting, checkpoint serialization, fresh-process reload, deterministic offline inference, and native absolute-action reconstruction. No robot was accessed, no physical inference was run, and no raw/master episode was changed.

## Validated configuration and result

| Item | Result |
|---|---|
| Host/GPU | NVIDIA Jetson AGX Thor Developer Kit; NVIDIA Thor GPU |
| Software | LeRobot 0.6.2, official checkout commit `f66e5128ecb2456e8c54a63d15404fa59c16aebc`; PyTorch `2.13.0a0+8145d630e8.nv26.06` |
| Episodes | exactly 65, 66, 67, 69, 70; 3,268 frames |
| Task | `place_bottle_on_cardboard_box` |
| Inputs | workspace RGB + wrist RGB `[3,240,320]`; native state `[12]`; no force |
| Target | native absolute action chunks `[16,12]` |
| Strong model | ResNet18 ACT VAE, dim 256, 8 heads, FFN 1024, encoder 4, decoder 1, VAE encoder 4, latent 32; 18,713,100 parameters |
| Training | batch 16, fp32, AdamW LR `1e-4`, 2,000 steps, seed 1000 |
| Loss | 2.696 at first log (step 20) to 0.095 at step 2,000; minimum 0.095 |
| Native error | MAE 0.0167138; MSE 0.000755601 |
| JAKA error | MAE 0.0217788; MSE 0.000832365 |
| RH56 error | MAE 0.0116489; MSE 0.000678836 |
| Checkpoint | `outputs/act/physical_bottle_5demo_validation_20260811_150941/strong_run/checkpoints/002000/pretrained_model` |
| Reload/inference | fresh process PASS; finite `[3268,16,12]`; duplicate deterministic run exactly equal |
| Semantic round trip | PASS; max native reconstruction error `5.960464477539063e-08` |

## Acceptance criteria

| # | Criterion | Evidence/result |
|---:|---|---|
| 1 | Maintained ACT used | PASS: official LeRobot 0.6.2 ACT trainer/policy |
| 2 | Five clean demos load | PASS: all 3,268 rows through `LeRobotDataset` |
| 3 | Episodes 68/71 absent | PASS |
| 4 | State/action semantics verified | PASS: JAKA 1-6 then RH56 1-6; absolute/native actions |
| 5 | Normalization round trip | PASS: max error `5.96e-08` |
| 6 | GPU forward/backward | PASS: 300-step tiny and 2,000-step strong runs |
| 7 | Loss substantially decreases | PASS: strong 2.696 to 0.095 |
| 8 | Checkpoint save/reload | PASS, including processors/config from disk |
| 9 | Finite `[H,12]` inference | PASS with `H=16` |
| 10 | Native inverse transform | PASS; no delta/reorder conversion |
| 11 | No episode/segment crossing | PASS; exhaustive loader check and padding mask |
| 12 | JAKA/RH56 fit training data | PASS: low errors in every included episode and all dimensions |

The output-envelope audit found small real extrapolations. More-than-5%-of-demonstrated-range excursions occurred on one side of JAKA 1-4 and RH56 thumb-close; several RH56 predictions were slightly negative. No values were artificially clamped. This does not break the offline pipeline acceptance criteria, but the model is not approved for physical commands until the deployment-contract safety gates are completed.

## Safeguards and tests

The integration tests in `tests/test_act_physical_bottle_lerobot_integration.py` cover exact 12-D state/action names and ordering, excluded episodes, exhaustive episode/segment chunk boundaries, the real processor normalization round trip, checkpoint config/stat reload, the exact workspace/wrist input feature identities, finite `[16,12]` output, and native inverse transformation. Results:

```text
container: 4 tests, OK
host pytest: 3 passed, 4 skipped (the real LeRobot tests intentionally require the container's PyTorch environment)
Python compile checks: PASS
git diff whitespace check for new implementation/tests/reports: PASS
```

The master dataset tree checksum was identical before and after validation: `e8aa4e06c30fcba0c382cbcb6ecb057c7f09c03599dd8c620590864a818bee3e`.

## Artifact index

- Environment: `research_log/act_thor_environment.md`
- Loader: `research_log/act_real_loader_validation.md`
- Normalization/semantics: `research_log/act_normalization_and_action_semantics.md`
- Training/evaluation: `research_log/act_five_demo_overfit.md`
- Future deployment: `research_log/act_deployment_contract.md`
- Machine-readable loader evidence: `outputs/act/physical_bottle_5demo_validation_20260811_150941/validation/loader_and_normalization_validation.json`
- Loss history/plot: `outputs/act/physical_bottle_5demo_validation_20260811_150941/strong_analysis/`
- Metrics, arrays, horizon plot, and 12 temporal plots: `outputs/act/physical_bottle_5demo_validation_20260811_150941/strong_evaluation/`

Before a first physical rollout, complete the six gates in `act_deployment_contract.md`: command-disabled live input verification, offline adapter parity, authoritative command-safety validation for envelope excursions, latency/chunk-timing decision, reviewed shadow run, and separate human authorization plus physical safety preparation. No physical success/generalization claim follows from this intentional same-trajectory overfit test.
