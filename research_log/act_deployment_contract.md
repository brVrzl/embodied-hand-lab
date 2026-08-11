# Future ACT robot-deployment contract

This is a future integration contract, not authorization to command hardware. No physical inference or robot access occurred during validation.

## Required observation at each policy query

The checkpoint has exactly three observation features:

1. `observation.images.workspace`: workspace camera RGB image.
2. `observation.images.wrist`: wrist camera RGB image.
3. `observation.state`: native measured state `[12]`.

The deployed camera adapter must preserve those identities. It must determine the camera API's actual color convention, convert to RGB exactly once, resize the source 640x480 image to 320x240 with area resampling, and supply CHW `[3,240,320]`. The saved LeRobot preprocessor converts uint8 image values to `[0,1]` and applies ImageNet mean `[0.485,0.456,0.406]` and std `[0.229,0.224,0.225]`. There is no crop or random transform.

The state vector order is immutable:

```text
[JAKA measured joint 1, JAKA measured joint 2, JAKA measured joint 3,
 JAKA measured joint 4, JAKA measured joint 5, JAKA measured joint 6,
 RH56 measured index, RH56 measured middle, RH56 measured ring,
 RH56 measured pinky, RH56 measured thumb-close,
 RH56 measured thumb-lateral]
```

The saved processor applies the five-demo per-dimension state mean/std. Deployment must load `policy_preprocessor.json` and `policy_preprocessor_step_3_normalizer_processor.safetensors` from the checkpoint; it must not recompute statistics online or depend on process-local training state.

## ACT output

The raw policy head represents a future chunk `[H,12]` with `H=16`. The saved postprocessor must be applied to inverse-normalize actions into the following exact native ordering:

```text
[JAKA accepted absolute/native target 1 ... target 6,
 RH56 accepted absolute/native index, middle, ring, pinky,
 thumb-close, thumb-lateral targets]
```

These are absolute/native targets. They are not deltas, velocities, or reordered joints. Do not add the current state, integrate the values, or infer semantics from a different robot. The saved file `policy_postprocessor_step_0_unnormalizer_processor.safetensors` is part of the deployment artifact.

Offline ACT adds no action clamp, smoothing, channel conversion, or unit conversion after inverse normalization. Predictions showed modest excursions outside the demonstrated envelope, including >5%-of-range excursions on JAKA 1-4 and RH56 thumb-close by the analysis criterion, plus small negative values on several RH56 channels. Before hardware use, the existing command interface's authoritative limits and safety validation must reject or safely handle invalid native absolute targets. That safety behavior must be validated separately; it must not be silently folded into model semantics.

## Timing and recurrent/episode state

- Dataset rate and current control-frequency assumption: 30 Hz.
- Chunk: 16 targets covering offsets 0 through 0.5 seconds.
- `n_action_steps=16`: LeRobot's normal `select_action` path queues all 16 actions, then queries the policy again after 16 control ticks. At an uninterrupted 30 Hz loop this is approximately 1.875 model queries/s and 0.533 seconds per queued chunk.
- Temporal ensembling: disabled (`temporal_ensemble_coeff=None`).
- Observation history: one observation (`n_obs_steps=1`).
- Episode/reset initialization: call `policy.reset()` before the first observation of every episode and on abort/reset. This clears the queued chunk; do not carry queued targets across an episode boundary.

The offline work did not establish that executing all 16 open-loop targets is the best physical control choice. Changing query frequency, `n_action_steps`, interpolation, or temporal ensembling would be a deliberate new control configuration requiring offline evaluation and a separate safety review.

## Artifact load sequence

1. Load the policy from `outputs/act/physical_bottle_5demo_validation_20260811_150941/strong_run/checkpoints/002000/pretrained_model`.
2. Load the saved preprocessor and postprocessor from the same directory.
3. Put the policy in evaluation mode, use deterministic inference settings, and call `policy.reset()`.
4. Form the two correctly identified RGB tensors and the ordered native state vector.
5. Apply the saved preprocessor, infer `[16,12]`, and apply the saved postprocessor.
6. Confirm finite native targets and pass each target through the existing, separately validated command-safety boundary before any command is considered.

Do not load only `model.safetensors`: config plus both serialized processor states are required to reproduce the validated native-space behavior.

## Remaining gates before a first physical rollout

The following work remains outside this task:

1. Implement a read-only live observation adapter that proves workspace/wrist identity, RGB conversion, 320x240 resizing, and the exact 12-D measured-state ordering without enabling commands.
2. Integrate checkpoint pre/postprocessors and compare that adapter against recorded frames offline.
3. Define and test the existing command boundary's authoritative JAKA/RH56 limits, finite-value rejection, timing/watchdog, abort, and reset behavior, explicitly covering the measured envelope excursions. Do not disguise this as model clamping.
4. Benchmark inference latency and queue timing in the final deployment process, then make an explicit decision on 16-action open-loop execution versus a separately validated query/ensemble configuration.
5. Conduct a reviewed, command-disabled shadow run and record observations/predictions only.
6. Obtain separate human authorization, physical workspace checks, conservative safety limits, and an emergency-stop operator before enabling the first physical ACT command.

Nothing in this offline overfit test claims the bottle task will succeed on hardware.
