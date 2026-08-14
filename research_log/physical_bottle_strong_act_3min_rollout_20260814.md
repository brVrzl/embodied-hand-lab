# Strong ACT 180-second rollout — 2026-08-14

## Result

The requested continuous 180-second rollout was started with the approved
configuration:

- Strong ACT checkpoint: `outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/100000/pretrained_model`
- policy query: 25 Hz
- command/control: 30 Hz
- executor: async absolute-time temporal ensemble, `m = 0.01`
- chunk/action: dynamically validated `[60, 12]`
- safety and freshness settings: unchanged

The run was automatically stopped after approximately 88.06 seconds by the
existing native hard timing gate. It is not a successful 180-second
qualification and was not retried.

## Exact stop evidence

The Python-side abort was:

`RuntimeError: JAKA target publication failed`

This was a consequence of the native worker stopping first, not the primary
fault. Native metrics report:

- `stop_classification = hard_timing_fault`
- `terminal_timing_fault.phase = cycle_completion`
- completion lateness: approximately 12.23 ms
- hard completion threshold: 12.0 ms
- `hard_timing_misses = 1`
- `missed_deadlines = 4`
- `tracking_hard_crossings = 0`
- controller alarms: 0
- controller error code: 0
- emergency stop: false
- RH56 serial stale-command drops: 0
- RH56 serial writes: 2,554 successful

The safety path therefore stopped the run as designed. The current evidence
does not identify why that one ServoJ cycle exceeded the completion limit;
the likely class is a transient native/runtime scheduling or SDK-cycle
overrun, but this report does not claim a more specific root cause.

## Data retained

The partial artifact is preserved at:

`outputs/act_physical_rollouts/20260814_strong_act_3min_002/`

It contains approximately 88.0 seconds of synchronized workspace/wrist video,
2,639 command rows, 2,202 policy queries, full `[60,12]` chunks, temporal
contributors, selected/post-safety/written targets, measured state, and native
metrics. Camera frame-number gaps were zero in the completed artifact; the
workspace and wrist rates were approximately 30.016 and 29.993 Hz.

The video is a partial qualification record only. No task success/failure
label is assigned here without a full synchronized stage review.
