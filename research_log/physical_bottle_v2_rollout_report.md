# Physical ACT / ACT+Force pilot rollout report

Date: 2026-08-12 (Asia/Shanghai)

This report covers one command-enabled physical run for each checkpoint. The
scene was reported reset before the ACT+Force run. The rollout tool was
bounded to 30 s, queried the policy at 15 Hz, sent the first two actions from
each 16-action chunk at 30 Hz, and logged FORCE_ACT without using it in the
standard ACT run.

## Result

| run | policy input | queries | accepted commands | inference p50/p95/max (ms) | camera FPS (workspace / wrist) | RH56 projection events | native safety counters | task label |
|---|---|---:|---:|---:|---:|---:|---|---|
| ACT | images + 12-D state | 451 | 901 | 15.615 / 22.550 / 28.218 | 30.091 / 30.026 | 76 (`thumb_close`) | rejected 0, timing misses 0, hard misses 0, tracking crossings 0, controller alarms 0 | `UNLABELED` |
| ACT+Force | images + 12-D state + raw 6-D FORCE_ACT | 451 | 901 | 12.843 / 17.995 / 25.998 | 30.108 / 30.045 | 42 (`thumb_close` 30, `middle` 12) | rejected 0, timing misses 1, hard misses 0, tracking crossings 0, controller alarms 0 | `UNLABELED` |

Both runs produced finite `[16, 12]` chunks and no inference failure. The
ACT+Force worker confirmed `observation.environment_state` with shape `[6]`;
force was not concatenated into the 12-D state. Its 42 RH56 projections were
recorded at the final policy-to-command boundary and were small boundary
projections of normalized hand outputs.

The physical task result is not classified as success or failure: the
operator label was left `UNLABELED`, and the saved rollout should not be used
to infer policy quality. Preserve both trajectories for review:

- [ACT rollout](../outputs/act_physical_rollouts/20260812_act/rollout_summary.json)
- [ACT+Force rollout](../outputs/act_physical_rollouts/20260812_act_force/rollout_summary.json)

## Findings and fixes

1. The rollout command loop and the inference thread both read the same
   latest-only JAKA status socket. The command-loop read used only to write
   `commands.jsonl` could consume the compact status packet before the policy
   snapshot. The two physical runs therefore show the startup JAKA
   observation timestamp repeated for most command rows. This is a real
   deployment freshness defect; these runs are not valid evidence that the
   policy operated with fresh JAKA state.

   Fixed in `tools/act_physical_rollout.py`: command logging now reuses the
   status snapshot already attached to the policy observation and no longer
   drains the receiver. This does not alter target generation, JAKA limits,
   native watchdogs, RH56 polling, or safety checks.

2. The pre-fix cleanup waited on camera/inference/RH56 shutdown before sending
   the JAKA stop packet. The native worker consequently reported
   `command_stream_timeout` at the end of both 30 s windows. It was not a
   mid-run controller, tracking, collision, hard timing, or RH56 fault: all
   901 targets were accepted and the timeout occurred only during wrapper
   shutdown. Cleanup now sends the JAKA controlled-stop packet and stops the
   native worker before waiting on auxiliary cleanup. This change has not yet
   received a new command-enabled physical validation.

3. `tools/act_live_shadow.py` now accepts the repository's native LeRobot
   `meta/stats.json` action entry directly, in addition to its previous
   envelope forms.

## Command-disabled post-fix check

The ACT+Force shadow check after these code changes completed 300 queries at
15.0000 Hz with output shape `[300, 16, 12]`, zero finite-output failures,
zero RH56 writes, and zero stale-source counts. Workspace/wrist capture was
30.120/30.052 Hz; one workspace frame-number gap was observed in this
read-only check. End-to-end latency was p50/p95/p99/max =
26.474/29.608/31.970/34.062 ms. The ACT+Force model input flag was true.

Artifact: [post-fix shadow result](../outputs/act_physical_rollouts/20260812_postfix_shadow_act_force/result2/shadow_benchmark.json)

This shadow run verifies the model/input and read-only device path. It cannot
by itself prove the physical rollout status-socket consumer fix; that needs a
new bounded command-enabled run.

## Validation

- Focused rollout/native tests: **39 passed**.
- Full repository suite after the fixes: **670 passed, 4 skipped**.
- `git diff --check`: passed after the edits.

## Current decision

**ACT PHYSICAL BASELINE: PARTIAL**
**ACT+Force PHYSICAL BASELINE: PARTIAL**

The deployment/inference path executed and hardware safety counters remained
clean, but neither run should be counted as a task-success trial because the
operator label is absent and the pre-fix JAKA status-consumer race invalidates
fresh-state interpretation. The next physical action should be one bounded
ACT+Force revalidation after scene reset, followed by manual stage/success
labeling; do not tune the policy from these two runs.
