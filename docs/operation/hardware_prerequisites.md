# Physical hardware prerequisites

This page is a prerequisite checklist, not authorization to run hardware.
Every physical gate needs explicit user authorization in a new/current session
for its exact stage, duration, motion bound, and acknowledgement flags.

Before any connection or command:

1. Confirm the intended worktree, branch, commit, clean task-owned diff, config,
   and executable hashes.
2. Confirm no other control client is logged in and no stale process remains.
3. Inspect the robot, hand, cabling, workspace, fixtures, and stop access.
4. Confirm controller state, power/enable, emergency stop, alarms, collision
   state, payload, installation, TCP, and safety limits at the controller.
5. Confirm the operator understands the displacement/orientation envelope,
   clutch, stop conditions, and abort procedure.
6. Perform only the separately authorized read-only/no-motion stages before
   considering motion.

Recorded operator-supplied state from the latest evidence:

| Item | Recorded value | Ownership |
|---|---|---|
| Payload mass | 0.8 kg | controller/operator state |
| Center of mass | `[9.289, 12.427, 36.961]` mm | controller/operator state |
| Installation | upright/floor; X=0°, Z=0° | controller/operator state |
| TCP1–TCP10 | zero | controller/operator state |
| Safety limits | unchanged | controller/operator state |

These values are not hard-coded runtime truth. The software must not silently
write them, and a future operator must verify them before motion.

Do not repeat the earlier approximately 128 mm multi-axis translation with
large wrist motion. Do not use a maintenance session to enter servo/EDG,
identify payload, calibrate TCP, or change controller settings.
