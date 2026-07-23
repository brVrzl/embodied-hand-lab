# Troubleshooting

## No Quest engagement

- Confirm the Unity build includes CTRL v1 `LeftControllerPacketSender`.
- Confirm host address/UDP port match and the firewall allows the intended
  interface.
- Release the left index fully, then press again with fresh wrist/head packets.
- Check controller age (150 ms policy) and wrist/head age (250 ms policy).
- Do not substitute SPACE-key instructions from an old prototype.

## `HOLD_REJECTED`

This is a feasible-candidate hold, not liveness failure. Inspect the recorded
reason: IK residual, continuation, collision, branch jump, limits, Jacobian
quality, output velocity, or output acceleration. Hold still or retreat through
the last safe direction. Do not raise a boundary merely to remove the message.

J5 proximity is warning metadata; actual Jacobian quality is authoritative.

## Simulation viewer

Run without `--viewer` to separate control/replay from display problems. Over
SSH, provide the actual local `DISPLAY` and `XAUTHORITY`; never copy a stale
username or absolute path from a dated handoff.

## Native worker does not connect

Stop. Do not retry multiple SDK sessions. Confirm another client or stale
process is not logged in. The current design intentionally uses one JAKA SDK
session because a prior second-session health monitor prevented the primary
worker reaching `CONNECTED`.

## Timing, tracking, or controller fault

Treat hard timing faults, tracking errors, servo/collision alarms, estop,
power/enable loss, or SDK errors as hard stops. Preserve metrics and raw logs,
record the exact commit/config, and follow
[incident response](../safety/incident_response.md). Do not resume the same
physical envelope merely because cleanup succeeded.
