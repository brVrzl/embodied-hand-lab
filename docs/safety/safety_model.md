# Safety model

## Fault classes

| Class | Examples | Required behavior |
|---|---|---|
| Input/producer liveness | stale Quest, producer or IPC timeout | stop physical output and clean up |
| Recoverable candidate infeasibility | IK, collision, singularity direction, output velocity/acceleration | `HOLD_REJECTED`, fresh heartbeat, hold last safe target |
| Native/controller hard fault | tracking error, servo alarm, collision, estop, power/enable loss, SDK error, hard timing fault | stop before another point and clean up |
| Operator action | clutch release, bounded gate end, explicit stop | stop/hold per stage and clean up |

The distinction prevents a rejected target from masquerading as dead
communication while ensuring that dead communication cannot masquerade as a
recoverable hold.

## Defense in depth

Shared policy rejects unsafe candidates before constructing
`AcceptedArmTarget`. The native worker independently checks continuity,
velocity, acceleration, tracking, liveness, timing, and controller health as
defensive assertions. Native checks must not silently reshape a target and
thereby make simulation and hardware different.

The startup hold measured after entering EDG is the physical authority.
Reference capture cannot waive startup continuity. Latest-destination
resampling is bounded and causal; rejected or old targets are never queued for
future playback.

## Configuration ownership

Payload, center of mass, installation, TCP, and controller safety limits belong
to controller/operator configuration. Current software reads health/state but
does not silently write these settings. A recorded value is evidence of a
previous operator report, not a guarantee of present controller state.
