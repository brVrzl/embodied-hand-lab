# JAKA joint-6 +5-degree validation plan

Date: 2026-07-16  
Status: successfully completed and physically accepted

## Scope and state

The preceding 0.25-degree Gate 3C test is accepted as successful. This plan is
for one additional, clearly observable joint-space validation only. It contains
no Cartesian command, TeleDex, Quest, or RH56 control.

The read-only planning executable has no EDG, servo-mode, power/enable, or
motion-command symbol. It completed login, countdown, fresh state capture,
validation, logout, and process termination with zero commands.

Fresh start:

`[1.56975867764, -0.0283112178407, -0.770829198415, 0.505283204717, 0.134604423998, 0.47423723495]` rad.

Proposed outward target:

`[1.56975867764, -0.0283112178407, -0.770829198415, 0.505283204717, 0.134604423998, 0.561503697549]` rad.

Only joint 6 changes: `+0.087266462600 rad` / `+5 degrees`. The physical
executable must capture a new start after its own countdown; these values are
review evidence, not a future command target.

## Motion profile

The same seventh-order endpoint-stationary polynomial is used over 5 seconds
outward and 5 seconds return, with a 1-second outward hold and 0.5-second
settling observation after return.

| Quantity | Plan |
|---|---:|
| Average outward speed | 1.0 deg/s |
| Peak velocity | 0.0381790774 rad/s / 2.1875 deg/s |
| Peak acceleration | 0.0262259750 rad/s² / 1.50264 deg/s² |
| Peak jerk | 0.0366519143 rad/s³ / 2.10 deg/s³ |
| Requested period | 8 ms / 125 Hz |
| Outward / hold / return / settle | 5.0 / 1.0 / 5.0 / 0.5 s |

Position, velocity, acceleration, and jerk are zero/stationary at trajectory
endpoints. No position step or automatic repetition is permitted.

## Tracking and envelope policy

- Tracking-quality warning: 0.0034906585 rad / 0.2 degrees. Warning only.
- Expected lag per cycle: `abs(commanded_velocity) * 0.150 s`.
- Dynamic hard threshold per cycle:
  `max(0.0130899694 rad, 2.5 * expected_lag)`.
- Dynamic threshold range over this plan: 0.75 to 0.8203125 degrees
  (0.0130899694 to 0.0143171540 rad).
- One crossing is recorded and rechecked; two consecutive crossings abort.
- Raw divergence increasing by at least 0.25 degrees in one cycle while already
  above the warning threshold is classified as rapid and aborts immediately.
- Observed joint-6 finite-difference speed above 3.5 deg/s is an immediate
  safety abort. It is a conservative envelope, not a servo-accuracy target.

Joint-6 absolute observed envelope:

- lower: 0.456783942430 rad (fresh start minus 1 degree);
- upper: 0.578956990069 rad (fresh start plus 6 degrees).

Every non-target joint command remains exactly invariant. Each non-target
observed joint must remain within ±0.00174532925 rad / ±0.1 degrees of its fresh
start; violation is independently fatal and is not attributed to joint-6 lag.

## Limits and physical effect

The fresh joint-6 lower safe-limit margin is 6.670156080 rad / 382.172 degrees.
The proposed target retains 5.634415147 rad / 322.828 degrees to the upper safe
limit, using the repository model's 5-degree conservative margin.

For an axial tool frame, joint-6 rotation should introduce approximately zero
tool-origin translation and +5 degrees orientation. A point 100 mm radially
from the axis may sweep up to approximately 8.724 mm. The RH56 assembly,
adapter, camera, connectors, and cables rotate through the full 5 degrees, so
direct operator clearance confirmation is mandatory.

## Validation and approval boundary

- Nineteen Gate 3C native/fake/trajectory/link-surface tests pass.
- The +5-degree read-only plan outcome is completed, with tool/user IDs 0/0 and
  logout code 0.
- EDG entries, servo-mode changes, and commands were all zero.
- Machine-readable plan:
  `docs/gate3c_measurements/jaka_gate3c_5deg_readonly_plan_20260716.json`.

Before physical execution, the operator must newly confirm E-stop access, no
person in the workspace, full RH56/camera/adapter/connector/cable clearance for
positive 5-degree rotation, expected direction, and readiness to interrupt.
The exact required approval phrase is:

`I_APPROVE_GATE3C_5_DEGREE_JOINT6_MOTION`

After approval, the command-capable worker must undergo fake lifecycle and
failure validation for the revised dynamic tracking/envelope policy before the
single physical run.

## Physical execution record

The approved disposable native process used binary SHA-256
`45a9d758b8749225b2744ddb5dd0bd48d7d1c011cbcad6a3463696ff6555f3de` and
captured a fresh state after its own countdown:

`[1.5697586776435382, -0.028311217840653623, -0.7708291984145285, 0.5052832047165559, 0.1346044239983394, 0.474237234949541]` rad.

The reconstructed outward target was:

`[1.5697586776435382, -0.028311217840653623, -0.7708291984145285, 0.5052832047165559, 0.1346044239983394, 0.5615036975492574]` rad.

All 1,439 planned commands completed. The lifecycle was
`login, preflight, precommand_check, enter_edg, enable_servo_move,
disable_servo_move, exit_edg, logout`; every reported lifecycle return code was
zero and no probe process remained afterward. Because the worker aborts on the
first nonzero cyclic read or command result, completing every planned command
also establishes that all 1,439 EDG reads and joint commands returned zero.

| Measurement | Result |
|---|---:|
| Observed positive joint-6 outward displacement | 0.087217723 rad / 4.997207 degrees |
| Commanded joint-6 displacement | 0.087266463 rad / 5.000000 degrees |
| Peak raw command/observation difference | 0.001951402 rad / 0.111807 degrees |
| Tracking-warning / dynamic-hard crossings | 0 / 0 |
| Maximum non-target observation delta | 0.000124151 rad / 0.007113 degrees |
| Final maximum return error | 0.000143248 rad / 0.008207 degrees |
| Period mean / median / p95 / p99 / max | 8.000021 / 7.999864 / 8.007175 / 8.034191 / 10.682119 ms |
| Wake mean / p99 / max | 0.064619 / 0.102286 / 2.740711 ms |
| EDG state-read mean / p99 / max | 0.023882 / 0.073402 / 0.360463 ms |
| Joint-command mean / p99 / max | 0.048746 / 0.134291 / 0.551333 ms |
| Start-period warnings / completion misses / hard misses | 3 / 0 / 0 |
| Process CPU / migrations | 51.60% / 2 |

The three timing warnings were isolated start-to-start periods above 8.8 ms;
the largest was 10.682119 ms, below the configured 12 ms hard boundary. There
were no consecutive warning periods, completion misses, or hard misses. The
raw same-cycle command/observation difference is not claimed as a
timestamp-aligned servo error.

Final EDG observation was:

`[1.5697491024666665, -0.0282743334, -0.77078974441, 0.5052728098333333, 0.13459978961777777, 0.4743804826]` rad.

Machine-readable artifacts:

- `docs/gate3c_measurements/jaka_gate3c_5deg_motion_20260716.json`;
- `docs/gate3c_measurements/jaka_gate3c_5deg_trajectory_20260716.csv`.

The operator observed joint 6 moving in the expected positive direction by
approximately 5 degrees. The RH56 assembly, wrist camera, adapter, connectors,
and cables remained clear and untensioned. No abnormal sound, vibration,
oscillation, collision indication, controller alarm, unexpected motion, or
contact was observed. The robot visibly returned to its starting pose.

Gate 3C is successfully completed. This validates the predefined joint-space
path only; it does not authorize Cartesian motion or live TeleDex input.
