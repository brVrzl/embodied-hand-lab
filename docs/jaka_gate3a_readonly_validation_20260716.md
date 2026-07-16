# JAKA Gate 3A — connected read-only validation

Date: 2026-07-16  
Scope: physical state acquisition and SDK timing only  
Motion performed: none

## Outcome

Gate 3A completed against the physical controller at `192.168.71.50` using the
installed native JAKA SDK. No EDG, servo enable/disable, power, movement, frame
write, program-control, collision-setting, or IO-write API was called.

The 30-second conservative run completed 300/300 fast read cycles with no SDK
failure or timeout and achieved 9.999965 Hz. A 25 Hz trial also completed all
250 reads, but slow status calls caused seven poll deadline misses. Stable
125 Hz operation was not demonstrated and is not claimed.

Raw values used for the condensed tables are preserved in
`jaka_gate3a_physical_results_20260716.json`.

## Read-only enforcement

The diagnostic is a separate executable in `native/jaka_readonly_diagnostic`.
Its public backend interface contains connection lifecycle and read methods
only. The broad vendor `JAKAZuRobot` object is private to
`readonly_backend.cpp`; the supervisor cannot access it.

An automated allowlist test enumerates every `client_.method(...)` call in that
translation unit. The only reachable vendor calls are:

- `login_in`, `login_out`;
- `get_sdk_version`;
- `get_actual_joint_position`, `get_actual_tcp_position`;
- `get_robot_status_simple`, `get_robot_state`, `get_robot_status`;
- `is_in_servomove`, `is_in_estop`, `is_in_collision`;
- `get_tool_id`, `get_tool_data`;
- `get_user_frame_id`, `get_user_frame_data`;
- `get_program_state`, `get_program_info`.

The test rejects EDG, joint/Cartesian movement, servo enable/disable, robot
enable/power, program mutation, IO write, frame mutation, and collision-setting
methods. This is enforced in code and regression tests, not only documented.

## Operator and configuration behavior

- Default mode is `dry-run`; no backend or SDK client is constructed.
- An explicit IPv4 address is mandatory. There is no compiled site address.
- Physical mode requires the exact acknowledgement
  `I_ACKNOWLEDGE_JAKA_READ_ONLY_CONNECTION`.
- The installed `login_in(const char*, bool)` API has no credential parameter;
  credentials are recorded as unsupported rather than invented or hard-coded.
- Before connection, the tool prints the target, read-only scope, polling rates,
  absence of EDG/write APIs, and that no motion is intended.
- SIGINT/SIGTERM/SIGHUP set a stop flag; normal cleanup then calls `login_out`.

Example dry-run:

```bash
cmake -S native/jaka_readonly_diagnostic -B build/jaka_readonly_diagnostic -DCMAKE_BUILD_TYPE=Release
cmake --build build/jaka_readonly_diagnostic -j2
build/jaka_readonly_diagnostic/jaka_readonly_diagnostic \
  --mode dry-run --robot-ip <CONTROLLER_IPV4>
```

Physical command used for the 30-second result:

```bash
build/jaka_readonly_diagnostic/jaka_readonly_diagnostic \
  --mode connected \
  --robot-ip 192.168.71.50 \
  --acknowledgement I_ACKNOWLEDGE_JAKA_READ_ONLY_CONNECTION \
  --duration-s 30 --poll-hz 10 --slow-poll-hz 1 \
  --max-samples 1000 --metrics-file /tmp/jaka_gate3a_30s_20260716.json
```

## State inventory

| Field | Availability | Source and observation |
|---|---|---|
| SDK version | Direct | `libadd jakaAPI_version: V2.2.7stable_linux` |
| Configured robot model | Configuration only | Repository/site configuration says JAKA mini2; SDK did not report it |
| Controller firmware/version | Unavailable | No getter in installed C++ header |
| Robot operating mode | Unavailable | Program and servo states exist, but neither is relabeled as operating mode |
| Fault/alarm | Direct | `get_robot_status_simple`: code 0, empty message |
| Powered / enabled | Direct | Both true during measurements; no change was requested |
| Emergency stop | Direct | False from `get_robot_state`/`is_in_estop` |
| Collision/protective state | Direct | False from `is_in_collision` |
| Servo-move state | Direct | False from `is_in_servomove` |
| Joint position | Direct | Six radians from `get_actual_joint_position` |
| Joint velocity | Direct but qualified | Six zeros from documented `instVel` fields in deprecated/config-dependent combined status |
| TCP pose | Direct | Translation in SDK millimetres and RPY in radians |
| Active tool | Direct | ID 0; frame values all zero |
| Active user frame | Direct | ID 0; frame values all zero |
| Program state | Direct | Value 0, corresponding to header enum `PROGRAM_IDLE` |
| Program motion line | Direct | 0 |
| SDK socket status | Direct | Connected during sampling |
| Controller timestamp | Unavailable in scope | Only timestamp getter is EDG-specific and was not called |

Representative joint and TCP values are in the JSON summary. They are state
observations, not commanded targets.

## Polling architecture observed

There is no supported combined high-rate read suitable for the future cyclic
thread. `get_robot_status` is combined but is deprecated, depends on SDK
configuration, and blocked for 20–66 ms in these measurements. It is useful at
a low status rate only.

The separate actual-joint and actual-TCP getters are materially faster. Gate 3A
therefore used:

- fast path at 10 Hz, later 25 Hz: actual joint plus actual TCP;
- slow path at 1 Hz: simple/legacy status, robot state, servo, E-stop,
  collision, program state, and combined status;
- static after login: tool/user IDs and definitions, program info, SDK version.

Putting fast and slow calls in one diagnostic thread was intentional for
measurement. The 25 Hz result proves that this composition is unsuitable for a
deterministic servo thread: a slow call delays the next release and then causes
catch-up behavior. Future runtime composition must isolate slow telemetry from
the EDG cycle.

All calls were deliberately issued sequentially. Whether the vendor SDK
internally serializes concurrent callers is undocumented and was not tested;
the architecture therefore continues to prohibit concurrent access to one SDK
client.

## Physical timing results

### 10 Hz, 30 seconds

| Metric | Result |
|---|---:|
| Cycles | 300 |
| Failed calls / reads / timeouts | 0 / 0 / 0 |
| Achieved polling rate | 9.999965 Hz |
| Period mean / median | 100.000284 / 99.999350 ms |
| Period standard deviation | 0.164712 ms |
| Period min / max | 99.680772 / 100.305770 ms |
| Period p95 / p99 | 100.282784 / 100.297290 ms |
| Process CPU | 57.68% |

P99.9 is unavailable because the run contained fewer than 1000 periods.

| SDK call | samples | mean | median | maximum | p95 | p99 |
|---|---:|---:|---:|---:|---:|---:|
| Actual joint | 300 | 1.899 ms | 1.892 ms | 6.032 ms | 2.201 ms | 3.234 ms |
| Actual TCP | 300 | 1.197 ms | 1.213 ms | 3.878 ms | 1.420 ms | 1.523 ms |
| Simple status | 30 | 1.113 ms | 1.056 ms | 3.162 ms | 1.215 ms | 2.598 ms |
| Robot state | 30 | 5.418 ms | 5.110 ms | 20.474 ms | 6.231 ms | 16.473 ms |
| Combined status | 30 | 30.823 ms | 25.814 ms | 58.066 ms | 51.634 ms | 57.440 ms |

### 25 Hz, 10 seconds

| Metric | Result |
|---|---:|
| Cycles | 250 |
| Failed calls / reads / timeouts | 0 / 0 / 0 |
| Achieved polling rate | 24.999852 Hz |
| Missed poll deadlines | 7 |
| Maximum consecutive misses | 2 |
| Period minimum / maximum | 2.137 / 84.788 ms |
| Period p95 / p99 | 40.031 / 68.024 ms |
| Combined status maximum | 66.414 ms |
| Process CPU | 60.31% |

Mean achieved rate alone is misleading: absolute scheduling jitter p99 was
31.439 ms because slow calls created long periods followed by catch-up cycles.
No 50 or 125 Hz trial was attempted after this result.

## Connection lifecycle and failures

- SDK/client construction: about 4–12 microseconds in measured processes.
- SDK exposes connection and authentication only as one `login_in` call, so
  connection and login cannot be separated honestly.
- Three successful physical logins averaged 166.048 ms.
- First successful fast state read across those sessions averaged 2.034 ms.
- Three successful logouts averaged 50.466 ms.
- Three physical sessions completed with two reconnect attempts and no failure.
- A Ctrl-C physical run called logout once, wrote metrics, and exited 130 with
  outcome `operator_interrupted_cleanly`.
- Unreachable TEST-NET address `192.0.2.1` returned SDK code `-1` after 3.057 s;
  it was classified as one connection failure and zero failed reads.
- Active physical network interruption was not induced because changing the
  shared robot-network route was not justified. Fake-backend interruption and
  timeout paths are covered automatically.

The SDK created a thread named `jakarobot_heart` that used about 54.6% CPU in a
live snapshot. Total process CPU was 55–60%. `login_out` and client destruction
did not end that thread before process exit: baseline thread count was one,
one constructed client left two threads, and three recreated clients left four.
The operating system removed all threads when the dedicated diagnostic process
exited; no diagnostic process remained. This is a mismatch with the earlier
assumption that SDK cleanup also cleans all in-process background threads.

Implications:

- use one vendor client per dedicated worker process;
- do not repeatedly construct clients in a long-lived supervisor;
- treat process exit as the reliable boundary for SDK background-thread cleanup;
- include heartbeat CPU contention in EDG timing validation;
- never place combined/general status calls in the 8 ms EDG thread.

## Automated validation

`tests/test_jaka_readonly_diagnostic.py` covers dry-run, configuration rejection,
exact physical acknowledgement, source-level vendor allowlisting, forbidden
write APIs, fake unreachable/timeout/disconnect, repeated lifecycle, failed
logout, bounded samples, deadline accounting, and signal cleanup. Fake results
validate lifecycle behavior only and are not physical validation.

The diagnostic suite passed 14 tests; the combined Gates 1–3A focused suite
passed 54 tests in 2.24 seconds.

## Gate recommendation

The evidence is sufficient to consider a **separately approved, narrowly
bounded zero-motion EDG timing gate**, but not continuous motion:

1. one SDK client in one disposable native process;
2. no `get_robot_status` or other slow getter in the 8 ms thread;
3. fixed current-joint invariant target with the existing frame/state checks;
4. initial five-second run, immediate exit on the first SDK failure or repeated
   deadline miss, and explicit E-stop/operator acknowledgement;
5. report EDG state-read and command-write timing separately;
6. verify process termination after every trial because `login_out` does not
   stop the SDK heartbeat thread in-process.

Approval is not recommended for TeleDex following, Cartesian motion, minimal
motion, 125 Hz claims, or hand operation.
