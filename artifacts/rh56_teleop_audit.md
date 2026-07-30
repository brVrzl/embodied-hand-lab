# RH56DFX teleoperation read-only audit

Audit basis: local baseline snapshot `45895c0a5e0ba2cf7b014be5a49263236baecb8a`.
This document was completed before changing RH56 control behavior. Evidence is
limited to repository code, committed vendor examples, configuration, and
offline tests. Device-internal behavior and physical bandwidth are explicitly
unknown unless stated otherwise.

## Scope and call chain

The hand-only path is:

```text
Quest UDP HTS/CTRL packet
-> QuestDatagramReceiverWorker bounded receive queue
-> LiveQuestControllerRouter
-> SmoothQuestJakaSession ingest/poll at a configured 60 Hz target tick
-> ProjectRh56Retargeter over the right-hand 21-landmark skeleton
-> relative grip-clutched six-channel normalized target
-> RH56PcDirectWorker single-slot pending target
-> RH56PcDirectControl clamp/per-command delta limit/rate gate/raw conversion
-> RH56SerialBackend ANGLE_SET request/response
-> RH56 device (internal execution unknown)
-> serial ANGLE_ACT/CURRENT/FORCE_ACT/STATUS/ERROR requests
-> PcDirectFeedback and `rh56_pc_direct_episode.v1`
-> hand-only JSONL/summary
```

The combined path uses the same receiver, router, session, hand worker,
controller, and serial backend. It additionally copies the latest RH56 episode
record into the combined event and summary. The RH56 worker owns the only RH56
serial backend; the arm producer never accesses that serial port.

## Six-channel target semantics

Canonical order is `index, middle, ring, pinky, thumb_close, thumb_lateral`.
The protocol order is `pinky, ring, middle, index, thumb_close,
thumb_lateral`. A command presented to the worker is actuator-space normalized
closure, not a joint angle: `0.0` means the calibrated open endpoint and `1.0`
means the calibrated closed endpoint. The production PC-direct profile limits
closure to `0.8`; each channel currently maps linearly from normalized closure
to an integer protocol register count, with `raw_open=1000`, `raw_close=0`,
`safe_min=0`, and `safe_max=1000`. Increasing normalized closure therefore
decreases the transmitted count. The serial command writes all six 16-bit
`ANGLE_SET` register values in protocol order.

The Quest source is neither a controller trigger mapped directly to an
actuator nor an actuator command. A valid right-hand HTS sample contains a
21-landmark skeleton. Retargeting derives per-finger curl, thumb bend/pinch,
and thumb lateral opposition features. Grip is an independent clutch: on a
fresh press the session captures current Quest features and fresh measured
`ANGLE_ACT`, then applies subsequent feature changes relative to that measured
six-channel reference. Four fingers, thumb close, and thumb lateral have
independent gains and dead zones. Targets are clamped to the configured
normalized maximum.

The active transformations are:

- retarget normalized clamp `[0, 1]`;
- retargeter per-sample `maximum_normalized_step=0.08` slew limit;
- relative feature dead zones: four fingers `0.015 rad`, thumb close `0.008
  rad`, thumb lateral normalized feature `0.015`;
- per-channel relative gains of `1.0` and normalized output clamp `[0, 0.8]`;
- PC-direct per-successful-command delta limit `0.05` normalized;
- float-to-integer rounding and 16-bit protocol encoding;
- canonical-to-protocol reorder and decreasing-count close direction.

There is no active low-pass filter or host interpolation in the physical hand
output. The `smoothing_alpha=0.35` text in a YAML note has no consumer and is
not behavior. Simulation-only per-target radian step limits are bypassed for
the normalized physical output. Exact integer quantization happens after the
PC-direct delta limit.

Invalid/stale skeleton tracking produces no new target and holds the last
command. Grip release/staleness also clears the pending slot and holds the last
command. It does not automatically open. A safe-open normalized target exists
as all zeros / raw 1000, and the retarget calibration parses a `safe_open`
loss-behavior value, but that value has no consumer in this call path. The
PC-direct clutch path deliberately does not emit an automatic open on tracking
loss, stop, or shutdown. Cleanup closes the port without a register write.

## Frequencies and sequence meanings

These rates are distinct and must not be inferred from one another:

| Quantity | Current code evidence |
|---|---|
| Quest packet receive | Arrival-driven; measured by unique accepted HTS receive timestamps. No fixed sender rate is guaranteed. |
| Hand retarget compute | At most the 60 Hz shared target tick while the hand clutch is capturing/updating and skeleton data is valid. Existing session report derives it from `hand_timestamps_ns`. |
| RH56 target generation | At most the 60 Hz shared target tick; only active UPDATE ticks generate targets. Generated values can repeat because the latest Quest sample can be reused and dead zones can zero changes. |
| Target submit | One submit for each valid active `_update_hand`; presently up to 60 Hz. Baseline worker stores only the newest target and has no observable submit sequence. |
| Serial command write | Rate-gated to at most `control_frequency_hz=15`; command is attempted after a complete feedback poll. The same pending target is retried each worker cycle, so exact repeats are written in baseline. |
| Complete feedback poll | One five-register poll at the beginning of each worker loop. The loop period uses the same 15 Hz setting, so successful complete feedback records are explicitly capped near 15 Hz. |
| Device execution / physical response | Unknown. No repository evidence establishes internal servo rate, interpolation, queueing, or mechanical bandwidth. |

Requested/generated/unique/submitted targets, write attempts, successful
writes, complete feedback records, and physical responses are not equivalent.
Baseline telemetry exposes only final command/feedback timestamps and backend
aggregate error/write counts, so it cannot reconstruct the missing stages or
command age. A feedback JSONL row is one full feedback cycle, not proof of a
command write.

The observed 903 feedback records in 60 seconds (`15.05 Hz`) agree with the
explicit 15 Hz loop configuration. It is not evidence of a natural device
throughput ceiling. The baseline uses the same configuration in hand-only and
combined paths, while the shared Quest target tick is 60 Hz in both.

## Producer and worker semantics

`RH56PcDirectWorker._pending_target` is a shared latest-only single-slot
mailbox protected by one lock. It is not an unbounded or bounded FIFO. When the
60 Hz producer outruns the worker, a new target overwrites the earlier pending
value; old intermediate targets neither queue nor later replay. `hold()` and
`arm_terminal_stop()` clear the slot. Stop/fault therefore cannot drain stale
motion targets after the requested transition.

The baseline has no target sequence, written sequence, submit timestamp, or
command-age measurement. `submit_target()` explicitly discards its timestamp.
The worker snapshots the pending value but does not consume/clear it, so every
active worker cycle passes the same latest value to `command()`. Once the 15 Hz
rate gate opens, identical targets are encoded and written again. This wastes a
transaction but does not build a backlog. The first target on every grip
engagement is captured from fresh measured `ANGLE_ACT`, preventing an earlier
clutch target from becoming the first resumed command.

## Worker cycle and serial transactions

The worker is one thread with a scheduled period of
`1/control_frequency_hz` (66.667 ms at 15 Hz). Each cycle performs, in order:

1. complete feedback poll;
2. terminal/hold/activation handling;
3. at most one position command write;
4. episode record callback;
5. interruptible wait until the next period.

Feedback therefore has ordering priority and a command must wait for all five
feedback request/response transactions. A failure in any register aborts the
remaining poll and faults the controller; the code does not continue to later
registers. Command and feedback are never concurrent on the port. Cycle
overruns skip sleep and rebase the next deadline, but do not run concurrent
I/O.

At 115200 baud with conventional 8N1 framing (10 wire bits/byte), request and
response sizes derived from the local protocol implementation are:

| Transaction | Request | Response | Ideal wire occupancy |
|---|---:|---:|---:|
| `ANGLE_ACT`, `CURRENT`, or `FORCE_ACT` read | 9 B | 20 B | 2.517 ms each |
| `STATUS` or `ERROR` read | 9 B | 14 B | 1.997 ms each |
| six-channel `ANGLE_SET` write | 20 B | 9 B | 2.517 ms |

A complete feedback poll is 133 bytes / about 11.55 ms of ideal wire time. A
feedback-plus-command cycle is 162 bytes / about 14.06 ms. Every exchange also
has an unconditional 5 ms host sleep, giving at least 25 ms added to a five-read
poll and another 5 ms for a command, before USB/adapter/device turnaround,
Python polling, decode, timeout, serialization, file I/O, and scheduling. The
configured serial timeout is 0.2 s, while `_exchange` uses a deadline of
`max(4*timeout, 0.1)=0.8 s`; there is no retry. Thus the nominal best-case
feedback-plus-command path is roughly 44 ms plus runtime overhead, below but
material relative to the configured 66.7 ms period. A single missing response
can block about 0.8 s and then faults.

Frame construction/checksum/reorder and JSON encoding are small but currently
unmeasured. More importantly, both hand-only and combined RH56 telemetry call
`flush()` synchronously for every worker record. The combined callback also
keeps all RH56 records in a run-long list. Any write/flush exception escapes
the callback into the worker's broad exception handler, is mislabeled as
`pc_direct_worker_failure`, and kills hand control. This is a confirmed
control-path fault-classification and latency risk.

## Feedback roles and failure behavior

| Register | Baseline role | Missing/timeout/stale behavior | Rate change this task |
|---|---|---|---|
| `ANGLE_ACT` | measured startup/re-engagement reference, normalized observation, feedback freshness | any read/decode/checksum error faults the whole poll; stale complete feedback faults before command | unchanged |
| `CURRENT` | raw observation/logging; no per-cycle closed-loop command computation in this path | same whole-poll fault | unchanged |
| `FORCE_ACT` | raw observation/logging; no per-cycle closed-loop command computation or threshold in this path | same whole-poll fault | unchanged |
| `STATUS` | raw safety/diagnostic evidence; repository lacks verified nonzero-code meanings | missing/invalid response faults; values are recorded raw | unchanged |
| `ERROR` | active safety gate and logging | any nonzero channel faults immediately; missing/invalid response faults | unchanged |

All five registers are read as separate transactions and must complete before a
new `PcDirectFeedback` exists. There is no partial record or per-register stale
state. Safety polling is not reduced or rescheduled in this task.

## Configuration propagation audit

- `control_frequency_hz` is YAML seconds-inverse and reaches
  `RH56PcDirectControl.command_period_ns`; it also implicitly becomes the
  worker/full-feedback loop period.
- serial `baudrate`, `timeout_sec`, `hand_id`, and port reach
  `RH56SerialBackend`; CLI overrides only the port. `_exchange` multiplies the
  configured timeout by four, an important effective-unit mismatch.
- `feedback_stale_timeout_sec` reaches the complete-feedback freshness gate. It
  is absent from the CH341-named profile, so that profile defaults to twice the
  serial timeout (also 0.4 s).
- `hand_delta_limit`, calibration endpoints/ranges/directions, and safety
  `max_close_strength` reach the controller.
- Quest `input.stale_after_ms=250`, clutch `stale_after_ms=150`, and target
  `target_generation_hz=60` reach the shared session/router in both paths.
- hand relative gains/dead zones and retarget `maximum_normalized_step` reach
  the retarget/session consumers.
- hand-only `--feedback-period-sec` applies only to read-only/bounded entry
  modes, not to the Quest worker path.
- there is no baseline command-age limit, mailbox capacity option, duplicate
  suppression option, diagnostics toggle, logging interval, or logging batch
  configuration.
- the YAML note `smoothing_alpha=0.35` is documentation-only and misleading.

No CLI or shell default raises the 15 Hz command/full-feedback setting. The
approximately 15 Hz feedback result is explicitly configured, not proven to be
a bus saturation ceiling.

## Repository-supported vendor and physical limits

The committed vendor Python example uses 115200 baud and sleeps 10 ms after a
read or write request. It is example behavior, not a documented normative
minimum command interval. The production backend uses 5 ms. The repository
does not establish a supported maximum command rate, device-busy response,
command rejection at high rate, an internal command queue, device
interpolation, whether duplicate commands reset device motion planning, or
physical response bandwidth. All are **unknown** and require a separately
authorized hand-only experiment with write acknowledgements, register timing,
measured `ANGLE_ACT`, error/status, and physical response observations.

## Confirmed findings versus unconfirmed hypotheses

Confirmed:

- producer target generation can be 60 Hz while serial command/full-feedback
  cycles are explicitly limited to 15 Hz;
- the mailbox already coalesces latest-only and does not queue old targets;
- exact duplicate commands are rewritten;
- submit/write sequence and age are not observable;
- feedback and command are serial, with five reads before at most one write;
- per-record synchronous JSONL flush occurs in the worker thread;
- logging exceptions can terminate the worker and be confused with serial
  failure;
- worker exceptions retain only an in-memory exception and generic fault
  reason; supervisor summaries lose type/message/traceback/context;
- `smoothing_alpha` in the PC-direct YAML note is not active.

Unconfirmed:

- that 15 Hz is too low for the device or for acceptable physical response;
- that increasing worker/command rate would improve successful write or
  physical response rate;
- that feedback polling is the dominant physical-latency source rather than
  Quest sampling, retarget dead zones/slew, device firmware, speed setting, or
  mechanics;
- any safe reduced rate for CURRENT/FORCE/STATUS/ERROR;
- any device benefit or harm from duplicate commands;
- device-internal interpolation, queueing, rejection, or busy behavior.

The low-risk implementation phase may add opt-in bounded diagnostics,
structured failures, exact duplicate suppression, and bounded/periodic normal
telemetry flushing. It must preserve the latest-only mailbox, one-port/one-
thread ownership, feedback-before-command order, all safety reads, all default
rates, clutch/stop semantics, actuator mapping, ranges, and physical limits.
