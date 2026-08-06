# Runtime stop-condition audit

本页是当前运行时停止条件的审计结果。目标是减少只由数值边界、诊断或
数据质量造成的无必要中断；它不改变 JAKA、RH56 或控制器的安全边界。

This audit separates hardware safety from software quality and diagnostics. It
does not certify a physical run and does not authorize connecting hardware.

## Classification

| Area / condition | Class | Current action | Decision |
| --- | --- | --- | --- |
| JAKA controller alarm/error, collision, E-stop, power/enable loss | Robot safety boundary | terminal stop | unchanged |
| JAKA SDK/status/transport failure, device identity, tool/user-frame mismatch | Robot safety boundary | terminal stop or startup rejection | unchanged |
| Native command-stream timeout, worker death, watchdog and terminal timing fault | Robot safety boundary | terminal stop | unchanged |
| RH56 fatal serial/checksum/protocol/write/device fault, worker death, nonzero `ERROR`, or stale feedback beyond freshness | Robot safety boundary | terminal stop | unchanged |
| One RH56 read timeout with a fresh complete snapshot | Hand-local runtime quality | hold one command cycle and retry | changed: repeated stale feedback still faults |
| Joint position limits, candidate singularity/collision checks | Robot safety boundary / command legality | reject/hold, with escape candidates retained | unchanged |
| Periodic J1/J4/J6 candidate step above `0.22 rad` | Candidate/dynamic feasibility | output prefilter hold or native shaping | changed: this value is not periodic travel or winding; zero-offset wrist motion is not a branch hard stop |
| No legal equivalent branch, actual branch/winding fault | Robot safety boundary / command legality | terminal safety stop | unchanged; equivalent-branch selection, absolute limits, winding guard, native checks, and cleanup remain |
| Output velocity, acceleration and jerk final assertions | Robot safety boundary / command legality | terminal stop before SDK call | unchanged |
| Native controlled-stop timeout and cleanup failure | Robot safety/lifecycle boundary | bounded stop and fault report | unchanged |
| Native transition shaper approaching the jerk boundary | Software numerical boundary | shape the next output | changed: internal target now has 0.5% headroom; final assertion is unchanged |
| Native no-progress classification during output transition | Runtime quality boundary | degraded hold, then persistent hard fault after configured duration/cycles | unchanged; already requires persistence and was not the latest failure |
| Python 20 ms control budget | Runtime quality boundary | discard late candidate and publish HOLD_REJECTED heartbeat | unchanged; it is not a hard robot stop and the budget is not widened |
| Quest/controller temporary stale input or input recovery timeout | Runtime quality/liveness boundary | clutch hold, persistent disengaged hold, fresh heartbeat | changed: timeout no longer exits the process |
| Camera stale/missing frame | Runtime quality boundary | metadata-only/degraded recording; persistent acquisition fault aborts recording | unchanged; it does not stop a healthy robot session |
| Camera/control timestamp regression | Runtime quality boundary | invalidate/abort the episode | unchanged; protects dataset integrity, not hardware |
| One missing canonical required field | Runtime quality boundary | metadata-only invalid slot | changed: persistent consecutive loss aborts recording |
| Recorder queue full or writer latency | Runtime quality boundary | bounded drop/degraded quality or recording abort on writer failure | unchanged; no blocking of control and no robot emergency stop |
| Camera/recorder/preview child failure | Runtime quality/lifecycle boundary | mark recording degraded/failed and perform bounded cleanup | unchanged; preview is non-required |
| Diagnostic event enqueue/drop, percentile/report generation | Diagnostic/reporting boundary | record/report if possible | no robot stop |
| Operator stop and configured maximum session duration | Diagnostic/operator boundary | normal bounded shutdown | unchanged |

## Call-chain audit table

| ID / trigger | Source / consequence before | Protected object / recoverable | Evidence and final decision | Offline coverage |
| --- | --- | --- | --- | --- |
| A1 IK no solution, target jump, soft joint margin, candidate collision, candidate singularity | `SharedJakaTargetGenerator.evaluate` rejects; session publishes `HOLD_REJECTED` | Arm target; yes, operator can retreat | Candidate is never committed; retain hold | shared-pipeline and singularity tests |
| A2 continuation backtrack exhausted | `SmoothQuestJakaSession.control_tick` ended candidate path | Arm target; yes | bounded hold and fresh heartbeat | shared-pipeline tests |
| A3 Python output velocity/acceleration predictor rejection | `JointOutputFeasibilityTracker.preview` rejects | Arm target; yes | `commit` is not called; last accepted target remains | output-feasibility tests |
| A4 `CONTROL_COMPUTE_BUDGET_EXHAUSTED` | candidate result rejected | Producer freshness; yes | counter increments, no native deadline/watchdog widening, heartbeat continues | shared-pipeline budget test |
| A5 Quest tracking/controller transient loss | clutch fault and mapper clear | Arm reference; yes | no stale reference/extrapolation; release-before-press required | live-controller recovery tests |
| A6 Quest input recovery timeout | previously terminal process path | Arm command stream; yes | persistent disengaged hold; heartbeat remains live; actual producer death is separate | live-controller timeout test |
| A7 periodic candidate step above `0.22 rad` | former periodic branch guard rejected the candidate | Arm target; yes | output prefilter/native shaping rejects or holds; no branch hard stop and no target commit | branch-continuity and wrist-step tests |
| A8 no legal equivalent branch / actual branch or winding fault | branch/winding guard rejects the candidate | Command legality; no | terminal hard stop remains; no branch/winding guard removed | branch-continuity tests |
| A9 singularity warning / escape candidate | warning or candidate rejection | Arm target; yes | warning is diagnostic; hard singularity at current state is terminal; retreat is allowed | singularity-liveness tests |
| A10 native PWL transition / final velocity-acceleration-jerk boundary | shaper/final checker acted before SDK | Physical output; no | hard boundary and watchdog unchanged; only existing internal 0.995 jerk transition headroom retained | native/resampler tests |
| A11 RH56 one read timeout with fresh snapshot | `RH56PcDirectControl.poll_feedback_register` faulted immediately | Hand command; yes | one-cycle hand hold/retry; repeated stale feedback faults | RH56 worker diagnostics test |
| A12 RH56 nonzero `ERROR`, checksum/protocol, write, worker death | worker failure and combined wrapper terminal path | Hand actuator state; no | hard stop retained; no automatic fatal retry | RH56 control/worker tests |
| A13 RH56 rate limit / delta gate / contact clamp | command not written or clamped | Hand target; yes | counter/hand hold; no arm hard stop | RH56 control tests |
| A14 isolated camera stale/drop/ring expiry | collector emits invalid quality row | Dataset slot; yes | metadata-only slot; persistent acquisition ends recording only | dataset quality tests |
| A15 canonical required field unavailable | immediate recording abort | Dataset slot; yes | one slot invalid; consecutive loss aborts recording | canonical-field regression test |
| A16 recorder queue/writer failure | recorder/collector marks episode done/aborted | Dataset only; yes | wrapper continues control and performs bounded recorder cleanup | nonblocking pipeline tests |
| A17 preview/event-log/placement failure | diagnostic path could escape from setup/loop | Telemetry only; yes | warning/counter; no control termination | static/offline path review |
| A18 native controller alarm/collision/E-stop/power/enable/SDK fault | native fault/typed terminal outcome | Robot; no | hard stop retained, cleanup first, no threshold relaxation | native fake-worker tests |
| A19 producer/IPC/native watchdog/actual liveness loss | wrapper/native terminal stop | Robot command authority; no | hard stop retained; no fabricated heartbeat after process death | native liveness tests |

The code paths reviewed for this table are the native JAKA worker, the shared
Quest/JAKA session and feasibility pipeline, the hardware runtime, RH56
PC-direct worker/control, and the episode collector/process runtime. Parse-time
configuration validation is intentionally not relaxed: invalid configuration
must fail before a device can be commanded.

## Main-chain simplification audit

Before this change, the runtime path was:

```text
Quest packet receiver/thread
  -> packet/freshness boundary and bounded queue
  -> SmoothQuestJakaSession clutch/reference and mapping/filter (60 Hz)
  -> SharedJakaTargetGenerator continuation IK
  -> full Python native-like output preview for every candidate
  -> full CandidateMetrics + attempt dictionaries + timing asdict
  -> AcceptedArmTarget
  -> Python/native IPC
  -> native 8 ms PWL/latest-destination shaper and final hard checks
  -> JAKA ServoJ/EDG
```

The hand path remains separate:

```text
Quest hand packet -> normalized retarget -> RH56 rate/delta/contact gates
  -> PC-direct worker scheduling -> serial read/write and raw registers
  -> hand-local hold or typed fatal worker result
```

The data path remains separate:

```text
workspace/wrist camera processes -> versioned shared-memory rings
  -> canonical sampler -> recorder process/episode writer
  \-> optional latest-only preview
```

After this change, the arm hot path is:

```text
Quest validation -> clutch/reference -> mapping/filter
  -> one continuation IK solve plus at most five configured retries
  -> branch/hard-range/singularity/collision checks
  -> coarse six-finite velocity/acceleration prefilter (no jerk prediction)
  -> immutable AcceptedArmTarget
  -> Python/native IPC -> native 8 ms shaping/final hard checks -> JAKA
```

Ownership and blocking audit:

| Component | Owner / rate | In control tick? | Blocking, allocation, persistence | Failure propagation |
| --- | --- | --- | --- | --- |
| Quest packet parse/freshness | Python receiver + session boundary / packet rate, target 60 Hz | Boundary only | bounded queue; no disk/actuator I/O | malformed/stale input is disengaged hold; actual producer/IPC death is hard stop |
| clutch/reference/mapping | Python producer / 60 Hz | Yes | small state updates; no disk/locks | invalid reference or transient tracking is bounded hold and fresh recapture |
| IK/Jacobian/collision/singularity | Python producer / candidate rate | Yes | MuJoCo/IK work; no camera/recorder wait | candidate rejection is bounded hold; actual hard state/branch/winding fault remains hard stop |
| output prefilter | Python producer / candidate rate | Yes | six finite values plus scalar velocity/acceleration; no native segment copy, no jerk object | reject current candidate only |
| `AcceptedArmTarget` + IPC | Python producer/native boundary / accepted target rate | Yes | immutable target and transport write; no disk | transport/command state unknown is hard stop |
| native shaper/final checker | native worker / 8 ms | Yes, separate process | real shaping, final velocity/acceleration/jerk/watchdog; no Python object allocation | actual output/timing/liveness fault is hard stop |
| RH56 retarget/serial | Python hand worker / configured hand schedule | Separate hand path | serial can block only its worker; feedback is typed raw data | ordinary rate/contact is hand hold; nonzero ERROR/protocol/worker death is hard stop |
| camera rings/canonical sampler | camera processes + collector / camera and dataset rates | No | fixed slots and metadata references; no native heartbeat dependency | isolated stale/drop/expiry is recording degraded |
| recorder/writer/preview/event log | separate processes/threads / dataset, preview, sampled event rates | No | disk/GUI work is asynchronous; normal control keeps only current event record | writer/acquisition failure affects recording; preview/event/placement is diagnostic |

Python output feasibility is now intentionally a prefilter rather than a
second native resampler. The existing detailed `preview` API remains only for
offline feasibility analysis/regression evidence; `SharedJakaTargetGenerator`
uses `prefilter` in the control tick. Python retains the final cheap finite,
branch, hard-range, obvious velocity/acceleration checks. Native remains the
actual 8 ms velocity/acceleration/jerk shaper and final hard-check authority;
JAKA controller protection remains in addition to both software layers.

Normal-tick diagnostic reductions:

- no unconditional full `_base_record` pose/hand/contact allocation;
- no unconditional `control_stage_timing_ms` dictionary;
- no unconditional output-feasibility attempt dictionaries or timing `asdict`;
- no per-tick full event-history append; the wrapper reads one current minimal
  record, while a bounded detailed deque keeps rejects, faults, clutch events,
  and sampled records;
- continuation retry and IK-call counts are fixed-width integer timing rings;
- local control/temporary calibration SHA-256 values are not computed in the
  live collection path. Formal dataset artifact manifests and external
  protocol checksums remain separate; legacy validation may still read hashes
  from older episodes.

## Offline timing benchmark

Command used for both runs (2 seconds per fake scenario, no hardware):

```bash
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python \
  tools/validation/benchmark_control_timing.py --duration-sec 2
```

The benchmark scenarios are A Quest/control, B plus metadata mailbox, C plus
fake RH56 feedback, and D plus fake RH56 command/contact work. Values below are
milliseconds in `p50/p95/p99/max` order. The old IK column included Jacobian
work; the new split reports pure IK and Jacobian separately, so the comparable
old aggregate is shown as `IK+J`.

| Scenario | Stage | Before | After |
| --- | --- | ---: | ---: |
| A | IK | 4.573/4.712/5.072/5.382 | 4.219/4.397/4.555/4.757 |
| A | Jacobian | — | 0.171/0.223/0.315/0.486 |
| A | collision/singularity | 0.208/0.225/0.256/0.424 | 0.208/0.236/0.277/0.376 |
| A | Python output feasibility | 0.080/0.098/0.123/0.259 | 0.033/0.041/0.051/0.052 |
| A | event/diagnostic | 0.139/0.189/0.472/0.478 | 0.029/0.161/0.186/0.217 |
| A | total | 7.278/7.847/8.861/10.314 | 6.789/7.300/8.398/9.066 |
| B | IK | 4.525/4.607/5.094/6.044 | 4.354/4.509/4.744/5.713 |
| B | Jacobian | — | 0.193/0.399/0.836/0.985 |
| B | collision/singularity | 0.206/0.224/0.340/0.353 | 0.227/0.305/0.345/0.355 |
| B | Python output feasibility | 0.077/0.090/0.106/0.293 | 0.038/0.073/0.915/1.236 |
| B | event/diagnostic | 0.140/0.157/0.391/0.399 | 0.031/0.183/0.244/0.254 |
| B | total | 7.233/7.764/8.735/9.094 | 7.013/7.850/8.680/8.920 |
| C | IK | 4.547/4.612/4.924/4.989 | 4.340/4.902/5.133/5.181 |
| C | Jacobian | — | 0.187/0.271/0.482/0.535 |
| C | collision/singularity | 0.203/0.219/0.354/0.390 | 0.218/0.276/0.339/0.376 |
| C | Python output feasibility | 0.078/0.094/0.339/25.682 | 0.036/0.058/1.004/1.065 |
| C | event/diagnostic | 0.134/0.285/0.351/0.395 | 0.030/0.126/0.261/0.287 |
| C | total | 7.249/7.650/9.657/33.252 | 7.018/8.370/8.977/9.141 |
| D | IK | 4.556/4.619/4.673/5.041 | 4.325/4.514/4.744/4.991 |
| D | Jacobian | — | 0.189/0.258/0.312/0.482 |
| D | collision/singularity | 0.207/0.219/0.237/0.333 | 0.220/0.286/0.307/0.335 |
| D | Python output feasibility | 0.078/0.086/0.101/0.103 | 0.036/0.062/0.069/1.052 |
| D | event/diagnostic | 0.143/0.161/0.175/0.425 | 0.030/0.127/0.236/0.240 |
| D | total | 7.320/7.651/8.212/8.506 | 7.027/7.949/8.508/8.572 |

In the after run all four scenarios had `continuation_retries` p50/p95/p99/max
of `0/0/0/0` and `ik_calls` of `1/1/1/1`. Detailed event records retained
were 7 of 120 samples per scenario (bounded sampling), with 120 fixed-width
timing samples. RSS is a process-level `ru_maxrss` proxy, not an allocation
count; shutdown of the fake session was 0.219--0.291 ms. These are short
software measurements only, not JAKA/RH56/Quest/camera validation.

## The one numerical change

| Quantity | Before | After | Why safe | Failure still detected |
| --- | --- | --- | --- | --- |
| Internal native transition jerk target | `limit * (1 - 1e-9)` | `limit * (1 - 0.005)` | This is only a software shaping target. It is below the configured limit and reduces finite-difference oscillation around the final check. | The final output jerk assertion still compares against the same configured nominal limit plus the existing numeric comparison envelope; non-finite and materially excessive jerk still fault before the SDK call. |

The configured native output jerk limit, physical/controller limits, output
velocity and acceleration hard boundaries, and the existing final numeric
comparison envelope are unchanged. The new headroom is a compile-time policy
constant shared by the native worker and the existing resampler path; no new
CLI or environment variable is required.

The recent evidence was a pre-SDK `native_output_jerk_hard_fault` with a raw
finite-difference value around `62.831890 rad/s^3` against a nominal
`62.831853 rad/s^3` limit, while JAKA alarm, hard timing fault and tracking
faults were zero. The transition shaper had produced many points exactly at
the nominal boundary. This change addresses that numerical mechanism without
turning a final output violation into a warning.

## Why other software thresholds were not widened

- The 20 ms budget protects control freshness by rejecting a late candidate;
  it does not terminate the native worker. Widening it would hide the
  observed control deadline problem and would not fix the native output fault.
- Camera age, canonical matching, queue watermarks and writer failures already
  degrade or abort only recording. Relaxing them would risk stale or
  mislabeled training data while providing no robot-safety benefit.
- Native command-stream, producer, IPC, and watchdog timeouts are liveness
  boundaries and remain hard stops. A Quest input recovery deadline is
  different: the live Python session stays in persistent disengaged hold with
  fresh heartbeats, while Python/process death emits no heartbeat and remains
  hard. RH56 accepts one fresh-snapshot read timeout for a retry; repeated
  stale feedback remains terminal.
- No-progress and transition-hold escalation already require a persistent
  duration/cycle count. There is no evidence that this path caused the latest
  stop, so increasing it would be an unsupported change.

## Validation scope

The change is validated offline with the native/resampler regression tests and
static checks. No JAKA, RH56, Quest, camera, or other actuator was connected
for this audit. A physical run must separately verify that the shaped output
remains smooth and that all hardware safety faults continue to stop the run.
