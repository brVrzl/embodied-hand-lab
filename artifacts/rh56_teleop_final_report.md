# RH56DFX teleoperation isolation audit and low-risk optimization report

## Repository isolation and provenance

- Original repository: `/home/thor/projects/embodied_lab`
- Isolated full working-tree copy: `/home/thor/projects/embodied_lab_rh56_dev`
- Source-state evidence: `/tmp/embodied_lab_rh56_source_state_20260730_105432`
- Work backup: `/tmp/embodied_lab_rh56_teleop_20260730_105521`
- Source branch: `main`
- Source HEAD: `156d50dd014b548c292fc68a3cd840487455eb40`
- Baseline snapshot branch: `local/rh56-prework-baseline`
- Baseline snapshot commit: `45895c0a5e0ba2cf7b014be5a49263236baecb8a`
- Development branch: `dev/rh56-teleop-improvement`
- Development tip: `5a609bfa13a31f2688dc047d7d2e28d20c9c2e61`
- Integration branch: `integration/rh56-teleop-improvement`
- No-conflict merge commit: `808fee132fd82fe1c93925a27497f974d084bf36`
- Remote pushes: none

The source working tree began with 43 tracked modified files and 18 untracked
status entries (25 actual untracked files after directory expansion). The
complete tracked binary diff and expanded untracked list were copied and
compared byte-for-byte before any work. The dirty tree became the local
baseline snapshot; none of its content is claimed as this task's work.

At completion, source branch, HEAD, status text, tracked binary diff, and
expanded untracked list all compare byte-for-byte with the initial records.
The original directory was not modified. The copy's `.venv` retained an
editable path to the source repository, so the RH56 combined wrapper now
explicitly prepends its own copy-local `src/` to `PYTHONPATH`; the offline
combined-entry regression verifies this isolation.

## Branch commits

Development commits after the baseline snapshot:

```text
228656f feat(rh56): add worker diagnostics and durable failures
4f20ff2 test(rh56): cover timing mailbox and logging faults
5a609bf docs(rh56): record teleop audit and decision table
```

The integration merge is `808fee1 merge(local): integrate RH56 teleop
improvements`. This final report is a documentation-only integration follow-up
and does not change the development branch or the merge commit.

## Touched files and purpose

| File | Purpose |
|---|---|
| `artifacts/rh56_teleop_audit.md` | Baseline-only, pre-behavior-change call-chain and bottleneck audit. |
| `artifacts/rh56_teleop_decision_table.md` | Options A–F evidence/risk/experiment decision table. |
| `artifacts/rh56_teleop_final_report.md` | This provenance, implementation, and validation report. |
| `configs/hand/rh56_pc_direct_teleop.yaml` | Opt-in bounded diagnostics, default-on exact duplicate suppression, default-off stale drop, bounded telemetry policy; rates unchanged. |
| `configs/hand/rh56_pc_direct_ch341_teleop.yaml` | Same diagnostics/logging policy; rate unchanged. |
| `scripts/run_quest_jaka_rh56_teleop.sh` | Bind combined subprocess imports to the isolated copy's `src/`. |
| `src/rh56_driver/pc_direct_control.py` | Per-register/write timing, sequences, command age, exact duplicate semantics, structured control/serial failures. |
| `src/rh56_driver/pc_direct_worker.py` | Preserve latest-only mailbox; add observable coalescing, deterministic cycle API, bounded timing windows, stale-drop option, and persistent worker failure record. |
| `src/rh56_driver/serial_backend.py` | Typed timeout/checksum/protocol/ack failures with register/address context. |
| `src/rh56_driver/telemetry.py` | Bounded periodic JSONL recorder with immediate fault flush and independent logging failures. |
| `tools/quest_rh56_hand_test.py` | Use bounded recorder and expose input/retarget/worker/logging summaries in hand-only mode. |
| `tools/quest_jaka_hardware.py` | Minimal RH56-only recorder construction/cleanup/summary changes in the combined entry. |
| `tests/test_rh56_worker_diagnostics.py` | Deterministic fake-clock/fake-serial coverage for queue, timing, fault, duplicate, logging, shutdown, and serial ownership contracts. |

All pre-existing touched files were copied under the backup directory with
their repository paths before modification. New files had no preimage.

## Complete RH56 call chain

```text
Quest UDP HTS/CTRL packet
-> QuestDatagramReceiverWorker bounded receive queue
-> LiveQuestControllerRouter
-> SmoothQuestJakaSession 60 Hz control tick
-> 21-landmark right-hand ProjectRh56Retargeter
-> grip-clutched relative six-channel normalized target
-> RH56PcDirectWorker latest-only single-slot mailbox
-> RH56PcDirectControl range/rate/delta/duplicate gates and raw conversion
-> RH56SerialBackend canonical-to-protocol reorder and ANGLE_SET exchange
-> RH56DFX device execution (internal behavior unknown)
-> ANGLE_ACT/CURRENT/FORCE_ACT/STATUS/ERROR serial exchanges
-> PcDirectFeedback / rh56_pc_direct_episode.v1
-> bounded RH56 JSONL recorder
-> hand-only or combined supervisor summary
```

Canonical channel order is `index, middle, ring, pinky, thumb_close,
thumb_lateral`; protocol order is `pinky, ring, middle, index, thumb_close,
thumb_lateral`. Targets are normalized actuator-space closure `[0, 1]`, capped
to `0.8` by the production profile, not hand joint angles. Current calibration
maps open `0.0` to raw 1000 and close `1.0` to raw 0, then rounds to integer
register counts. A fresh grip engagement captures measured `ANGLE_ACT` and
Quest hand features. Grip release, invalid/stale skeleton, and shutdown hold
the last command and do not automatically open or write a stop register.

Active shaping before this task was: retarget clamp, retarget feature slew
`0.08` normalized/sample, per-feature dead zones, relative gains, maximum-close
clamp, controller `0.05` normalized/successful-write delta, and integer
quantization. There is no active PC-direct low-pass filter; the old YAML note
claiming `smoothing_alpha=0.35` had no consumer and was corrected.

## Seven distinct frequencies

| Frequency | Requested/current code limit | Runtime truth |
|---|---|---|
| Quest hand/controller packet receive | arrival-driven, no fixed host request rate | must be measured from accepted unique receive timestamps |
| Hand retarget compute | at most the shared 60 Hz tick while capture/update is valid | now summarized independently |
| RH56 target generation | at most 60 Hz on valid active UPDATE ticks | may repeat because a Quest sample can be reused or dead-zone output can hold |
| Target submission | one per generated physical-hand target, at most 60 Hz | worker reports submitted and exact-unique rates separately |
| Serial command write | at most 15 Hz; only after complete feedback | attempt and successful rates are distinct; duplicate suppression can reduce attempts |
| Complete feedback poll | one five-register poll per 15 Hz worker cycle | complete-record rate is measured independently from JSONL row count |
| Device execution/physical response | unknown | requires separately authorized instrumented hand-only testing |

Requested target rate is 60 Hz. Generated, unique, and submitted rates are
data/state dependent and no greater than that tick rate. Serial write attempts
and successes are no greater than the unchanged 15 Hz gate. Full feedback is
also explicitly scheduled at 15 Hz. Physical onset, internal servo frequency,
interpolation, queueing, and response bandwidth cannot be established offline.

The historical 903 feedback records in 60 seconds (`15.05 Hz`) match the
explicit 15 Hz worker setting. The direct limiter for approximately 15 Hz is
therefore configuration plus the shared worker period, not a proven natural
bus/device ceiling. Serial and runtime occupancy consume substantial margin but
do not prove the physical bottleneck.

## Serial transaction and bandwidth estimate

At 115200 baud and conventional 8N1 framing:

- ANGLE/CURRENT/FORCE read: 9-byte request + 20-byte response = about 2.517 ms
  ideal wire occupancy each;
- STATUS/ERROR read: 9 + 14 bytes = about 1.997 ms each;
- six-channel ANGLE_SET: 20 + 9 bytes = about 2.517 ms;
- complete five-register feedback: 133 bytes = about 11.55 ms ideal wire time;
- feedback plus command: 162 bytes = about 14.06 ms ideal wire time.

Each exchange also has an unconditional 5 ms host sleep. A feedback-plus-
command cycle therefore adds 30 ms of fixed sleeps and has a nominal minimum
near 44 ms before USB/device turnaround, Python polling/decoding, scheduling,
serialization, and file I/O. Configured serial timeout is 0.2 s, while the
effective exchange deadline is `max(4*timeout, 0.1)=0.8 s`; there is no retry.

The committed vendor example sleeps 10 ms per request but does not document a
normative minimum interval. Maximum accepted command rate, busy/rejection
behavior, firmware queueing/interpolation, duplicate-command effects, and
physical bandwidth remain **unknown**.

## Confirmed problems and implemented low-risk changes

Confirmed baseline problems:

- exact settled targets were rewritten every active 15 Hz cycle;
- submit sequence, written sequence, coalescing, command age, write attempt/
  success rate, per-register latency, and cycle jitter were not observable;
- generic worker failure lost exception type/message/traceback/register context;
- JSONL callback exceptions killed the serial worker and looked like worker or
  serial failure;
- ordinary RH56 telemetry synchronously flushed every row in hand-only and
  combined paths;
- combined RH56 records grew for the full episode;
- copied `.venv` editable imports could resolve to the original project in the
  combined wrapper;
- YAML documented a nonexistent PC-direct smoothing alpha.

Implemented:

1. Kept the existing latest-only single-slot mailbox; added submitted,
   observed, and written sequences plus coalesced-unobserved count.
2. Added optional bounded timing windows (default disabled): p50/p95/p99/max
   for submit/unique/write/feedback/cycle intervals, command age, write/full-
   feedback/per-register latency, plus cycle jitter and overruns.
3. Added exact duplicate suppression after the delta-limited state has fully
   reached the exact normalized requested tuple.
4. Added optional stale-command dropping, default disabled. Measured activation
   targets are exempt; stop/terminal behavior clears the mailbox as before.
5. Added typed serial failure context and `rh56_control_failure.v1`.
6. Added `rh56_worker_failure.v1` with UTC/monotonic time, exception type,
   message, traceback, thread, state, fault reason, control failure, sequences,
   counters, and bounded diagnostics. Fault records request immediate flush and
   enter hand-only/combined summaries.
7. Added bounded JSONL buffering: capacity 64, normal flush after 16 records or
   1 s, immediate fault flush, and best-effort shutdown flush. File write and
   flush failures remain `rh56_logging_failure.v1`/recorder summary data and do
   not become serial failures or kill the worker. The buffer drops oldest
   retained rows rather than growing without bound after persistent I/O error.
8. Removed the combined run-long RH56 record list; only counters and the last
   telemetry row remain in memory.
9. Bound the combined wrapper to copy-local Python sources.

Exact duplicate semantics deliberately use normalized tuple equality, not raw
count equality and not a nonzero deadband. A distinct target is still written
even if integer quantization produces the same raw counts. While a far target
is being advanced by the existing 0.05 delta limit, every needed step still
writes. The first command after every measured activation and any explicit
forced/safety command bypass suppression. Stop and terminal transitions still
clear the pending slot and produce no new ordinary position write.

Latest-only semantics remain: a producer overwrite replaces a pending target
that the worker has not observed; no old target is queued or replayed. Once the
worker observes the latest target, it may revisit it on later cycles only to
complete the existing delta-limited progression. Hold/terminal stop clears the
slot. Stale dropping is not active by default.

## Changes intentionally not implemented

No command frequency, worker frequency, feedback rate, safety polling rate,
serial concurrency, feedback schedule, interpolation, slew shaping, nonzero
actuator deadband, Quest stale threshold, current/force/status/error threshold,
automatic reconnect, physical range, or six-channel mapping was changed.
STATUS and ERROR continue to be read every complete feedback cycle. A failure
in one feedback register still aborts the remaining poll and faults the
controller; no safety feedback was deleted or relaxed.

Options A–F and candidate-only experimental ranges are detailed in
`artifacts/rh56_teleop_decision_table.md`. Raising the whole loop, layered
feedback, and host interpolation require true hand-only device evidence.

## Diagnostics and record fields

Supervisor/per-record fields include:

- Quest input, grip sample, and retarget counts/rates;
- submitted and exact-unique target count/rate;
- mailbox kind/capacity, pending/observed/written sequence, coalesced count;
- serial attempt/success count/rate and exact-duplicate suppression count;
- command disposition and submit-to-write age;
- complete feedback count/rate and latency;
- ANGLE/CURRENT/FORCE/STATUS/ERROR per-register latency;
- cycle interval/duration/jitter/overrun;
- stale-drop enabled/limit/count;
- logging capacity/depth/drop/flush/failure counts;
- structured control and worker failures.

Diagnostics-disabled telemetry retains targets, channel order, safety state,
sequences, disposition, command age, and failures, but omits bounded timing
windows/per-register details (`rh56_diagnostics` and per-row worker diagnostics
are `null`). Offline comparison proves diagnostics on/off emits identical
target values and order.

## Validation

Development branch:

```text
.venv/bin/python -m pytest -q tests/test_rh56_serial_backend.py \
  tests/test_rh56_pc_direct_control.py \
  tests/test_quest_rh56_hand_test_entry.py
32 passed in 0.65s

.venv/bin/python -m pytest -q tests/test_rh56_worker_diagnostics.py \
  tests/test_quest_jaka_bounded_teleop_entry.py
28 passed in 1.48s
```

Integration branch after merge:

```text
specified three-file RH56 suite: 32 passed in 0.66s
extended diagnostics + combined-entry suite: 28 passed in 1.22s
git diff --check: passed, no output
git diff --check local/rh56-prework-baseline..integration/rh56-teleop-improvement: passed, no output
PYTHONDONTWRITEBYTECODE=1 .venv/bin/python -m compileall -q src tools: passed, no output
bash -n scripts/run_quest_jaka_rh56_teleop.sh scripts/run_quest_rh56_hand_test.sh: passed, no output
```

The 14 new diagnostics tests cover latest-only coalescing, no old-target FIFO,
exact duplicate and distinct/forced target behavior, sequence/age, feedback-
before-command ordering, per-register timing, default-off and activation-exempt
stale dropping, diagnostics equivalence, structured timeout/protocol/write/
worker errors, logging isolation, immediate/batched/shutdown flush, bounded
failure buffering, single-owner non-overlapping serial access, and unchanged
15 Hz defaults. Existing hand-only fake construction/start remains covered.
The combined plant-free entry test validates both gates with zero network,
device open, or hardware command.

Not run: the repository-wide pytest suite, native JAKA build/tests, unrelated
vision/dataset/simulation suites, and external lint/type tools. They are outside
the RH56-only change surface; the repository config declares no separate lint
or type-check entry. No test was skipped, weakened, deleted, or changed to hide
a failure.

No RH56 hardware was opened or commanded. No JAKA process, SDK session, native
worker, login, enable, ServoJ, or EDG operation was started. No default command
or feedback rate was increased. No safety feedback requirement was reduced.

## Independent baseline diffs

Development diff before the final-report-only integration commit:

```text
12 files changed, 1816 insertions(+), 82 deletions(-)
```

Integration diff including this report:

```text
13 files changed, 2200 insertions(+), 82 deletions(-)
```

The development and integration code content are identical at merge commit
`808fee1`; only this final report is added after that merge.

## Shared-file conflict notes

No changes were made to `src/quest_jaka_sim/smooth_session.py`, hand retarget
logic, arm targets, arm heartbeat, arm clutch, compute budget, accepted-target
starvation logic, `native/jaka_servo_worker/`, `src/jaka_driver_adapter/`, or
`src/teleoperation/output_feasibility.py`.

Potential parallel-task merge overlap is limited to:

- `tools/quest_jaka_hardware.py`: import area near line 55;
  `_timestamp_rate_hz` near line 77; RH56-only recorder construction inside
  `main()` near lines 585–609; RH56 cleanup near 1039–1044; RH56 summary near
  1219–1249. Preserve concurrent arm changes in all surrounding regions and
  merge only these RH56 blocks.
- `scripts/run_quest_jaka_rh56_teleop.sh`: copy-local `PYTHONPATH` at line 8.

No arm-control function or parameter was modified in these regions.

## Recommended first physical experiment

In a new, separately authorized session, run hand-only only. Keep command and
all feedback rates at 15 Hz, keep STATUS/ERROR polling, keep stale drop off, and
enable diagnostics in a copied RH56 YAML. Use a short bounded script covering
hold/re-engage, slow and fast open/close, and independent thumb/finger motion.
Capture Quest receive, retarget, submit/unique/coalesced, serial attempt/
success, command age, per-register latency, ANGLE_ACT/CURRENT/FORCE/STATUS/
ERROR, plus synchronized external video. Measure physical onset and settling
against successful writes and ANGLE_ACT. Do not treat feedback JSONL lines as
command rate. Stop on any existing fault gate. Only then decide whether a
separate trial of options A, B, or E is justified.

## Integration into the actual project

Do not copy the baseline snapshot commit into another history as if it were RH56
work. Merge or cherry-pick only `228656f`, `4f20ff2`, and `5a609bf` (plus this
report if desired), or merge the local integration branch after reconciling the
parallel arm branch. Because `tools/quest_jaka_hardware.py` is shared, resolve
that file function-by-function and preserve all arm starvation fixes. Re-run
the exact 32-test RH56 suite, the 28-test diagnostics/combined suite, compileall,
shell syntax, and diff-check in the destination. Do not push this local branch
without a separate user decision.

## Requested command outputs

`git log --oneline --decorate --graph --all -n 30` at the integration merge:

```text
*   808fee1 (integration/rh56-teleop-improvement) merge(local): integrate RH56 teleop improvements
|\
| * 5a609bf (dev/rh56-teleop-improvement) docs(rh56): record teleop audit and decision table
| * 4f20ff2 test(rh56): cover timing mailbox and logging faults
| * 228656f feat(rh56): add worker diagnostics and durable failures
|/
* 45895c0 (local/rh56-prework-baseline) chore(local): snapshot pre-RH56 working tree
* 156d50d (origin/main, origin/HEAD, main) chore: prune obsolete tests and research scaffolding
```

`git status --short --branch` after merge and validation:

```text
## integration/rh56-teleop-improvement
```

Both requested stat commands at merge produced the same code/document delta:

```text
artifacts/rh56_teleop_audit.md                | 259 ++++++++++++++
artifacts/rh56_teleop_decision_table.md       |  27 ++
configs/hand/rh56_pc_direct_ch341_teleop.yaml |   9 +
configs/hand/rh56_pc_direct_teleop.yaml       |  15 +-
scripts/run_quest_jaka_rh56_teleop.sh         |   1 +
src/rh56_driver/pc_direct_control.py          | 289 +++++++++++++++-
src/rh56_driver/pc_direct_worker.py           | 424 +++++++++++++++++++++--
src/rh56_driver/serial_backend.py             | 142 +++++++-
src/rh56_driver/telemetry.py                  | 144 ++++++++
tests/test_rh56_worker_diagnostics.py         | 464 ++++++++++++++++++++++++++
tools/quest_jaka_hardware.py                  |  66 +++-
tools/quest_rh56_hand_test.py                 |  58 +++-
12 files changed, 1816 insertions(+), 82 deletions(-)
```

`git diff --check local/rh56-prework-baseline..integration/rh56-teleop-improvement`
completed successfully with no output.
