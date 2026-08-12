# Force bandwidth and ACT shadow validation summary

Date: 2026-08-11 (Asia/Shanghai)

## Final verdicts

- **FORCE BANDWIDTH: `INSUFFICIENT_EVIDENCE`**
- **ACT SHADOW: `PASS`**
- **FIRST PHYSICAL ACT TIMING:** `n_action_steps=2`, 15 Hz policy queries, 30 Hz accepted-action cadence; measured p99 latency margin 38.724 ms and worst-observed margin 35.050 ms against the 66.667 ms query period
- **FORCEGUARD:** feasibility not yet claimed; effective native unique force rate is unknown

No robot or hand command was sent. ACT physical control was not enabled. No raw episode, production control configuration, safety limit, watchdog, clutch behavior, or recorder profile was changed.

## What is established

### RH56

The production `FORCE_ACT` rate of 10 Hz is a host scheduler setting. The official protocol permits reading all six channels from address 1582 in one request and documents 115200-baud 8N1 transport, but does not state internal refresh rate, sensor bandwidth, minimum poll period, or latency guarantee.

A stationary, read-only 10/15/20/30 Hz sweep sustained every requested force rate with no timeout, CRC, protocol error, or loss of the 15 Hz `ANGLE_ACT` stream. Force RTT p99 remained 6.82–7.16 ms and estimated wire utilization reached 40.09% at the 30 Hz sweep point. Thus 10 Hz is not the serial transport ceiling. Because a stationary input was almost constant, the unique-value update rate and physical information bandwidth remain unknown; production stays at 10 Hz.

### ACT live shadow

The validated LeRobot 0.6.2 ACT checkpoint was queried 300 times at 30 Hz from the two live RealSense cameras, JAKA measured joints, and RH56 measured positions. The model produced finite `[16,12]` native/absolute action chunks on every query. Total observation-to-output latency was 21.751 ms p50, 27.942 ms p99, and 31.617 ms maximum. A fixed observation produced bit-identical repeated output.

There were no authoritative hardware-limit excursions. JAKA channel 4 was outside the five-demonstration envelope on all shadow predictions by approximately 0.018–0.024 rad; all other dimensions remained inside. Predictions were logged without clamping and never executed.

## Timing recommendation basis

The first-rollout recommendation of two consumed actions per query is an **exploratory ablation value** constrained by measured Thor latency. It yields a 15 Hz query rate and 66.667 ms open-loop prefix. A 15 Hz ACT precedent is **directly supported by the phase-conditioned ACT paper**, while RDP and M²-ResiPolicy directly support the broader principle of slower high-level inference combined with faster physical feedback. These papers do not directly determine the JAKA/RH56 value.

The one-action/30 Hz schedule is supported at measured p99 but has only 1.717 ms worst-observed margin, so it is reserved as a later responsiveness ablation. The 4/8/16-action schedules meet compute deadlines but impose progressively longer open-loop periods without demonstrated task benefit.

## ForceGuard experiment bounds

The plausible native force rate cannot yet be stated numerically. With `R` defined as a future measured unique-update rate:

- history candidates: 3/4/5/8 unique updates (**exploratory ablation values**), reported in milliseconds as `count/R`;
- provisional polling-constrained history: 300–500 ms, 3–5 scheduled opportunities at current 10 Hz (**exploratory**, not guaranteed unique);
- failure-horizon candidates: 2/3/5 unique updates, provisionally 200–500 ms at the current schedule (**exploratory**);
- degradation rates: `R, R/2, R/4, R/8` (**exploratory ablation values**);
- degradation latency: `0, T, 2T, 4T`, `T=1/R` (**exploratory values derived from the measured native period once known**).

No numeric detector threshold is proposed.

## Reproducibility artifacts

- Literature audit: `research_log/force_policy_literature_parameter_audit.md`
- RH56 protocol and measurement: `research_log/rh56_force_bandwidth_final_summary.md`
- ACT live timing and predictions: `research_log/thor_act_shadow_benchmark.md`
- Schedule analysis: `research_log/act_execution_timing_recommendation.md`
- ForceGuard ranges: not frozen; the native register-information rate remains unresolved.
- RH56 stationary data: `outputs/force_shadow_20260811/rh56_force_bandwidth_stationary.json`
- Live ACT data: `outputs/force_shadow_20260811/live_shadow_300q_final/`
- Resource telemetry: `outputs/force_shadow_20260811/live_runtime_2/`

## Verification

- Full host suite: `661 passed, 4 skipped` (the skips require the selected container environment); two existing multiprocessing/fork deprecation warnings.
- Focused live-shadow/RH56/JAKA/backend suite: `42 passed`.
- LeRobot 0.6.2 GPU artifact-backed integration: `4 tests, OK` (checkpoint/config/processor reload, native normalization round trip, exact state/action order, episode exclusions/boundaries, finite `[16,12]` output).
- Python compile checks for all three new tools: PASS.
- `git diff --check`: PASS.

## Required remaining work before any physical ACT rollout

1. Review and approve the command-disabled shadow results, especially the JAKA-4 demonstration-envelope excursion.
2. Perform a manually varied, read-only RH56 test to establish unique update rate; keep production at 10 Hz until reviewed.
3. Add and validate command-disabled queue interruption/reset behavior for a two-action prefix and future force supervision.
4. Define an approved rollout pose, workspace, operator stop procedure, safety checklist, and conservative command bridge under the repository's hardware-safety process.
5. Run a dry command-sink integration that proves correct native action ordering, limits reporting without clamping, processor reload, reset semantics, and zero stale queued actions.
6. Obtain explicit authorization before enabling any physical command path.
