# Frozen evaluation protocol (pre-policy-rollout)

Date: 2026-10-14. Task: Pick up the bottle and place it on the cardboard box.

Selection is independent of simulation outcomes: historical π validation winner4500;
ACT existing last/100000. Both use existing training statistics unchanged.

Execution: synchronous simulation at30Hz,20 MuJoCo steps of1/600s per action;
query current simulated RGB workspace/wrist and measured12D joint state;
consume h0..h7 then query again. Prediction lengths unchanged (ACT60,π16).
No temporal ensemble, offset based on wall time, manual policy patches or object inputs.
Native absolute arm radians; six hand closure fractions0..1. Project once at
scene actuator boundary onto arm model joint limits and hand[0,1]; retain raw
chunks and executed targets, report projected action/dimension fractions.
Episode reset clears simulator and policy history/RNG. Model seed20261014;
environment seed recorded separately. Episode limit40 simulated seconds.

Stages (ordered): approach hand grasp-center within0.10m; grasp/lift requires
hand-object contact and bottle-center rise>0.04m; transport horizontal displacement
>0.08m after lift; place requires actual box contact in its top support region;
release requires first four measured hand closure fractions<0.35 and zero
hand-bottle contacts; success requires released bottle remains on target for1s
with linear speed<0.025m/s and angular speed<0.25rad/s at episode end.
No contact-only, still-held, or near-target result counts as success.

Formal nominal20 distinct instances is gated on independent physical expert/replay
completion. Perturbed20 is additionally gated on that acceptance and a defensible
real-demonstration position/orientation coverage estimate. If those gates fail,
only development diagnostics are reported, not a qualified benchmark or model
ranking. Initial-state lists are retained before their execution. No duplicated
identical deterministic resets count as independent nominal scenes.

All started attempts stay in attempts.jsonl; completed/error records in episodes.jsonl
and CSV. Technical errors are retained separately from model task timeout and physical
instability, included in attempted denominators. Fixes create new output directories.
Warm latency excludes first query (compilation); real-time factor=sim time/wall time.
30Hz is the simulated actuator clock, not measured inference throughput.

Raw traces, videos, source RGB comparisons and weights remain local and untracked;
only lightweight evidence summaries and explicit paths are committed. No real device
connections are permitted. This is a single-task engineering investigation.
