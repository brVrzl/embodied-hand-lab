# ACT execution timing recommendation

Date: 2026-08-11 (Asia/Shanghai)

## Recommendation for a first physical rollout

This is a future deployment contract, not authorization to command the robot.

| Parameter | Recommended initial value | Basis and classification |
|---|---:|---|
| ACT query frequency | 15 Hz | **Exploratory ablation value**, constrained by Thor timing; also has a **direct literature precedent** in the phase-conditioned ACT system, but that different robot does not prove suitability here |
| consumed actions per query (`n_action_steps`) | 2 of the predicted 16 | **Exploratory ablation value**, selected to limit open-loop duration while retaining large measured timing margin |
| accepted action cadence | 30 Hz | existing dataset/control timing requirement, not newly inferred from literature |
| camera observations | 30 Hz | existing validated dataset/live-camera rate |
| JAKA measured state | 30 Hz minimum at policy assembly | existing validated live read rate |
| RH56 measured angle | retain 15 Hz | existing production scheduler setting; no change justified by this task |
| `FORCE_ACT` acquisition | retain 10 Hz pending dynamic measurement | current software setting; higher effective information rate remains unproven |

With `n_action_steps=2`, each query contributes 66.667 ms of 30 Hz actions. Thor's measured p99 command-disabled observation-to-native-action latency is 27.942 ms, leaving 38.724 ms p99 margin; the worst observed query leaves 35.050 ms. This is a **derived result from measured Thor timing**.

The recommendation does not change the existing low-level JAKA interpolation/ServoJ cadence, safety limits, watchdogs, or RH56 command scheduler. Those remain under the existing controller's requirements.

## Offline schedule evaluation

The saved model predicts `H=16`; LeRobot's ACT implementation slices `predict_action_chunk(...)[:, :n_action_steps]` and places only those actions in its execution queue. Therefore consuming fewer than 16 predicted actions does not require retraining. With temporal ensembling disabled, the policy queries again only when that queue is empty. A reset must clear the queue and any policy state before the first observation of an episode.

All candidate values below are **exploratory ablation values**. Periods and margins are **derived from the measured 30 Hz data cadence and 27.942 ms p99 / 31.617 ms maximum Thor latency**.

| `n_action_steps` | Re-query period | Required inference rate | p99 latency margin | Worst-observed margin | Open-loop actions before re-query | Timing support | Force-supervision interpretation |
|---:|---:|---:|---:|---:|---:|---|---|
| 1 | 33.333 ms | 30 Hz | 5.391 ms | 1.717 ms | 33.333 ms | p99 supported, thin maximum margin | most interruptible; leaves little system jitter margin |
| 2 | 66.667 ms | 15 Hz | 38.724 ms | 35.050 ms | 66.667 ms | comfortably supported | short queue suitable for first supervised trials |
| 4 | 133.333 ms | 7.5 Hz | 105.391 ms | 101.717 ms | 133.333 ms | supported | slower policy response; supervisor must interrupt queued actions explicitly |
| 8 | 266.667 ms | 3.75 Hz | 238.724 ms | 235.050 ms | 266.667 ms | supported | substantial open-loop interval |
| 16 | 533.333 ms | 1.875 Hz | 505.391 ms | 501.717 ms | 533.333 ms | supported | full trained chunk, least responsive |

“Supported” means only that measured shadow latency meets the deadline. It does not mean safe or task-effective.

## Literature constraints on interpretation

- RDP directly demonstrates a multi-timescale design: a 1–2 Hz slow diffusion policy with a 24 FPS force/tactile-conditioned fast correction path and >500 Hz interpolated command delivery. This supports decoupling policy planning from physical feedback; it does not specify our `n_action_steps`.
- M²-ResiPolicy similarly reports 10 Hz master inference, 60 Hz wrench residual correction, and 125 Hz force-mixed PBIC.
- The phase-conditioned ACT paper directly reports 30 Hz demonstrations and 15 Hz ACT target prediction. It also reports worse smoothness/precision when taking every third or fifth action from each chunk. That observation favors orderly short-prefix consumption, not arbitrary action subsampling.
- FACTR's chunk size 100 and FACTR 2's chunk size 30 are architecture/task settings without reported policy execution rates, so neither supports copying a physical open-loop duration.

## Queue and later supervision contract

For the proposed first schedule, a query produces 16 predictions but only the first two enter the normal 30 Hz execution queue. A later force supervisor must be able to cancel the remaining queued prefix before substituting or stopping actions; it must never append a corrective action behind stale queued actions. The exact interruption behavior must be validated in a command-disabled integration test before rollout. No ForceGuard logic was implemented in this task.

The 30 Hz one-step schedule is a sensible later responsiveness ablation because measured p99 fits, but its 1.717 ms worst-observed margin is not the preferred first configuration. The 4/8/16-step schedules remain offline comparisons because their longer open-loop intervals weaken responsiveness without evidence of a task benefit.
