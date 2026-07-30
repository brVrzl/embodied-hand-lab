# RH56DFX pinch retarget validation

Current physical outcome: FAIL / incomplete. No index, middle, or tripod RH56 fingertip-contact pose has yet been confirmed, so no pose blend is enabled and no tissue-grasp success is claimed.

## Human intent features

All distances are divided by `distance(wrist, middle MCP)`.

| labelled Quest pose | thumb-index | thumb-middle | index-middle | detector result |
|---|---:|---:|---:|---|
| open | 1.229 | 1.451 | 0.225 | none |
| fist | 0.538 | 0.670 | 0.164 | none (power-grasp exclusion) |
| index pinch | 0.051 | 0.258 | 0.228 | index on all 72 stable-tail frames |
| middle pinch | 0.733 | 0.102 | 0.757 | middle on all 73 stable-tail frames |
| attempted tripod | 0.703 | 0.046 | 0.693 | middle, correctly not tripod |

The detector uses separate thumb-index and thumb-middle distances, both finger curls, index-middle distance for tripod, and mean ring/pinky curl to exclude a strong power grasp. Entry is 0.15 palm lengths and exit is 0.22; the separate thresholds provide hysteresis. Tracking invalid/nonfinite input resets immediately to `none`. A true tripod additionally requires both thumb distances below the entry/exit threshold and index-middle below 0.22/0.30.

The first tripod capture was a valid but middle-only human pose. A retry delivered a completely frozen old skeleton (`ptp=0` for every checked landmark-derived feature) and was rejected. Therefore tripod intent remains physically uncaptured even though the synthetic three-distance classifier path is offline tested.

## Bounded RH56 index-pinch search

No historical raw six-channel array was sent. Starting from measured all-open state, only canonical index, thumb close, and thumb lateral were increased in four bounded steps; middle/ring/pinky stayed open. Every step used the production 40 Hz worker, 0.05 delta limit, measured activation write, 0.8 ceiling, latest-only mailbox, and unchanged RH56 feedback/fault gates.

| label | requested index / thumb close / lateral | measured index / thumb close / lateral | final index force | final current | software/hardware fault |
|---|---|---|---:|---:|---|
| A | 0.35 / 0.35 / 0.35 | 0.345 / 0.343 / 0.361 | -16 | 0 | none |
| B | 0.45 / 0.40 / 0.45 | 0.449 / 0.395 / 0.459 | -19 | 0 | none |
| C | 0.50 / 0.45 / 0.55 | 0.499 / 0.445 / 0.562 | -15 | 0 | none |
| D | 0.55 / 0.50 / 0.65 | 0.548 / 0.494 / 0.659 | -15 | 0 | none |
| release | 0 / 0 / 0 | 0 / 0.007 / 0.019 | -2 | 0 | none |

None of A--D produced a measurable contact load. Visual fingertip contact has not been confirmed by the operator in the recorded evidence, so these are search poses, not validated pinch presets. After D the hand was returned through the same bounded path to measured all-open state.

The earlier independent-ceiling test at index about 0.52, thumb close about 0.74, and thumb lateral about 0.81 produced predictable index/thumb self-collision and high force. This demonstrates why pinch cannot be created by independently maximizing closure/opposition and why no further automatic sweep was performed.

## Pose blending status

Not enabled. The detector and event diagnostics are implemented, but the required canonical normalized index/middle/tripod actuator poses do not yet have confirmed RH56 fingertip-contact geometry. Enabling a blend before that confirmation would violate the requirement that presets come from a current bounded physical validation rather than a guessed six-channel array.

Once each pose is confirmed, blending must occur after continuous relative retargeting and before submission, with confidence ramping, entry/exit hysteresis, maximum blend-weight rate, the existing channel delta/slew limits, and the unchanged 0.8 ceiling. Tracking loss must ramp/exit the assist and then follow the existing clutch hold/reacquisition contract.
