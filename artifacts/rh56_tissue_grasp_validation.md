# RH56DFX tissue grasp and extraction validation

RH56DFX tissue grasp remains physically unverified.

No run in this task has completed the required sequence:

```text
grasp tissue -> extract -> hold -> release
```

Current prerequisites and evidence:

- Real Quest open/fist and thumb opposition feature endpoints have been captured and loaded by the hand-only path.
- All four RH56 fingers and both thumb channels can reach the existing 0.8 software endpoint in bounded free-space tests.
- Index and middle pinch intent are distinguishable in labelled human tracking data.
- No RH56 index, middle, or tripod fingertip-contact actuator pose has yet been visually confirmed.
- The bounded index-pinch search A--D produced no measurable contact load; it was stopped before the known high-opposition self-collision region and the hand was released to all-open.
- The first live run with the new calibration received zero right-hand landmarks and therefore issued zero RH56 writes.
- No tissue was contacted, held, extracted, or released; there is no grasp-success, slip-rate, or release-success evidence.

CURRENT/FORCE observations currently support only conservative data interpretation:

- free-space endpoint and index-pinch-search commands ended with zero CURRENT and low FORCE_ACT values;
- predictable index/thumb self-collision produced a large index FORCE_ACT increase and angle stall;
- retreating thumb lateral to minimum removed that load;
- no external-object contact transient or tissue-slip signature has yet been recorded.

This is enough to design contact detection from command/ANGLE stall plus CURRENT/FORCE change, but not enough to enable an automatic hold compensation loop. No hardware force/current/speed register, safety threshold, or software ceiling was changed.
