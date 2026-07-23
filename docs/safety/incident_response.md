# Incident response

On collision/servo alarm, estop, tracking fault, timing fault, unexpected
motion, or communication failure:

1. stop commanding and allow the existing cleanup path to finish;
2. do not immediately reconnect, re-enable, or retry the same motion;
3. record controller-visible alarms, robot/command state, timestamps, exact
   commit, config, executable, acknowledgement, and operator observations;
4. preserve raw logs without editing them;
5. reproduce offline or with the fake worker where possible;
6. add/fix regression coverage before proposing another bounded physical gate;
7. state whether the cause is proven, suspected, or unresolved.

The July 2026 Quest/JAKA incident sequence is preserved under
`docs/history/incidents/quest_jaka_20260722_23/`. It includes a J4 collision
alarm, payload correction reported by the operator, a failed two-session
health-monitor attempt with no motion, the later sole-session timing run, and
the offline output-acceleration correction. The historical outcomes must not be
rewritten as a single PASS.
