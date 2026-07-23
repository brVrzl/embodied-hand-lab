# Physical test gates

Physical progress is monotonic and evidence-based:

1. offline build/static/unit/replay validation;
2. read-only and no-motion checks;
3. plant-free/fake-worker transport validation;
4. separately authorized bounded physical gate;
5. evidence review before any larger envelope.

Every gate records branch/commit, config and executable identity, controller
state, acknowledgement, bounds, start/end state, stop reason, metrics, raw-log
relationship, and explicit PASS/FAIL/partial status. A pass applies only to the
tested path and envelope.

Historical Gates 1–3C are indexed under
`docs/history/gates/jaka_foundation_20260716/`. Gate 3C includes successful
minimal J6 motion for its exact historical procedure; it does not validate the
current full Quest pipeline.

The next recommended Quest/JAKA gate is described in
`docs/status/current_status.md`. This repository-maintenance session does not
authorize it.
