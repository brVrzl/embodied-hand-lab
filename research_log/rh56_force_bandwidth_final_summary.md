# RH56 force bandwidth decision record

Date: 2026-08-12 (Asia/Shanghai)

## Decision

- Production `FORCE_ACT` polling remains **10 Hz**.
- The force-bandwidth investigation is closed for this repository freeze.
- The effective native register-information rate remains **unresolved**.
- No analog or physical sensor-bandwidth claim is made.

## Evidence retained

Read-only diagnostics showed that the host can poll `FORCE_ACT` substantially
faster than the production schedule without serial errors. A direct per-read
teleoperation trace achieved approximately 30 Hz at a 30 Hz request and
approximately 41–45 Hz at 45–60 Hz requests; the worker saturated near 45 Hz.
Because the dynamic human interactions were not synchronized across rates and
the higher-rate blocks did not expose a controlled, repeatable set of new
register states, increased polling cannot be interpreted as increased native
information. The result is therefore **`STILL_INSUFFICIENT_EVIDENCE`**, not a
claim of 30, 45, or 60 Hz sensing.

The earlier active-grasp experiment likewise found no safe basis for changing
the production schedule. No JAKA command was sent, no ACT control was enabled,
and no production control or recorder semantics were changed.

## Reproducibility and scope

Raw traces, plots, and generated analysis remain outside version control under
the ignored `outputs/` directory. The one-off polling and active-grasp tools and
their tests were removed when this diagnostic phase was closed. The normal
production reader and its 10 Hz scheduler remain intact.
