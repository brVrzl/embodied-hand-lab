# Blinded RGB annotation protocol for bottle manipulation

## Purpose and boundary

This protocol produces episode-level visual evidence for the ICRA 2027
exploration. It is an offline labeling procedure; it does not open or command
the JAKA, RH56DFX, cameras, or Quest. Completed annotations are derived
artifacts and must never replace or modify raw/master datasets.

The annotation pass is RGB-only. In particular, the reviewer must not see
RH56 `FORCE_ACT`, RH56 position, JAKA state, action targets, `hand_grip`, arm
`action_status`, contact-clamp state, or plots/overlays derived from them until
the RGB labels are locked. `FORCE_ACT` is native actuator/push-rod load
feedback. It is not fingertip force, tactile-array sensing, or contact-location
ground truth. `hand_grip`, `action_status`, and contact clamping are controller
or command-path state, not interaction ground truth.

The machine-readable contract is `annotation_schema.json`. Start from
`configs/experiments/icra2027_bottle_annotation_template.yaml`; write filled
records to an ignored derived-output area such as
`outputs/icra2027_bottle_annotations/`, not beside or inside the source data.

## Dataset identity and canonical time

Before viewing an episode, record all of the following:

- logical and source dataset IDs;
- capture session ID;
- logical and source episode IDs;
- the source/materialized dataset SHA-256 fingerprint and exactly what that
  fingerprint covers;
- every RGB evidence view, its repository- or dataset-relative path, media
  SHA-256, feature key, and canonical frame count.

Use an existing materialization fingerprint when it covers the source
inventory and materialization configuration. Otherwise build a deterministic
manifest of the exact reviewed inputs and hash that manifest. Never pair an
annotation with policy outputs or sensor traces when the fingerprint or any
logical/source/session ID differs. A filename match is not dataset identity.

The maintained staging rows use zero-based `frame_index` and
`timestamp_ns` in the `host_monotonic_ns` domain. Decoded frame `i` from both
canonical videos corresponds to canonical row `i`. Every observed event is an
inclusive uncertainty interval:

- `start` is the earliest canonical row on which the event may have occurred;
- `end` is the latest canonical row on which it may have occurred;
- use the same row for both bounds only when the event is unambiguous at one
  frame;
- verify both timestamp/frame pairs exist, refer to the same episode, are
  monotonic, and agree with the canonical row table.

Do not claim sub-frame timing. These bounds describe uncertainty in an event
onset, not the duration of a manipulation phase.

## Labeling workflow

1. Verify source IDs, fingerprints, video readability, frame counts, and the
   canonical alignment before labeling. Mark a missing or unreadable view in
   `evidence_views`; do not silently substitute another encode.
2. Present synchronized workspace and wrist RGB only. A player may expose
   canonical frame index and timestamp, frame stepping, slow playback, and
   synchronized scrubbing. Optional RGB-only motion cues may propose a region
   to inspect, but the reviewer must accept every label from direct RGB
   evidence.
3. Review the whole episode once without assigning labels. On a second pass,
   mark event uncertainty intervals, milestone outcomes, task outcome, visible
   slip, and the first supported failure stage. List the view IDs that directly
   support each non-indeterminate decision.
4. Use `unobservable` when occlusion, framing, blur, truncation, or ambiguity
   prevents a visual judgment. Use `not_observed` only after the relevant
   visible interval has been reviewed and the event is absent. Do not convert
   missing evidence into a negative label.
5. Record `high`, `medium`, or `low` confidence and a concise note for every
   assessment. Confidence describes evidence quality; it does not repair an
   unsupported label.
6. Complete the RGB sections and assign a unique `label_lock_id` before any
   sensor, action, controller, operator-outcome, or termination record is
   exposed. Preserve the locked record. A later correction creates a new
   annotation ID/revision; it does not overwrite the original.
7. Populate `control_termination` only after the RGB lock when run metadata or
   an operator record is needed. Do not revise visual task/milestone labels to
   agree with the termination record.
8. For a reliability subset, obtain an independent secondary RGB-only review.
   Adjudicate disagreements while still signal-blind. Report categorical
   agreement and event-boundary disagreement in frames or milliseconds; do
   not retain only the adjudicated labels.

This supports manual annotation immediately. A semi-automatic helper is
defensible only if it performs video decode/synchronization, frame navigation,
or RGB-only candidate proposal. A proposal based on `FORCE_ACT`, hand position,
action state, contact clamp, or future samples breaks the blinded contract.

## Observable event definitions

Each event is recorded as `observed`, `not_observed`, or `unobservable`.
`visible_interaction_onset` is optional because neither camera may expose a
defensible interaction cue; when it has been assessed but is occluded, retain
an explicit `unobservable` record.

- `closure_onset`: earliest/latest frame at which physical finger closure is
  visibly underway. Do not use `hand_grip` or the commanded hand target.
- `visible_interaction_onset`: earliest/latest frame with a direct visible cue
  that the hand/object interaction has begun, such as clear surface contact or
  object response. Proximity alone and controller contact state do not count.
- `lift_off`: earliest/latest frame at which the bottle visibly loses support
  from its pickup surface. If the support boundary is occluded, use
  `unobservable`.
- `placement_contact`: earliest/latest frame at which the bottle visibly makes
  contact with its intended destination support. A downward hand trajectory
  alone is insufficient.
- `release_onset`: earliest/latest frame at which opening or withdrawal that
  releases the bottle is visibly underway. Do not use the hand command.
- `object_supported_after_release`: earliest/latest frame at which the bottle
  is visibly supported at the destination without continued support from the
  hand. If continued hand support cannot be ruled out, use `unobservable`.

## Milestones and task outcome

Milestones are outcomes, not inferred phase labels. Mark `achieved`,
`not_achieved`, `unobservable`, or `not_applicable`:

- `approach`: the hand reaches a visually plausible pickup/pregrasp region for
  the target bottle without a visible approach-ending miss.
- `grasp`: the bottle is visibly secured well enough for the attempted pickup;
  closure or proximity by itself is not grasp success.
- `lift`: the bottle visibly leaves its pickup support.
- `transport`: the bottle is visibly carried from the pickup region toward the
  placement region without losing the grasp.
- `place`: the bottle becomes visibly supported at the intended destination.
- `release`: the hand visibly relinquishes support while the bottle remains
  supported at the destination.

The task outcome is separately `success`, `failure`, or `indeterminate`.
Success requires the task goal to be visible, including supported placement
after release. A completed recording is not automatically task success, and a
runtime abort is not automatically manipulation failure. If the task goal or
its failure is not visible, use `indeterminate` even when an operator or log
contains a success/failure claim.

## Failure taxonomy

Assign at most one stage: the earliest stage for which direct evidence shows
that the required manipulation progress was not achieved. The taxonomy
describes where progress ended; it does not diagnose why.

- `approach`: visible miss, collision/displacement, or terminal misalignment
  before a plausible pregrasp region is reached.
- `precontact`: a plausible approach is reached, but the attempt visibly ends
  before defensible interaction/grasp initiation.
- `grasp`: interaction/closure is attempted, but the bottle is not visibly
  secured for pickup.
- `lift`: a grasp appears secured, but lift-off is not achieved or is
  immediately lost.
- `transport`: the bottle lifts but is lost or transport cannot be completed.
- `place`: transport reaches the destination region, but supported placement
  is not achieved.
- `release`: placement is reached, but the hand does not relinquish support or
  the bottle fails as release occurs.
- `control_software_abort`: a post-lock run/control record establishes a
  control or software abort before task completion, with no earlier visually
  supported manipulation-stage failure.

Leave the stage null when the evidence does not distinguish one. For
`control_software_abort`, use only `post_lock_control_record` as its evidence
source and preserve the independently supported task outcome (often
`indeterminate`). Record runtime/recording status separately in
`control_termination`, including normal end, operator stop, control abort,
software abort, data-capture abort, or unknown.

## Visible slip

Mark slip `observed` only when consecutive RGB frames directly show unintended
relative motion between the bottle and grasping hand, or a visible progressive
loss of the grasp. Record the uncertainty interval and supporting view IDs, and
set `direct_rgb_evidence: true`.

Load changes, hand/action disagreement, contact clamping, apparent motion from
camera movement, motion blur, or an occluded bottle are not direct slip
evidence. In those cases use `unobservable` or `not_observed` as appropriate,
with `direct_rgb_evidence: false`. Later FORCE_ACT analyses may test prediction
of the locked visual label; they may not retroactively define it.

## Minimum quality checks before analysis

- Schema version, annotation ID, reviewer, notes, and lock ID are present.
- Dataset fingerprint and all logical/source/session IDs exactly match every
  paired artifact.
- Evidence-view IDs resolve uniquely and every available view hash/frame count
  matches the reviewed media.
- Observed event/slip intervals resolve to existing causal canonical rows;
  end is not earlier than start.
- No interval is attached to `not_observed` or `unobservable`.
- Every positive or negative visual judgment cites at least one evidence view.
- `visible_slip=observed` always has `direct_rgb_evidence=true`.
- Visual task outcome and control termination remain separate fields.
- The RGB lock predates any signal-conditioned analysis.

Treat annotations that fail these checks as unusable until corrected in a new
auditable revision. Do not alter source episodes to make an annotation pass.
