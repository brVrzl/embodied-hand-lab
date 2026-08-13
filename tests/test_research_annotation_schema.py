from __future__ import annotations

import json
from pathlib import Path

from embodiment_core.config import load_yaml


SCHEMA_PATH = Path("research/exploration/annotation_schema.json")
TEMPLATE_PATH = Path(
    "configs/experiments/icra2027_bottle_annotation_template.yaml"
)


def _schema() -> dict[str, object]:
    return json.loads(SCHEMA_PATH.read_text(encoding="utf-8"))


def test_all_annotation_schema_local_references_resolve() -> None:
    schema = _schema()

    def visit(value: object) -> None:
        if isinstance(value, dict):
            reference = value.get("$ref")
            if isinstance(reference, str) and reference.startswith("#/"):
                target: object = schema
                for component in reference[2:].split("/"):
                    component = component.replace("~1", "/").replace("~0", "~")
                    assert isinstance(target, dict)
                    assert component in target, reference
                    target = target[component]
            for child in value.values():
                visit(child)
        elif isinstance(value, list):
            for child in value:
                visit(child)

    visit(schema)


def test_annotation_schema_uses_standard_draft_and_closes_root() -> None:
    schema = _schema()

    assert schema["$schema"] == "https://json-schema.org/draft/2020-12/schema"
    assert schema["type"] == "object"
    assert schema["additionalProperties"] is False
    assert set(schema["properties"]) == set(schema["required"])


def test_annotation_schema_requires_exact_dataset_and_timeline_identity() -> None:
    definitions = _schema()["$defs"]
    identity = definitions["dataset_identity"]
    timeline = definitions["canonical_timeline"]
    interval = definitions["canonical_interval"]

    assert {
        "logical_dataset_id",
        "source_dataset_id",
        "session_id",
        "logical_episode_id",
        "source_episode_id",
        "dataset_fingerprint",
    } <= set(identity["required"])
    fingerprint = identity["properties"]["dataset_fingerprint"]
    assert set(fingerprint["required"]) == {"algorithm", "digest", "scope"}
    assert fingerprint["properties"]["algorithm"]["const"] == "sha256"

    assert timeline["properties"]["timestamp_field"]["const"] == "timestamp_ns"
    assert (
        timeline["properties"]["timestamp_domain"]["const"]
        == "host_monotonic_ns"
    )
    assert timeline["properties"]["frame_index_field"]["const"] == "frame_index"
    assert interval["required"] == ["start", "end"]
    for boundary in ("start", "end"):
        assert (
            interval["properties"][boundary]["$ref"]
            == "#/$defs/canonical_point"
        )


def test_annotation_schema_enforces_rgb_only_signal_blinding() -> None:
    blinding = _schema()["$defs"]["blinding"]
    required_hidden = {
        "force_act_hidden",
        "rh56_position_hidden",
        "jaka_state_hidden",
        "action_targets_hidden",
        "hand_grip_hidden",
        "action_status_hidden",
        "contact_clamp_hidden",
        "labels_locked_before_signal_access",
    }

    assert blinding["properties"]["mode"]["const"] == "rgb_only"
    assert required_hidden <= set(blinding["required"])
    for field in required_hidden:
        assert blinding["properties"][field]["const"] is True


def test_annotation_schema_names_only_the_reviewed_events_and_milestones() -> None:
    schema = _schema()
    events = schema["properties"]["events"]
    milestones = schema["properties"]["milestones"]

    assert set(events["properties"]) == {
        "closure_onset",
        "visible_interaction_onset",
        "lift_off",
        "placement_contact",
        "release_onset",
        "object_supported_after_release",
    }
    assert "visible_interaction_onset" not in events["required"]
    assert set(events["required"]) == set(events["properties"]) - {
        "visible_interaction_onset"
    }
    assert set(milestones["properties"]) == {
        "approach",
        "grasp",
        "lift",
        "transport",
        "place",
        "release",
    }
    assert set(milestones["required"]) == set(milestones["properties"])


def test_task_outcome_failure_stage_and_control_termination_are_separate() -> None:
    schema = _schema()
    failure = schema["$defs"]["failure_assessment"]
    stage_options = failure["properties"]["stage"]["oneOf"][0]["enum"]

    assert {"task_outcome", "failure", "control_termination"} <= set(
        schema["required"]
    )
    assert set(stage_options) == {
        "approach",
        "precontact",
        "grasp",
        "lift",
        "transport",
        "place",
        "release",
        "control_software_abort",
    }
    assert (
        schema["properties"]["task_outcome"]["$ref"]
        != schema["properties"]["control_termination"]["$ref"]
    )
    termination_sources = schema["$defs"]["control_termination"]["properties"][
        "evidence_source"
    ]["enum"]
    assert "rgb_only" not in termination_sources


def test_visible_slip_requires_direct_rgb_evidence_when_observed() -> None:
    slip = _schema()["$defs"]["visible_slip_assessment"]
    observed_rule = slip["allOf"][0]

    assert observed_rule["if"]["properties"]["status"]["const"] == "observed"
    assert "interval" in observed_rule["then"]["required"]
    assert (
        observed_rule["then"]["properties"]["direct_rgb_evidence"]["const"]
        is True
    )
    assert (
        observed_rule["else"]["properties"]["direct_rgb_evidence"]["const"]
        is False
    )


def test_annotation_template_tracks_schema_contract_without_schema_dependency() -> None:
    schema = _schema()
    template = load_yaml(TEMPLATE_PATH)

    assert set(template) == set(schema["required"])
    assert template["schema_version"] == schema["properties"]["schema_version"][
        "const"
    ]
    assert set(template["dataset_identity"]) == set(
        schema["$defs"]["dataset_identity"]["required"]
    )
    assert set(template["events"]) == set(
        schema["properties"]["events"]["properties"]
    )
    assert set(template["milestones"]) == set(
        schema["properties"]["milestones"]["properties"]
    )
    assert template["visible_slip"]["direct_rgb_evidence"] is False
    assert template["failure"]["stage"] is None
