#!/usr/bin/env bash
# Offline teacher-forced evaluation for every saved nominal52 strong-ACT checkpoint.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LEROBOT_IMAGE:-jaka-lerobot-dev:snapshot-before-raw-mount}"
EXPECTED_IMAGE_ID="sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e"
CONTAINER_HOME="${LEROBOT_CONTAINER_HOME:-/home/thor/LeRobot/container-home}"
RUN_ROOT="$ROOT_DIR/outputs/training/physical_bottle_v4_nominal52/act_strong_run"
VIEW="$ROOT_DIR/outputs/training/physical_bottle_v4_nominal52/lerobot/act_strong_view"
ANALYSIS="$ROOT_DIR/outputs/training/physical_bottle_v4_nominal52/analysis"
CURATION="$ANALYSIS/horizon_coverage.json"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

command -v docker >/dev/null 2>&1 || die "docker is required"
[[ -f "$CURATION" ]] || die "run tools/analyze_act_horizon_coverage.py first"
[[ -f "$VIEW/meta/embodied_lab_loader_validation.json" ]] || die "validated strong view is missing"
actual_image_id="$(docker image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null || true)"
[[ "$actual_image_id" == "$EXPECTED_IMAGE_ID" ]] || die "image $IMAGE is not the audited pinned image"

run_container() {
  docker run --rm --runtime=nvidia --ipc=host --network none \
    -v "$ROOT_DIR:/workspace/embodied_lab:rw" \
    -v "$CONTAINER_HOME:/home/lerobot:rw" \
    -w /workspace/embodied_lab \
    -e HOME=/home/lerobot \
    "$IMAGE" "$@"
}

mapfile -t checkpoints < <(find "$RUN_ROOT/checkpoints" -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort)
[[ ${#checkpoints[@]} -gt 0 ]] || die "no checkpoints found under $RUN_ROOT/checkpoints"

for step in "${checkpoints[@]}"; do
  checkpoint="$RUN_ROOT/checkpoints/$step/pretrained_model"
  output="$ANALYSIS/checkpoints/$step"
  [[ -d "$checkpoint" ]] || die "checkpoint policy is incomplete: $checkpoint"
  if [[ ! -f "$output/teacher_forced_replay.json" ]]; then
    run_container python tools/lerobot_physical_bottle.py evaluate-checkpoint \
      --view /workspace/embodied_lab/outputs/training/physical_bottle_v4_nominal52/lerobot/act_strong_view \
      --checkpoint "/workspace/embodied_lab/outputs/training/physical_bottle_v4_nominal52/act_strong_run/checkpoints/$step/pretrained_model" \
      --output "/workspace/embodied_lab/outputs/training/physical_bottle_v4_nominal52/analysis/checkpoints/$step" \
      --chunk-size 60 \
      --batch-size 32 \
      --num-workers 8
  fi
  PYTHONPATH="$ROOT_DIR/src" "$ROOT_DIR/.venv/bin/python" \
    "$ROOT_DIR/tools/analyze_act_nominal_checkpoint.py" \
    --arrays "$output/teacher_forced_arrays.npz" \
    --curation "$CURATION" \
    --output "$output" \
    --label "nominal52_strong_${step}"
done
