#!/usr/bin/env bash
# Repository-owned entrypoint for the official LeRobot 0.6.2 ACT trainer.
#
# The raw/master datasets stay outside the container's write scope.  Only the
# disposable LeRobot views and ignored training outputs are written below
# outputs/training/physical_bottle_v2. The source dataset ID is historical v2;
# its retained configuration names this formal mixed-quality reproducibility
# baseline explicitly.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LEROBOT_IMAGE:-jaka-lerobot-dev:snapshot-before-raw-mount}"
EXPECTED_IMAGE_ID="sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e"
LEROBOT_SOURCE="${LEROBOT_SOURCE:-$ROOT_DIR/third_party/lerobot}"
EXPECTED_LEROBOT_COMMIT="f66e5128ecb2456e8c54a63d15404fa59c16aebc"
CONTAINER_HOME="${LEROBOT_CONTAINER_HOME:-$ROOT_DIR/outputs/training/lerobot_container_home}"
MODE="${1:-both}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

case "$MODE" in
  act|act-force|both|val4|strong-act) ;;
  -h|--help)
    cat <<'EOF'
Usage: scripts/train_physical_bottle_lerobot.sh [act|act-force|both|val4|strong-act]

Builds and validates the disposable LeRobot v3 view, then starts the pinned
official LeRobot 0.6.2 ACT trainer. `both` runs ACT and ACT+Force sequentially.
Existing generated views are reused only when they contain a prior loader
validation report. Existing training output directories are never overwritten.
`act`, `act-force`, `both`, and `val4` reproduce the retained mixed-quality
LeRobot runs. `strong-act` trains the current nominal52 baseline. The launcher
is network-disabled.
`strong-act` trains the fixed nominal52 session split for 100k steps with the
canonical-size ACT and a 60-step prediction horizon. It does not run a robot.
EOF
    exit 0
    ;;
  *) die "mode must be act, act-force, both, val4, or strong-act" ;;
esac
if [[ $# -gt 1 ]]; then
  die "this entrypoint accepts one mode argument"
fi

command -v docker >/dev/null 2>&1 || die "docker is required"
mkdir -p "$CONTAINER_HOME"
if [[ "$MODE" == "strong-act" ]]; then
  [[ -d "$ROOT_DIR/data/training/physical_bottle_v4_nominal52/act" ]] || \
    die "materialize physical_bottle_v4_nominal52 first"
else
  [[ -d "$ROOT_DIR/data/training/physical_bottle_v2/act" ]] || die "run physical bottle materialization first"
  [[ -d "$ROOT_DIR/data/training/physical_bottle_v2/act_force" ]] || die "run physical bottle materialization first"
fi

actual_image_id="$(docker image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null || true)"
[[ "$actual_image_id" == "$EXPECTED_IMAGE_ID" ]] || die "image $IMAGE is not the audited pinned image: $actual_image_id"
[[ -e "$LEROBOT_SOURCE/.git" ]] || die "LeRobot source submodule is missing: $LEROBOT_SOURCE"
actual_lerobot_commit="$(git -C "$LEROBOT_SOURCE" rev-parse HEAD 2>/dev/null || true)"
[[ "$actual_lerobot_commit" == "$EXPECTED_LEROBOT_COMMIT" ]] || \
  die "LeRobot source commit mismatch: expected $EXPECTED_LEROBOT_COMMIT, got $actual_lerobot_commit"

run_container() {
  docker run --rm --runtime=nvidia --ipc=host --network none \
    -v "$ROOT_DIR:/workspace/embodied_lab:rw" \
    -v "$LEROBOT_SOURCE:/workspace/lerobot_source:ro" \
    -v "$CONTAINER_HOME:/home/lerobot:rw" \
    -w /workspace/embodied_lab \
    -e HOME=/home/lerobot \
    -e PYTHONPATH=/workspace/lerobot_source/src \
    "$IMAGE" "$@"
}

run_one() {
  local kind="$1"
  local variant="${2:-default}"
  local dataset_name="physical_bottle_v2"
  local master="$ROOT_DIR/data/training/$dataset_name/$kind/master"
  local view_name="${kind}_view"
  local run_name="${kind}_run"
  local config_name="act_physical_bottle_mixed.json"
  local chunk_size=16
  local split_args=()
  if [[ "$variant" == "val4" ]]; then
    view_name="${kind}_val4_view"
    run_name="${kind}_val4_run"
    config_name="act_physical_bottle_mixed_val4.json"
    split_args=(--split-config /workspace/embodied_lab/configs/training/physical_bottle_mixed_val4.yaml)
  fi
  if [[ "$variant" == "strong" ]]; then
    [[ "$kind" == "act" ]] || die "strong baseline is defined only for standard ACT"
    dataset_name="physical_bottle_v4_nominal52"
    master="$ROOT_DIR/data/training/$dataset_name/act/master"
    view_name="act_strong_view"
    run_name="act_strong_run"
    config_name="act_physical_bottle_nominal52_strong.json"
    chunk_size=60
    split_args=(--split-config /workspace/embodied_lab/configs/training/physical_bottle_nominal52_split.yaml)
    local weights="$CONTAINER_HOME/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth"
    local expected_weights_sha256="f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec"
    [[ -f "$weights" ]] || die "cached ImageNet ResNet18 weights are missing: $weights"
    [[ "$(sha256sum "$weights" | awk '{print $1}')" == "$expected_weights_sha256" ]] || \
      die "cached ImageNet ResNet18 weights failed the pinned checksum"
  fi
  local view="$ROOT_DIR/outputs/training/$dataset_name/lerobot/$view_name"
  local config="/workspace/embodied_lab/configs/training/lerobot/$config_name"
  local label="act"
  if [[ "$kind" == "act_force" ]]; then
    config_name="act_force_physical_bottle_mixed.json"
    if [[ "$variant" == "val4" ]]; then
      config_name="act_force_physical_bottle_mixed_val4.json"
    fi
    config="/workspace/embodied_lab/configs/training/lerobot/$config_name"
    label="act_force"
  fi
  local trainer_args=(--config_path="$config")
  mkdir -p "$ROOT_DIR/outputs/training/$dataset_name/logs"
  if [[ -e "$view" ]]; then
    [[ -f "$view/meta/embodied_lab_loader_validation.json" ]] || \
      die "existing derived view is not validated; preserve it and choose a new view: $view"
    echo "Reusing validated derived view: $view"
  else
    run_container python tools/lerobot_physical_bottle.py build-view \
      --master "/workspace/embodied_lab/data/training/$dataset_name/$kind/master" \
      --view "/workspace/embodied_lab/outputs/training/$dataset_name/lerobot/$view_name" \
      --kind "$kind" "${split_args[@]}"
  fi
  run_container python tools/lerobot_physical_bottle.py validate-view \
    --view "/workspace/embodied_lab/outputs/training/$dataset_name/lerobot/$view_name" \
    --chunk-size "$chunk_size"
  local output="$ROOT_DIR/outputs/training/$dataset_name/$run_name"
  [[ ! -e "$output" ]] || die "training output already exists: $output"
  local log="$ROOT_DIR/outputs/training/$dataset_name/logs/${label}_${variant}_training.log"
  run_container python -m lerobot.scripts.lerobot_train "${trainer_args[@]}" 2>&1 | tee -a "$log"
}

if [[ "$MODE" == "act" || "$MODE" == "both" ]]; then
  run_one act
fi
if [[ "$MODE" == "act-force" || "$MODE" == "both" ]]; then
  run_one act_force
fi
if [[ "$MODE" == "val4" ]]; then
  run_one act val4
  run_one act_force val4
fi
if [[ "$MODE" == "strong-act" ]]; then
  run_one act strong
fi

echo "Training completed under $ROOT_DIR/outputs/training"
