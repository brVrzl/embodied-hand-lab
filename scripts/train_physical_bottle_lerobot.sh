#!/usr/bin/env bash
# Repository-owned entrypoint for the official LeRobot 0.6.2 ACT trainer.
#
# The raw/master datasets stay outside the container's write scope.  Only the
# disposable LeRobot views and ignored training outputs are written below
# outputs/training/physical_bottle_v2.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LEROBOT_IMAGE:-jaka-lerobot-dev:snapshot-before-raw-mount}"
EXPECTED_IMAGE_ID="sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e"
CONTAINER_HOME="${LEROBOT_CONTAINER_HOME:-/home/thor/LeRobot/container-home}"
MODE="${1:-both}"
STRONG_STAGE="${2:-2000}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

case "$MODE" in
  act|act-force|both|val4|strong-pretrained) ;;
  -h|--help)
    cat <<'EOF'
Usage: scripts/train_physical_bottle_lerobot.sh [act|act-force|both|val4]
       scripts/train_physical_bottle_lerobot.sh strong-pretrained [2000|5000|10000]

Builds and validates the disposable LeRobot v3 view, then starts the pinned
official LeRobot 0.6.2 ACT trainer. `both` runs ACT and ACT+Force sequentially.
Existing generated views are reused only when they contain a prior loader
validation report. Existing training output directories are never overwritten.
`strong-pretrained` reuses the val4 ACT view and the locally cached audited
ImageNet ResNet18 weights; the launcher remains network-disabled. Each later
stage resumes only from the preceding reviewed checkpoint.
EOF
    exit 0
    ;;
  *) die "mode must be act, act-force, both, val4, or strong-pretrained" ;;
esac
if [[ "$MODE" == "strong-pretrained" ]]; then
  case "$STRONG_STAGE" in
    2000|5000|10000) ;;
    *) die "strong-pretrained stage must be 2000, 5000, or 10000" ;;
  esac
elif [[ $# -gt 1 ]]; then
  die "a training stage is accepted only for strong-pretrained"
fi

command -v docker >/dev/null 2>&1 || die "docker is required"
[[ -d "$CONTAINER_HOME" ]] || die "container home does not exist: $CONTAINER_HOME"
[[ -d "$ROOT_DIR/data/training/physical_bottle_v2/act" ]] || die "run physical bottle materialization first"
[[ -d "$ROOT_DIR/data/training/physical_bottle_v2/act_force" ]] || die "run physical bottle materialization first"

actual_image_id="$(docker image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null || true)"
[[ "$actual_image_id" == "$EXPECTED_IMAGE_ID" ]] || die "image $IMAGE is not the audited pinned image: $actual_image_id"

run_container() {
  docker run --rm --runtime=nvidia --ipc=host --network none \
    -v "$ROOT_DIR:/workspace/embodied_lab:rw" \
    -v "$CONTAINER_HOME:/home/lerobot:rw" \
    -w /workspace/embodied_lab \
    -e HOME=/home/lerobot \
    "$IMAGE" "$@"
}

run_one() {
  local kind="$1"
  local variant="${2:-default}"
  local master="$ROOT_DIR/data/training/physical_bottle_v2/$kind/master"
  local view_name="${kind}_view"
  local run_name="${kind}_run"
  local config_name="act_physical_bottle_v2.json"
  local split_args=()
  if [[ "$variant" == "val4" ]]; then
    view_name="${kind}_val4_view"
    run_name="${kind}_val4_run"
    config_name="act_physical_bottle_v2_val4.json"
    split_args=(--split-config /workspace/embodied_lab/configs/training/physical_bottle_v2_val4.yaml)
  fi
  if [[ "$variant" == "strong-pretrained" ]]; then
    [[ "$kind" == "act" ]] || die "strong-pretrained is defined only for standard ACT"
    view_name="act_val4_view"
    run_name="act_strong_pretrained_val4_run"
    config_name="act_physical_bottle_v2_strong_pretrained_val4.json"
    split_args=(--split-config /workspace/embodied_lab/configs/training/physical_bottle_v2_val4.yaml)
    local weights="$CONTAINER_HOME/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth"
    local expected_weights_sha256="f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec"
    [[ -f "$weights" ]] || die "cached ImageNet ResNet18 weights are missing: $weights"
    [[ "$(sha256sum "$weights" | awk '{print $1}')" == "$expected_weights_sha256" ]] || \
      die "cached ImageNet ResNet18 weights failed the pinned checksum"
  fi
  local view="$ROOT_DIR/outputs/training/physical_bottle_v2/lerobot/$view_name"
  local config="/workspace/embodied_lab/configs/training/lerobot/$config_name"
  local label="act"
  if [[ "$kind" == "act_force" ]]; then
    config_name="act_force_physical_bottle_v2.json"
    if [[ "$variant" == "val4" ]]; then
      config_name="act_force_physical_bottle_v2_val4.json"
    fi
    config="/workspace/embodied_lab/configs/training/lerobot/$config_name"
    label="act_force"
  fi
  local trainer_args=(--config_path="$config")
  mkdir -p "$ROOT_DIR/outputs/training/physical_bottle_v2/logs"
  if [[ -e "$view" ]]; then
    [[ -f "$view/meta/embodied_lab_loader_validation.json" ]] || \
      die "existing derived view is not validated; preserve it and choose a new view: $view"
    echo "Reusing validated derived view: $view"
  else
    run_container python tools/lerobot_physical_bottle.py build-view \
      --master "/workspace/embodied_lab/data/training/physical_bottle_v2/$kind/master" \
      --view "/workspace/embodied_lab/outputs/training/physical_bottle_v2/lerobot/$view_name" \
      --kind "$kind" "${split_args[@]}"
  fi
  run_container python tools/lerobot_physical_bottle.py validate-view \
    --view "/workspace/embodied_lab/outputs/training/physical_bottle_v2/lerobot/$view_name"
  local output="$ROOT_DIR/outputs/training/physical_bottle_v2/$run_name"
  if [[ "$variant" == "strong-pretrained" && "$STRONG_STAGE" != "2000" ]]; then
    local previous_stage="002000"
    if [[ "$STRONG_STAGE" == "10000" ]]; then
      previous_stage="005000"
    fi
    local checkpoint="$output/checkpoints/$previous_stage"
    [[ -d "$checkpoint/pretrained_model" && -d "$checkpoint/training_state" ]] || \
      die "reviewed prerequisite checkpoint is missing: $checkpoint"
    trainer_args+=(
      --resume=true
      --checkpoint_path="/workspace/embodied_lab/outputs/training/physical_bottle_v2/$run_name/checkpoints/$previous_stage"
      --steps="$STRONG_STAGE"
    )
  else
    [[ ! -e "$output" ]] || die "training output already exists: $output"
  fi
  local log="$ROOT_DIR/outputs/training/physical_bottle_v2/logs/${label}_${variant}_training.log"
  if [[ "$variant" == "strong-pretrained" ]]; then
    log="$ROOT_DIR/outputs/training/physical_bottle_v2/logs/${label}_${variant}_${STRONG_STAGE}_training.log"
  fi
  run_container python -m lerobot.scripts.lerobot_train "${trainer_args[@]}" 2>&1 | tee "$log"
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
if [[ "$MODE" == "strong-pretrained" ]]; then
  run_one act strong-pretrained
fi

echo "Training completed under $ROOT_DIR/outputs/training/physical_bottle_v2"
