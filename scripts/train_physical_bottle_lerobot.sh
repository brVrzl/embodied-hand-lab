#!/usr/bin/env bash
# Repository-owned entrypoint for the current formal nominal52 ACT baseline.
#
# The immutable master dataset stays outside the container's write scope.
# Only the disposable LeRobot view and ignored training outputs are writable.

set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE="${LEROBOT_IMAGE:-jaka-lerobot-dev:snapshot-before-raw-mount}"
EXPECTED_IMAGE_ID="sha256:150b3af40810b763ce54ecdf8ff927dde3a36b00f5946feb9617ae823f5e8f1e"
LEROBOT_SOURCE="${LEROBOT_SOURCE:-$ROOT_DIR/third_party/lerobot}"
EXPECTED_LEROBOT_COMMIT="f66e5128ecb2456e8c54a63d15404fa59c16aebc"
CONTAINER_HOME="${LEROBOT_CONTAINER_HOME:-$ROOT_DIR/outputs/training/lerobot_container_home}"
MODE="${1:-strong-act}"

die() {
  echo "ERROR: $*" >&2
  exit 1
}

case "$MODE" in
  strong-act) ;;
  -h|--help)
    cat <<'EOF'
Usage: scripts/train_physical_bottle_lerobot.sh strong-act

Builds and validates the current human-audited nominal52 LeRobot view, then
starts the pinned official LeRobot 0.6.2 ACT trainer. Existing generated views
are reused only when they contain a prior loader validation report. Existing
training output directories are never overwritten. The launcher is
network-disabled and never runs a robot.
EOF
    exit 0
    ;;
  *) die "the only active mode is strong-act" ;;
esac
if [[ $# -gt 1 ]]; then
  die "this entrypoint accepts one mode argument"
fi

command -v docker >/dev/null 2>&1 || die "docker is required"
mkdir -p "$CONTAINER_HOME"
DATASET_NAME="physical_bottle_v4_nominal52"
MASTER="$ROOT_DIR/data/training/$DATASET_NAME/act/master"
VIEW="$ROOT_DIR/outputs/training/$DATASET_NAME/lerobot/act_strong_view"
OUTPUT="$ROOT_DIR/outputs/training/$DATASET_NAME/act_strong_run"
CONFIG="/workspace/embodied_lab/configs/training/lerobot/act_physical_bottle.json"
DATASET_CONFIG="/workspace/embodied_lab/configs/training/physical_bottle.yaml"
[[ -d "$MASTER" ]] || die "materialize the current physical_bottle dataset first"

actual_image_id="$(docker image inspect "$IMAGE" --format '{{.Id}}' 2>/dev/null || true)"
[[ "$actual_image_id" == "$EXPECTED_IMAGE_ID" ]] || \
  die "image $IMAGE is not the audited pinned image: $actual_image_id"
[[ -e "$LEROBOT_SOURCE/.git" ]] || die "LeRobot source submodule is missing: $LEROBOT_SOURCE"
actual_lerobot_commit="$(git -C "$LEROBOT_SOURCE" rev-parse HEAD 2>/dev/null || true)"
[[ "$actual_lerobot_commit" == "$EXPECTED_LEROBOT_COMMIT" ]] || \
  die "LeRobot source commit mismatch: expected $EXPECTED_LEROBOT_COMMIT, got $actual_lerobot_commit"

weights="$CONTAINER_HOME/.cache/torch/hub/checkpoints/resnet18-f37072fd.pth"
expected_weights_sha256="f37072fd47e89c5e827621c5baffa7500819f7896bbacec160b1a16c560e07ec"
[[ -f "$weights" ]] || die "cached ImageNet ResNet18 weights are missing: $weights"
[[ "$(sha256sum "$weights" | awk '{print $1}')" == "$expected_weights_sha256" ]] || \
  die "cached ImageNet ResNet18 weights failed the pinned checksum"

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

mkdir -p "$ROOT_DIR/outputs/training/$DATASET_NAME/logs"
if [[ -e "$VIEW" ]]; then
  [[ -f "$VIEW/meta/embodied_lab_loader_validation.json" ]] || \
    die "existing derived view is not validated; preserve it and choose a new view: $VIEW"
  echo "Reusing validated derived view: $VIEW"
else
  run_container python tools/lerobot_physical_bottle.py build-view \
    --master "/workspace/embodied_lab/data/training/$DATASET_NAME/act/master" \
    --view "/workspace/embodied_lab/outputs/training/$DATASET_NAME/lerobot/act_strong_view" \
    --kind act \
    --dataset-config "$DATASET_CONFIG"
fi

run_container python tools/lerobot_physical_bottle.py validate-view \
  --view "/workspace/embodied_lab/outputs/training/$DATASET_NAME/lerobot/act_strong_view" \
  --chunk-size 60
[[ ! -e "$OUTPUT" ]] || die "training output already exists: $OUTPUT"
LOG="$ROOT_DIR/outputs/training/$DATASET_NAME/logs/act_strong_training.log"
run_container python -m lerobot.scripts.lerobot_train "--config_path=$CONFIG" 2>&1 | tee -a "$LOG"

echo "Training completed under $OUTPUT"
