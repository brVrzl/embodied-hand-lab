#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
RUNTIME_FILE="$ROOT_DIR/training/act/runtime.yaml"
LEROBOT_SOURCE=${LEROBOT_SOURCE:-$ROOT_DIR/third_party/lerobot}
REQUIRE_IMAGE=0

case "${1:-}" in
  "") ;;
  --require-image) REQUIRE_IMAGE=1 ;;
  --help|-h)
    printf 'usage: %s [--require-image]\n' "$0"
    printf 'Verify the pinned LeRobot source submodule and optional ACT image.\n'
    exit 0
    ;;
  *)
    printf 'usage: %s [--require-image]\n' "$0" >&2
    exit 2
    ;;
esac

EXPECTED_COMMIT=$(awk '$1 == "official_commit:" {print $2}' "$RUNTIME_FILE")
IMAGE=$(awk '$1 == "image:" {print $2}' "$RUNTIME_FILE")
EXPECTED_IMAGE_ID=$(awk '$1 == "image_id:" {print $2}' "$RUNTIME_FILE")

[[ -e "$LEROBOT_SOURCE/.git" ]] || {
  printf 'LeRobot source submodule is missing: %s\n' "$LEROBOT_SOURCE" >&2
  exit 1
}
ACTUAL_COMMIT=$(git -C "$LEROBOT_SOURCE" rev-parse HEAD)
[[ "$ACTUAL_COMMIT" == "$EXPECTED_COMMIT" ]] || {
  printf 'LeRobot commit mismatch: expected %s, got %s\n' "$EXPECTED_COMMIT" "$ACTUAL_COMMIT" >&2
  exit 1
}

printf 'lerobot_repo=%s\n' "$LEROBOT_SOURCE"
printf 'lerobot_commit=%s\n' "$ACTUAL_COMMIT"
printf 'lerobot_dirty_files=%s\n' "$(git -C "$LEROBOT_SOURCE" status --short | wc -l)"
printf 'container_image=%s\n' "$IMAGE"

if command -v docker >/dev/null 2>&1 && docker image inspect "$IMAGE" >/dev/null 2>&1; then
  ACTUAL_IMAGE_ID=$(docker image inspect "$IMAGE" --format '{{.Id}}')
  [[ "$ACTUAL_IMAGE_ID" == "$EXPECTED_IMAGE_ID" ]] || {
    printf 'LeRobot image mismatch: expected %s, got %s\n' "$EXPECTED_IMAGE_ID" "$ACTUAL_IMAGE_ID" >&2
    exit 1
  }
  printf 'container_image_id=%s\n' "$ACTUAL_IMAGE_ID"
elif (( REQUIRE_IMAGE )); then
  printf 'container image is not available: %s\n' "$IMAGE" >&2
  exit 1
else
  printf 'container_image=not_checked_or_missing\n'
fi

printf 'status=ready\n'
