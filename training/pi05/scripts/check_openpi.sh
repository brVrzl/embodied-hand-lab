#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
EXPERIMENT_DIR=$(cd "$SCRIPT_DIR/.." && pwd)
REPOSITORY_ROOT=$(cd "$EXPERIMENT_DIR/../.." && pwd)
OPENPI_REPO=${PI05_OPENPI_REPO:-$REPOSITORY_ROOT/third_party/openpi}
OPENPI_IMAGE=${PI05_OPENPI_IMAGE:-jaka-openpi:thor-cuda13}
REQUIRE_IMAGE=0

case "${1:-}" in
    "") ;;
    --require-image) REQUIRE_IMAGE=1 ;;
    --help|-h)
        printf 'usage: %s [--require-image]\n' "$0"
        printf 'Verify the pinned OpenPI checkout and optional Thor container image.\n'
        exit 0
        ;;
    *)
        printf 'usage: %s [--require-image]\n' "$0" >&2
        exit 2
        ;;
esac

LOCK_FILE=$REPOSITORY_ROOT/configs/training/pi05/openpi.lock.json
EXPECTED_COMMIT=$(python3 - "$LOCK_FILE" <<'PY'
import json
import sys
print(json.loads(open(sys.argv[1], encoding="utf-8").read())["commit"])
PY
)

if [[ ! -e "$OPENPI_REPO/.git" ]]; then
    printf 'OpenPI checkout missing or is not a Git worktree: %s\n' "$OPENPI_REPO" >&2
    exit 1
fi

ACTUAL_COMMIT=$(git -C "$OPENPI_REPO" rev-parse HEAD)
if [[ "$ACTUAL_COMMIT" != "$EXPECTED_COMMIT" ]]; then
    printf 'OpenPI commit mismatch: expected %s, got %s\n' "$EXPECTED_COMMIT" "$ACTUAL_COMMIT" >&2
    exit 1
fi

for required in \
    src/openpi/models/pi0.py \
    src/openpi/models/pi0_config.py \
    src/openpi/training/config.py \
    src/openpi/training/data_loader.py \
    scripts/train.py; do
    if [[ ! -e "$OPENPI_REPO/$required" ]]; then
        printf 'OpenPI checkout is missing required path: %s\n' "$required" >&2
        exit 1
    fi
done

printf 'openpi_repo=%s\n' "$OPENPI_REPO"
printf 'openpi_commit=%s\n' "$ACTUAL_COMMIT"
printf 'openpi_dirty_files=%s\n' "$(git -C "$OPENPI_REPO" status --short | wc -l)"
printf 'openpi_image=%s\n' "$OPENPI_IMAGE"

if command -v docker >/dev/null 2>&1 && docker image inspect "$OPENPI_IMAGE" >/dev/null 2>&1; then
    printf 'container_image=present\n'
elif (( REQUIRE_IMAGE )); then
    printf 'container image is not available: %s\n' "$OPENPI_IMAGE" >&2
    exit 1
else
    printf 'container_image=not_checked_or_missing\n'
fi

printf 'status=ready\n'
