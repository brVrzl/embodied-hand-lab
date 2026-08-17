#!/usr/bin/env bash
set -Eeuo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
case "${1:-}" in
  "") ARGS=() ;;
  --require-image) ARGS=(--require-image) ;;
  --help|-h)
    printf 'usage: %s [--require-image]\n' "$0"
    printf 'Verify the pinned OpenPI and LeRobot training dependencies.\n'
    exit 0
    ;;
  *)
    printf 'usage: %s [--require-image]\n' "$0" >&2
    exit 2
    ;;
esac

"$ROOT_DIR/training/pi05/scripts/check_openpi.sh" "${ARGS[@]}"
"$ROOT_DIR/scripts/check_lerobot.sh" "${ARGS[@]}"
