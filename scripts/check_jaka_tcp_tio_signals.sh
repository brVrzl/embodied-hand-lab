#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"

echo "[check_jaka_tcp_tio_signals] Probing JAKA TIO RS485 semaphores over TCP/IP JSON..."
"$PYTHON_BIN" tools/check_jaka_tcp_tio_signals.py "$@"
