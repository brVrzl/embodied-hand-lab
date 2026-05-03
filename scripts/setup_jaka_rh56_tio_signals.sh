#!/usr/bin/env bash
set -euo pipefail

cd "$(dirname "$0")/.."

PYTHON_BIN="${PYTHON_BIN:-.venv/bin/python}"

echo "[setup_jaka_rh56_tio_signals] Adding RH56 state signals through JAKA TCP/IP JSON..."
"$PYTHON_BIN" tools/check_jaka_tcp_tio_signals.py \
  --compact \
  --terminator none \
  --prepare \
  --delete-existing \
  --delete-first \
  --add-signals \
  --polls "${POLLS:-3}" \
  --poll-sec "${POLL_SEC:-0.5}" \
  --channel-id "${CHANNEL_ID:-1}" \
  --slave-id "${SLAVE_ID:-1}" \
  --address-stride "${ADDRESS_STRIDE:-2}" \
  --sig-type "${SIG_TYPE:-3}" \
  --frequency-hz "${FREQUENCY_HZ:-0.0}" \
  --groups ${RH56_SIGNAL_GROUPS:-angle}
