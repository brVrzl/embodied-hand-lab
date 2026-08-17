#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPOSITORY_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
EXPERIMENT_ROOT=${PI05_EXPERIMENT_ROOT:-$REPOSITORY_ROOT/outputs/training/pi05_rh56}
OPENPI_REPO=${PI05_OPENPI_REPO:-$REPOSITORY_ROOT/third_party/openpi}
OPENPI_CACHE=${PI05_OPENPI_CACHE:-$EXPERIMENT_ROOT/openpi_cache}
JAX_CACHE=${PI05_JAX_CACHE:-$EXPERIMENT_ROOT/jax_cache}
IMAGE=${PI05_OPENPI_IMAGE:-jaka-openpi:thor-cuda13}
EXP_NAME=${PI05_EXP_NAME:-weekend}
STEPS=${PI05_STEPS:-20000}
BATCH_SIZE=${PI05_BATCH_SIZE:-4}
NUM_WORKERS=${PI05_NUM_WORKERS:-0}
SAVE_INTERVAL=${PI05_SAVE_INTERVAL:-100}
LOG_INTERVAL=${PI05_LOG_INTERVAL:-10}
KEEP_PERIOD=${PI05_KEEP_PERIOD:-500}
SEED=${PI05_SEED:-20260814}
LOG_DIR=$EXPERIMENT_ROOT/logs
RUN_LOG=$LOG_DIR/${EXP_NAME}.log
SUPERVISOR_LOG=$LOG_DIR/${EXP_NAME}.supervisor.log
PID_FILE=$EXPERIMENT_ROOT/${EXP_NAME}.supervisor.pid
LOCK_FILE=$EXPERIMENT_ROOT/${EXP_NAME}.supervisor.lock
STOP_FILE=$EXPERIMENT_ROOT/${EXP_NAME}.stop_requested
CONTAINER_ID_FILE=$EXPERIMENT_ROOT/${EXP_NAME}.container_id
STATUS_FILE=$EXPERIMENT_ROOT/status.json
CHECKPOINT_DIR=$EXPERIMENT_ROOT/checkpoints/pi05_rh56/$EXP_NAME

mkdir -p "$EXPERIMENT_ROOT" "$LOG_DIR" "$OPENPI_CACHE" "$JAX_CACHE"

now_utc() {
    date -u +%Y-%m-%dT%H:%M:%SZ
}

supervisor_pid() {
    if [[ -s "$PID_FILE" ]]; then
        cat "$PID_FILE"
    fi
}

supervisor_running() {
    if [[ ! -s "$PID_FILE" ]]; then
        return 1
    fi
    local pid
    pid=$(<"$PID_FILE")
    kill -0 "$pid" 2>/dev/null || return 1
    ps -p "$pid" -o args= | grep -Fq 'train_weekend.sh supervise'
}

latest_valid_step() {
    [[ -d "$CHECKPOINT_DIR" ]] || return 1
    local candidate
    while IFS= read -r candidate; do
        if [[ -d "$candidate/train_state" && -d "$candidate/params" && -d "$candidate/assets" ]]; then
            basename "$candidate"
            return 0
        fi
    done < <(find "$CHECKPOINT_DIR" -mindepth 1 -maxdepth 1 -type d -name '[0-9]*' -printf '%f\t%p\n' | sort -nr -k1,1 | cut -f2-)
    return 1
}

write_status() {
    local state=$1
    local message=$2
    local step
    step=$(latest_valid_step || true)
    python3 - "$STATUS_FILE" "$state" "$message" "$step" "$EXP_NAME" "$STEPS" "$RUN_LOG" "$CHECKPOINT_DIR" "$(supervisor_pid || true)" <<'PY'
import json
import os
import pathlib
import re
import shutil
import sys
from datetime import datetime, timezone

path, state, message, step, exp_name, steps, log_path, checkpoint_dir, pid = sys.argv[1:]
root = pathlib.Path(path).parent
root.mkdir(parents=True, exist_ok=True)
recent_losses = []
try:
    text = pathlib.Path(log_path).read_text(errors="replace")[-200_000:]
    pattern = re.compile(r"Step (\d+): grad_norm=([0-9.eE+-]+), loss=([0-9.eE+-]+), param_norm=([0-9.eE+-]+)")
    for match in pattern.finditer(text):
        recent_losses.append({
            "step": int(match.group(1)),
            "grad_norm": float(match.group(2)),
            "loss": float(match.group(3)),
            "param_norm": float(match.group(4)),
        })
    recent_losses = recent_losses[-5:]
except (OSError, ValueError):
    recent_losses = []
latest_checkpoint_timestamp = None
if step.isdigit():
    checkpoint_path = pathlib.Path(checkpoint_dir) / step
    try:
        latest_checkpoint_timestamp = datetime.fromtimestamp(
            checkpoint_path.stat().st_mtime, timezone.utc
        ).isoformat()
    except OSError:
        pass
payload = {
    "updated_utc": datetime.now(timezone.utc).isoformat(),
    "state": state,
    "message": message,
    "experiment": exp_name,
    "configured_steps": int(steps),
    "current_checkpoint_step": int(step) if step.isdigit() else None,
    "supervisor_pid": int(pid) if pid.isdigit() else None,
    "log_path": log_path,
    "checkpoint_dir": checkpoint_dir,
    "latest_checkpoint_timestamp": latest_checkpoint_timestamp,
    "recent_losses": recent_losses,
    "disk_free_bytes": shutil.disk_usage(root).free,
}
temporary = root / ".status.json.tmp"
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
os.replace(temporary, path)
PY
}

run_container() {
    local resume_flag=${1:-}
    rm -f "$CONTAINER_ID_FILE"
    docker run --rm --runtime nvidia --network none --ipc=host --shm-size=16g \
        --cidfile "$CONTAINER_ID_FILE" \
        -v "$REPOSITORY_ROOT:/workspace/embodied_lab:rw" \
        -v "$OPENPI_REPO:/app:ro" \
        -v "$OPENPI_CACHE:/root/.cache/openpi:rw" \
        -v "$JAX_CACHE:/root/.cache/jax:rw" \
        -w /workspace/embodied_lab \
        -e HF_LEROBOT_HOME=/workspace/embodied_lab/outputs/training/pi05_rh56/lerobot_home_v2 \
        -e PYTHONUNBUFFERED=1 \
        -e PYTHONPATH=/workspace/embodied_lab/experiments/pi05_rh56:/workspace/embodied_lab/src:/app/src \
        "$IMAGE" /.venv/bin/python experiments/pi05_rh56/scripts/openpi_runner.py train \
        --experiment-root /workspace/embodied_lab/outputs/training/pi05_rh56 \
        --exp-name "$EXP_NAME" \
        --steps "$STEPS" \
        --batch-size "$BATCH_SIZE" \
        --num-workers "$NUM_WORKERS" \
        --save-interval "$SAVE_INTERVAL" \
        --log-interval "$LOG_INTERVAL" \
        --keep-period "$KEEP_PERIOD" \
        --seed "$SEED" \
        $resume_flag
}

supervise() {
    exec 9>"$LOCK_FILE"
    flock -n 9 || exit 2
    local failures=0
    local window_start
    window_start=$(date +%s)
    trap 'write_status stopping "supervisor received termination"; exit 143' TERM INT
    while true; do
        local step
        step=$(latest_valid_step || true)
        local resume_flag=
        if [[ -n "$step" ]]; then
            resume_flag=--resume
            write_status resuming "resuming from checkpoint step $step"
        else
            write_status starting "starting from pi05_base"
        fi
        set +e
        {
            printf '%s start exp=%s resume=%s\n' "$(now_utc)" "$EXP_NAME" "${resume_flag:-false}"
            run_container "$resume_flag"
        } >>"$RUN_LOG" 2>&1
        local rc=$?
        set -e
        if [[ -f "$STOP_FILE" ]]; then
            write_status stopped "supervisor stopped by request"
            return 143
        fi
        if [[ $rc -eq 0 ]]; then
            write_status completed "trainer exited successfully at configured step limit"
            return 0
        fi
        local now
        now=$(date +%s)
        if (( now - window_start > 900 )); then
            failures=0
            window_start=$now
        fi
        failures=$((failures + 1))
        write_status crashed "trainer exit code $rc; automatic resume attempt $failures"
        if (( failures >= 3 )); then
            write_status failed "three trainer failures within the restart window; manual inspection required"
            return "$rc"
        fi
        sleep 60
    done
}

start() {
    if supervisor_running; then
        printf 'already running pid=%s\n' "$(supervisor_pid)"
        return 0
    fi
    rm -f "$STOP_FILE"
    write_status launching "launching detached supervisor"
    nohup setsid "$SCRIPT_DIR/train_weekend.sh" supervise >>"$SUPERVISOR_LOG" 2>&1 < /dev/null &
    local pid=$!
    printf '%s\n' "$pid" >"$PID_FILE"
    printf 'started supervisor pid=%s log=%s\n' "$pid" "$RUN_LOG"
}

stop() {
    touch "$STOP_FILE"
    if [[ -s "$CONTAINER_ID_FILE" ]]; then
        local container_id
        container_id=$(<"$CONTAINER_ID_FILE")
        docker stop --timeout 10 "$container_id" >/dev/null 2>&1 || true
    fi
    if supervisor_running; then
        local pid
        pid=$(supervisor_pid)
        kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid"
        for _ in {1..20}; do
            supervisor_running || break
            sleep 0.5
        done
        if supervisor_running; then
            kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid"
        fi
        printf 'termination requested for supervisor process group %s\n' "$pid"
    else
        printf 'not running\n'
    fi
}

status() {
    local state=stopped
    if supervisor_running; then
        state=running
    fi
    local step
    step=$(latest_valid_step || true)
    write_status "$state" "status query"
    printf 'state=%s pid=%s step=%s checkpoint=%s\n' "$state" "$(supervisor_pid || true)" "${step:-none}" "$CHECKPOINT_DIR"
    printf 'log=%s supervisor_log=%s\n' "$RUN_LOG" "$SUPERVISOR_LOG"
    df -h "$EXPERIMENT_ROOT"
    free -h
    timeout 5 nvidia-smi --query-gpu=name,temperature.gpu,utilization.gpu,memory.used,memory.total,power.draw --format=csv,noheader 2>/dev/null || true
    tail -n 20 "$RUN_LOG" 2>/dev/null || true
}

case ${1:-status} in
    start|resume)
        start
        ;;
    supervise)
        supervise
        ;;
    stop)
        stop
        ;;
    status)
        status
        ;;
    *)
        printf 'usage: %s {start|resume|stop|status}\n' "$0" >&2
        exit 2
        ;;
esac
