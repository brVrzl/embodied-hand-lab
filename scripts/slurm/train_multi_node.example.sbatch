#!/usr/bin/env bash
# Copy this file, replace or remove every angle-bracket placeholder, and
# confirm the site's supported srun/torchrun rendezvous pattern with the
# administrator. Some sites use --gpus-per-node instead of --gres.
#SBATCH --job-name=<JOB_NAME>
#SBATCH --account=<ACCOUNT>
#SBATCH --partition=<PARTITION>
#SBATCH --qos=<QOS>
#SBATCH --nodes=<NODE_COUNT>
#SBATCH --ntasks-per-node=1
#SBATCH --gres=gpu:<GPU_COUNT_PER_NODE>
#SBATCH --cpus-per-task=<CPU_COUNT>
#SBATCH --mem=<MEMORY>
#SBATCH --time=<WALL_TIME>
#SBATCH --output=slurm-%x-%j.out
#SBATCH --error=slurm-%x-%j.err
##SBATCH --mail-type=END,FAIL
##SBATCH --mail-user=<EMAIL>
##SBATCH --signal=B:TERM@120

set -euo pipefail

usage() {
  echo "Usage: sbatch $0 TRAINER_MODULE CONFIG_PATH [TRAINER_ARGS ...]" >&2
  echo "No trainer module currently ships in this repository." >&2
}

if (( $# < 2 )); then
  usage
  exit 2
fi

TRAINING_MODULE="$1"
TRAINING_CONFIG="$2"
shift 2

REPOSITORY_ROOT="${REPOSITORY_ROOT:-${SLURM_SUBMIT_DIR:-}}"
if [[ -z "$REPOSITORY_ROOT" || ! -d "$REPOSITORY_ROOT" ]]; then
  echo "Set REPOSITORY_ROOT or submit from the repository directory." >&2
  exit 2
fi
cd "$REPOSITORY_ROOT"

if [[ ! -f "$TRAINING_CONFIG" ]]; then
  echo "Training config does not exist: $TRAINING_CONFIG" >&2
  exit 2
fi

if [[ -n "${TRAINING_ACTIVATE:-}" ]]; then
  if [[ ! -f "$TRAINING_ACTIVATE" ]]; then
    echo "TRAINING_ACTIVATE does not exist: $TRAINING_ACTIVATE" >&2
    exit 2
  fi
  # The activation file is site-owned and must be reviewed before use.
  # shellcheck source=/dev/null
  source "$TRAINING_ACTIVATE"
fi

PYTHON_BIN="${PYTHON_BIN:-python}"
: "${GPUS_PER_NODE:?Set GPUS_PER_NODE to match the per-node Slurm GPU request}"
if [[ ! "$GPUS_PER_NODE" =~ ^[1-9][0-9]*$ ]]; then
  echo "GPUS_PER_NODE must be a positive integer, got: $GPUS_PER_NODE" >&2
  exit 2
fi

: "${SLURM_NNODES:?This template must run inside a Slurm allocation}"
: "${SLURM_JOB_NODELIST:?Slurm did not provide an allocated host list}"
readarray -t ALLOCATED_HOSTS < <(scontrol show hostnames "$SLURM_JOB_NODELIST")
if (( ${#ALLOCATED_HOSTS[@]} != SLURM_NNODES )); then
  echo "Allocated hostname count does not match SLURM_NNODES." >&2
  exit 2
fi

MASTER_ADDR="${MASTER_ADDR:-${ALLOCATED_HOSTS[0]}}"
MASTER_PORT="${MASTER_PORT:-29500}"
if [[ ! "$MASTER_PORT" =~ ^[0-9]+$ ]] \
  || (( MASTER_PORT < 1024 || MASTER_PORT > 65535 )); then
  echo "MASTER_PORT must be an available port from 1024 through 65535." >&2
  exit 2
fi

NNODES="$SLURM_NNODES"
RUN_ID="${RUN_ID:-${SLURM_JOB_NAME:-training}-${SLURM_JOB_ID:-manual}}"
OUTPUT_ROOT="${OUTPUT_ROOT:-artifacts/training}"
TRAINING_RUN_DIR="$OUTPUT_ROOT/$RUN_ID"
mkdir -p "$TRAINING_RUN_DIR"

export PYTHON_BIN TRAINING_MODULE TRAINING_CONFIG GPUS_PER_NODE
export MASTER_ADDR MASTER_PORT NNODES TRAINING_RUN_DIR

echo "Launching module=$TRAINING_MODULE config=$TRAINING_CONFIG"
echo "nodes=$NNODES gpus_per_node=$GPUS_PER_NODE"
echo "master=$MASTER_ADDR:$MASTER_PORT run_dir=$TRAINING_RUN_DIR"

# Launch one torchrun parent per node. SLURM_PROCID is the zero-based node rank
# because this step requests exactly one task per node. Some clusters require a
# different PMI/srun pattern; follow local administrator guidance.
srun \
  --nodes="$NNODES" \
  --ntasks="$NNODES" \
  --ntasks-per-node=1 \
  --kill-on-bad-exit=1 \
  bash -c '
    set -euo pipefail
    NODE_RANK="${SLURM_PROCID:?Slurm did not provide a process rank}"
    exec "$PYTHON_BIN" -m torch.distributed.run \
      --nnodes="$NNODES" \
      --nproc_per_node="$GPUS_PER_NODE" \
      --node_rank="$NODE_RANK" \
      --master_addr="$MASTER_ADDR" \
      --master_port="$MASTER_PORT" \
      -m "$TRAINING_MODULE" \
      --config "$TRAINING_CONFIG" \
      "$@"
  ' bash "$@"
