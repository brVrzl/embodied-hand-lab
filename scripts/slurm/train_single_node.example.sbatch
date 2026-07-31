#!/usr/bin/env bash
# Copy this file, replace or remove every angle-bracket placeholder, and
# follow the cluster administrator's Slurm and environment-module guidance.
# Some sites use --gpus-per-node instead of --gres.
#SBATCH --job-name=<JOB_NAME>
#SBATCH --account=<ACCOUNT>
#SBATCH --partition=<PARTITION>
#SBATCH --qos=<QOS>
#SBATCH --nodes=1
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
: "${GPUS_PER_NODE:?Set GPUS_PER_NODE to match the Slurm GPU request}"
if [[ ! "$GPUS_PER_NODE" =~ ^[1-9][0-9]*$ ]]; then
  echo "GPUS_PER_NODE must be a positive integer, got: $GPUS_PER_NODE" >&2
  exit 2
fi

RUN_ID="${RUN_ID:-${SLURM_JOB_NAME:-training}-${SLURM_JOB_ID:-manual}}"
OUTPUT_ROOT="${OUTPUT_ROOT:-artifacts/training}"
export TRAINING_RUN_DIR="$OUTPUT_ROOT/$RUN_ID"
mkdir -p "$TRAINING_RUN_DIR"

echo "Launching module=$TRAINING_MODULE config=$TRAINING_CONFIG"
echo "node_count=1 gpus_per_node=$GPUS_PER_NODE run_dir=$TRAINING_RUN_DIR"

# This uses Python's torchrun module so PYTHON_BIN selects the exact
# environment. The target module must implement the DDP trainer contract.
srun --nodes=1 --ntasks=1 --kill-on-bad-exit=1 \
  "$PYTHON_BIN" -m torch.distributed.run \
  --standalone \
  --nnodes=1 \
  --nproc_per_node="$GPUS_PER_NODE" \
  -m "$TRAINING_MODULE" \
  --config "$TRAINING_CONFIG" \
  "$@"
