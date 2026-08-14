#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPOSITORY_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
OUTPUT=${1:-$REPOSITORY_ROOT/outputs/training/pi05_rh56/environment_summary.json}
OPENPI_REPO=${PI05_OPENPI_REPO:-/home/thor/openpi/repo}

python3 - "$OUTPUT" "$REPOSITORY_ROOT" "$OPENPI_REPO" <<'PY'
import json
import os
import pathlib
import platform
import subprocess
import sys
from datetime import datetime, timezone

output, repository, openpi_repo = map(pathlib.Path, sys.argv[1:])

def command(*args):
    try:
        result = subprocess.run(args, check=False, capture_output=True, text=True, timeout=30)
        return {"returncode": result.returncode, "stdout": result.stdout.strip(), "stderr": result.stderr.strip()}
    except Exception as exc:
        return {"error": repr(exc)}

os_release = {}
for line in pathlib.Path("/etc/os-release").read_text().splitlines():
    if "=" in line:
        key, value = line.split("=", 1)
        os_release[key] = value.strip('"')

payload = {
    "captured_utc": datetime.now(timezone.utc).isoformat(),
    "uname": command("uname", "-a"),
    "architecture": platform.machine(),
    "python": command(sys.executable, "--version"),
    "ubuntu": os_release,
    "l4t_core": command("dpkg-query", "-W", "-f=${Version}", "nvidia-l4t-core"),
    "cuda_and_gpu": command("nvidia-smi"),
    "memory": command("free", "-h"),
    "disk": command("df", "-h", str(repository)),
    "temperatures_power": command("timeout", "3", "tegrastats", "--interval", "1000"),
    "docker": command("docker", "version", "--format", "{{.Server.Version}}"),
    "openpi_repo": {
        "path": str(openpi_repo),
        "head": command("git", "-C", str(openpi_repo), "rev-parse", "HEAD"),
        "status": command("git", "-C", str(openpi_repo), "status", "--short"),
        "diff_stat": command("git", "-C", str(openpi_repo), "diff", "--stat"),
    },
    "openpi_image": command("docker", "image", "inspect", "jaka-openpi:thor-cuda13", "--format", "{{.Id}}"),
    "repository": str(repository),
}
output.parent.mkdir(parents=True, exist_ok=True)
temporary = output.with_suffix(".tmp")
temporary.write_text(json.dumps(payload, indent=2, sort_keys=True) + "\n")
os.replace(temporary, output)
print(output)
PY
