"""Read-only host and repository diagnostics.

The doctor never opens a robot, serial, camera, or headset connection.  It
reports software capabilities and device-path inventory only.
"""

from __future__ import annotations

import glob
import importlib.util
import json
import os
from pathlib import Path
import platform
import resource
import shutil
import socket
import subprocess
import sys
import tempfile
from typing import Any, Sequence

from .config import load_yaml


SAFE_ENVIRONMENT_KEYS = (
    "CUDA_VISIBLE_DEVICES",
    "NCCL_SOCKET_IFNAME",
    "NCCL_IB_DISABLE",
    "NCCL_P2P_DISABLE",
    "TORCH_DISTRIBUTED_DEBUG",
    "MASTER_ADDR",
    "MASTER_PORT",
    "LOCAL_RANK",
    "RANK",
    "WORLD_SIZE",
    "SLURM_JOB_ID",
    "SLURM_NODEID",
    "EMBODIED_LAB_ROOT",
    "EMBODIED_LAB_SOURCE_REVISION",
)
CREDENTIAL_ENVIRONMENT_KEYS = (
    "WANDB_API_KEY",
    "HF_TOKEN",
    "HUGGING_FACE_HUB_TOKEN",
)


def repository_root() -> Path:
    explicit = os.environ.get("EMBODIED_LAB_ROOT")
    if explicit:
        return Path(explicit).expanduser().resolve()
    source_root = Path(__file__).resolve().parents[2]
    if (source_root / "pyproject.toml").is_file():
        return source_root
    current = Path.cwd().resolve()
    return current


def _run(command: Sequence[str], *, timeout: float = 5.0) -> dict[str, Any]:
    executable = shutil.which(command[0])
    if executable is None:
        environment_executable = Path(sys.executable).parent / command[0]
        if environment_executable.is_file() and os.access(
            environment_executable, os.X_OK
        ):
            executable = str(environment_executable)
    if executable is None:
        return {"available": False, "command": list(command)}
    try:
        result = subprocess.run(
            [executable, *command[1:]],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except (OSError, subprocess.TimeoutExpired) as exc:
        return {
            "available": True,
            "command": list(command),
            "ok": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    stdout = result.stdout.strip()
    stderr = result.stderr.strip()
    return {
        "available": True,
        "command": list(command),
        "ok": result.returncode == 0,
        "returncode": result.returncode,
        "stdout": stdout,
        "stderr": stderr,
    }


def _first_line(result: dict[str, Any]) -> str | None:
    text = result.get("stdout") or result.get("stderr")
    if not text:
        return None
    return str(text).splitlines()[0]


def _system_memory_bytes() -> int | None:
    if sys.platform == "linux":
        try:
            for line in Path("/proc/meminfo").read_text().splitlines():
                if line.startswith("MemTotal:"):
                    return int(line.split()[1]) * 1024
        except (OSError, ValueError, IndexError):
            return None
    if sys.platform == "darwin":
        result = _run(("sysctl", "-n", "hw.memsize"))
        try:
            return int(str(result.get("stdout", "")))
        except ValueError:
            return None
    return None


def _disk(path: Path) -> dict[str, Any]:
    try:
        usage = shutil.disk_usage(path)
    except OSError as exc:
        return {"path": str(path), "available": False, "error": str(exc)}
    return {
        "path": str(path),
        "available": True,
        "total_bytes": usage.total,
        "used_bytes": usage.used,
        "free_bytes": usage.free,
        "writable": os.access(path, os.W_OK),
    }


def _networks() -> list[dict[str, Any]]:
    try:
        names = [name for _, name in socket.if_nameindex()]
    except OSError:
        names = []
    interfaces: list[dict[str, Any]] = []
    for name in sorted(names):
        record: dict[str, Any] = {"name": name}
        speed_path = Path("/sys/class/net") / name / "speed"
        if speed_path.is_file():
            try:
                record["reported_speed_mbit_s"] = int(speed_path.read_text().strip())
            except (OSError, ValueError):
                pass
        interfaces.append(record)
    return interfaces


def _ulimits() -> dict[str, Any]:
    names = {
        "open_files": resource.RLIMIT_NOFILE,
        "processes": getattr(resource, "RLIMIT_NPROC", None),
        "locked_memory_bytes": getattr(resource, "RLIMIT_MEMLOCK", None),
        "stack_bytes": resource.RLIMIT_STACK,
    }
    values: dict[str, Any] = {}
    for name, key in names.items():
        if key is None:
            continue
        try:
            soft, hard = resource.getrlimit(key)
        except (OSError, ValueError):
            continue
        values[name] = {"soft": soft, "hard": hard}
    return values


def _python_packages() -> dict[str, Any]:
    packages: dict[str, Any] = {}
    for import_name, label in (
        ("numpy", "numpy"),
        ("yaml", "PyYAML"),
        ("mujoco", "mujoco"),
        ("serial", "pyserial"),
        ("h5py", "h5py"),
        ("cv2", "opencv"),
        ("pyrealsense2", "pyrealsense2"),
    ):
        spec = importlib.util.find_spec(import_name)
        packages[label] = {"importable": spec is not None}
        if spec is not None and import_name in {"numpy", "yaml", "mujoco"}:
            try:
                module = __import__(import_name)
                packages[label]["version"] = str(
                    getattr(module, "__version__", "unknown")
                )
            except Exception as exc:
                packages[label]["import_error"] = (
                    f"{type(exc).__name__}: {exc}"
                )
    return packages


def _torch_report() -> dict[str, Any]:
    if importlib.util.find_spec("torch") is None:
        return {"importable": False}
    try:
        import torch
    except Exception as exc:
        return {
            "importable": False,
            "import_error": f"{type(exc).__name__}: {exc}",
        }
    distributed = getattr(torch, "distributed", None)
    cudnn_version = None
    try:
        cudnn_version = torch.backends.cudnn.version()
    except Exception:
        pass
    nccl_version: object = None
    try:
        nccl_version = torch.cuda.nccl.version()
    except Exception:
        pass
    devices: list[dict[str, Any]] = []
    if torch.cuda.is_available():
        for index in range(torch.cuda.device_count()):
            properties = torch.cuda.get_device_properties(index)
            devices.append(
                {
                    "index": index,
                    "name": properties.name,
                    "total_memory_bytes": properties.total_memory,
                    "compute_capability": [
                        properties.major,
                        properties.minor,
                    ],
                }
            )
    return {
        "importable": True,
        "version": str(torch.__version__),
        "compiled_cuda_version": getattr(torch.version, "cuda", None),
        "cuda_available": bool(torch.cuda.is_available()),
        "cuda_device_count": int(torch.cuda.device_count()),
        "cudnn_version": cudnn_version,
        "nccl_version": nccl_version,
        "distributed_available": bool(
            distributed is not None and distributed.is_available()
        ),
        "gloo_available": bool(
            distributed is not None
            and distributed.is_available()
            and getattr(distributed, "is_gloo_available", lambda: False)()
        ),
        "nccl_available": bool(
            distributed is not None
            and distributed.is_available()
            and getattr(distributed, "is_nccl_available", lambda: False)()
        ),
        "devices": devices,
    }


def _nvidia_report() -> dict[str, Any]:
    query = _run(
        (
            "nvidia-smi",
            "--query-gpu=index,name,memory.total,driver_version",
            "--format=csv,noheader,nounits",
        )
    )
    topology = _run(("nvidia-smi", "topo", "-m"))
    nvcc = _run(("nvcc", "--version"))
    return {
        "driver_and_gpus": query,
        "topology": topology,
        "cuda_toolkit_nvcc": nvcc,
        "cuda_toolkit_version_line": _first_line(nvcc),
    }


def _config_report(root: Path) -> dict[str, Any]:
    errors: list[str] = []
    count = 0
    for path in sorted((root / "configs").rglob("*.yaml")):
        count += 1
        try:
            load_yaml(path)
        except Exception as exc:
            errors.append(f"{path.relative_to(root)}: {type(exc).__name__}: {exc}")
    return {"yaml_count": count, "errors": errors, "valid": not errors}


def collect_doctor_report(root: Path | None = None) -> dict[str, Any]:
    root = repository_root() if root is None else Path(root).resolve()
    data_path = root / "data"
    models_path = root / "models"
    shm_path = Path("/dev/shm")
    infiniband_path = Path("/sys/class/infiniband")
    config = _config_report(root)
    packages = _python_packages()
    required_paths = {
        "pyproject": (root / "pyproject.toml").is_file(),
        "default_sim_config": (
            root / "configs/sim/quest_hts_jaka_mini2_offline.yaml"
        ).is_file(),
        "default_mjcf": (
            root / "assets/jaka_rh56_visual_coacd.xml"
        ).is_file(),
        "data_directory": data_path.is_dir(),
        "models_directory": models_path.is_dir(),
    }
    problems: list[str] = []
    if not required_paths["pyproject"]:
        problems.append("pyproject.toml is missing from the selected repository root")
    if not config["valid"]:
        problems.append("one or more YAML configurations cannot be parsed")
    for package in ("numpy", "PyYAML"):
        if not packages[package]["importable"]:
            problems.append(f"required Python package is unavailable: {package}")
    for name in ("default_sim_config", "default_mjcf"):
        if not required_paths[name]:
            problems.append(f"required repository path is missing: {name}")

    report: dict[str, Any] = {
        "schema_version": "embodied_lab.doctor.v1",
        "status": "ready_offline" if not problems else "not_ready",
        "safety": {
            "mode": "read_only_inventory",
            "device_connections_attempted": False,
            "robot_commands_sent": False,
        },
        "repository": {
            "root": str(root),
            "project_git_metadata_present": (root / ".git").exists(),
            "source_revision_override": os.environ.get(
                "EMBODIED_LAB_SOURCE_REVISION"
            ),
            "required_paths": required_paths,
            "configurations": config,
        },
        "host": {
            "os": platform.system(),
            "os_release": platform.release(),
            "platform": platform.platform(),
            "architecture": platform.machine(),
            "python": platform.python_version(),
            "python_executable": sys.executable,
            "cpu_count": os.cpu_count(),
            "memory_total_bytes": _system_memory_bytes(),
            "hostname": socket.gethostname(),
            "ulimits": _ulimits(),
        },
        "storage": {
            "repository": _disk(root),
            "data": _disk(data_path if data_path.exists() else root),
            "models": _disk(models_path if models_path.exists() else root),
            "temporary": _disk(Path(tempfile.gettempdir())),
            "shared_memory": _disk(shm_path)
            if shm_path.exists()
            else {"path": str(shm_path), "available": False},
        },
        "network": {
            "interfaces": _networks(),
            "infiniband_devices": sorted(
                item.name for item in infiniband_path.iterdir()
            )
            if infiniband_path.is_dir()
            else [],
            "bandwidth_probe_performed": False,
        },
        "nvidia": _nvidia_report(),
        "pytorch": _torch_report(),
        "python_packages": packages,
        "tools": {
            name: _run(command)
            for name, command in (
                ("cmake", ("cmake", "--version")),
                ("docker", ("docker", "--version")),
                ("apptainer", ("apptainer", "--version")),
                ("singularity", ("singularity", "--version")),
                ("slurm", ("sinfo", "--version")),
                ("lspci", ("lspci", "--version")),
            )
        },
        "device_path_inventory": {
            "serial": sorted(
                glob.glob("/dev/ttyUSB*")
                + glob.glob("/dev/ttyACM*")
                + glob.glob("/dev/serial/by-id/*")
            ),
            "video": sorted(glob.glob("/dev/video*")),
            "live_probe_performed": False,
        },
        "environment": {
            "values": {
                key: os.environ[key]
                for key in SAFE_ENVIRONMENT_KEYS
                if key in os.environ
            },
            "credential_presence": {
                key: key in os.environ for key in CREDENTIAL_ENVIRONMENT_KEYS
            },
        },
        "problems": problems,
    }
    return report


def write_report(path: Path, payload: dict[str, Any]) -> None:
    path = path.expanduser().resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f".{path.name}.", suffix=".tmp", dir=path.parent
    )
    temporary_path = Path(temporary_name)
    try:
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            json.dump(payload, stream, indent=2, sort_keys=True)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
    except BaseException:
        temporary_path.unlink(missing_ok=True)
        raise


def render_summary(report: dict[str, Any]) -> str:
    host = report["host"]
    pytorch = report["pytorch"]
    nvidia = report["nvidia"]["driver_and_gpus"]
    gpu_summary = (
        nvidia.get("stdout")
        if nvidia.get("ok")
        else "no usable nvidia-smi"
    )
    lines = [
        f"Embodied Lab doctor: {report['status']}",
        (
            f"Host: {host['os']} {host['os_release']} {host['architecture']}; "
            f"Python {host['python']}; CPUs {host['cpu_count']}"
        ),
        f"Repository: {report['repository']['root']}",
        (
            "Project Git metadata: "
            + (
                "present"
                if report["repository"]["project_git_metadata_present"]
                else "absent (source bundle provenance)"
            )
        ),
        f"NVIDIA: {gpu_summary}",
        (
            "PyTorch: "
            + (
                f"{pytorch.get('version')} "
                f"(CUDA available={pytorch.get('cuda_available')})"
                if pytorch.get("importable")
                else "not installed"
            )
        ),
        (
            "Config YAML: "
            f"{report['repository']['configurations']['yaml_count']} parsed, "
            f"{len(report['repository']['configurations']['errors'])} errors"
        ),
        "Hardware connectivity: not attempted (read-only inventory only)",
    ]
    if report["problems"]:
        lines.append("Blocking offline problems:")
        lines.extend(f"  - {item}" for item in report["problems"])
    return "\n".join(lines)
