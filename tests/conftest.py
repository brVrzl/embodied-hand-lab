"""Shared offline test-environment setup."""

from __future__ import annotations

from collections.abc import Iterator
import os
from pathlib import Path
import subprocess
import sys

import pytest


@pytest.fixture(scope="session", autouse=True)
def _python_environment_tools_on_path() -> Iterator[None]:
    """Expose tools installed beside the interpreter to subprocess fixtures.

    The documented suite is intentionally runnable as ``.venv/bin/python``
    without first activating the virtual environment.  CMake-backed tests
    therefore need the interpreter's sibling executables on ``PATH``.
    """

    original_path = os.environ.get("PATH")
    # Do not resolve the virtual-environment interpreter symlink: its sibling
    # scripts (cmake, ctest, ninja) live in the lexical ``.venv/bin`` directory.
    executable_directory = str(Path(sys.executable).parent.absolute())
    path_entries = (original_path or "").split(os.pathsep)
    if executable_directory not in path_entries:
        os.environ["PATH"] = os.pathsep.join(
            entry for entry in (executable_directory, original_path) if entry
        )
    try:
        yield
    finally:
        if original_path is None:
            os.environ.pop("PATH", None)
        else:
            os.environ["PATH"] = original_path


@pytest.fixture(scope="session")
def teleop_shaping_library(
    _python_environment_tools_on_path: None,
) -> Path:
    """Build and return the native shaping reference used by Python tests.

    Keeping this as an explicit shared fixture makes the suite independent of
    test collection order and of any pre-existing ``build/`` directory.
    """

    root = Path(__file__).resolve().parents[1]
    source = root / "native/teleop_shaping"
    build = root / "build/teleop_shaping"
    from teleop_rearchitecture.cpp_shaping import default_cpp_library

    library = default_cpp_library(root)
    subprocess.run(
        [
            "cmake",
            "-S",
            str(source),
            "-B",
            str(build),
            "-DCMAKE_BUILD_TYPE=Release",
        ],
        check=True,
    )
    subprocess.run(["cmake", "--build", str(build), "-j2"], check=True)
    if not library.is_file():
        raise FileNotFoundError(f"native shaping build did not produce {library}")
    return library
