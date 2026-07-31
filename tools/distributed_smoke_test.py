#!/usr/bin/env python3
"""Compatibility entry point for :mod:`training_infra.cli`."""

from __future__ import annotations

from pathlib import Path
import sys


REPOSITORY_ROOT = Path(__file__).resolve().parents[1]
SOURCE_ROOT = REPOSITORY_ROOT / "src"
if str(SOURCE_ROOT) not in sys.path:
    sys.path.insert(0, str(SOURCE_ROOT))

from training_infra.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
