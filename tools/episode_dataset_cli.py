#!/usr/bin/env python3
"""Compatibility entry point for :mod:`episode_dataset.cli`."""

from __future__ import annotations

from episode_dataset.cli import main


if __name__ == "__main__":
    raise SystemExit(main())
