from __future__ import annotations

import re
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EMBEDDED_BASIC_AUTH = re.compile(r"https?://[^/\\s:@]+:[^@\\s/]+@")


def test_runtime_sources_do_not_embed_url_credentials() -> None:
    offenders: list[str] = []
    for directory in ("src", "tools", "scripts", "configs"):
        for path in (ROOT / directory).rglob("*"):
            if path.is_file() and path.suffix in {".py", ".sh", ".yaml", ".yml", ".json"}:
                if EMBEDDED_BASIC_AUTH.search(path.read_text(encoding="utf-8", errors="replace")):
                    offenders.append(str(path.relative_to(ROOT)))
    assert offenders == []
