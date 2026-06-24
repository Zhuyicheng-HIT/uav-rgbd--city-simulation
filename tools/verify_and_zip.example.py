#!/usr/bin/env python3
from __future__ import annotations

import hashlib
import subprocess
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
EXCLUDE_DIRS = {".git", "build", "install", "log", "__pycache__", ".venv", "venv"}
EXCLUDE_SUFFIXES = {
    ".db3",
    ".bag",
    ".mcap",
    ".mp4",
    ".avi",
    ".mov",
    ".mkv",
    ".engine",
    ".onnx",
}
INCLUDE_LARGE_FILES = {
    "payload/vision_avoid/irreality.pt",
    "payload/vision_avoid/irreality.engine",
    "payload/vision_sim_ws/models/irreality.pt",
}


def iter_files():
    for path in sorted(ROOT.rglob("*")):
        if not path.is_file():
            continue
        rel = path.relative_to(ROOT)
        if any(part in EXCLUDE_DIRS for part in rel.parts):
            continue
        rel_posix = rel.as_posix()
        if path.suffix.lower() in EXCLUDE_SUFFIXES and rel_posix not in INCLUDE_LARGE_FILES:
            continue
        if rel_posix == "SHA256SUMS.txt":
            continue
        yield path


def main() -> int:
    sums = []
    total = 0
    for path in iter_files():
        data = path.read_bytes()
        total += len(data)
        digest = hashlib.sha256(data).hexdigest()
        sums.append(f"{digest}  {path.relative_to(ROOT).as_posix()}")

    (ROOT / "SHA256SUMS.txt").write_text("\n".join(sums) + "\n", encoding="utf-8")
    print(f"files: {len(sums)}")
    print(f"selected size: {total / 1024 / 1024:.2f} MiB")
    print("wrote SHA256SUMS.txt")

    zip_name = ROOT.name + ".zip"
    subprocess.run(
        ["zip", "-r", zip_name, ".", "-x", "*/.git/*", "*/build/*", "*/install/*", "*/log/*"],
        cwd=ROOT,
        check=True,
    )
    print(f"wrote {zip_name}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
