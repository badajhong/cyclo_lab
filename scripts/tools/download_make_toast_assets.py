#!/usr/bin/env python3
"""Download pinned toast-task objects and the gray serving plate; no simulator required."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile
import urllib.request


ROOT = Path(__file__).resolve().parents[2]
MANIFEST = ROOT / "source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/asset_manifest.json"
DEFAULT_DESTINATION = ROOT / "source/cyclo_lab/data/objects/robodojo_make_toast"


def digest(path: Path) -> str:
    result = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            result.update(chunk)
    return result.hexdigest()


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--destination", type=Path, default=DEFAULT_DESTINATION)
    args = parser.parse_args()
    manifest = json.loads(MANIFEST.read_text())
    base = (f"https://huggingface.co/datasets/{manifest['repository']}/resolve/"
            f"{manifest['revision']}")
    for entry in manifest["files"]:
        target = args.destination / entry["path"]
        if target.is_file() and digest(target) == entry["sha256"]:
            print(f"Verified: {target}")
            continue
        target.parent.mkdir(parents=True, exist_ok=True)
        fd, temporary = tempfile.mkstemp(prefix=target.name + ".", suffix=".tmp", dir=target.parent)
        try:
            request = urllib.request.Request(f"{base}/{entry['source_path']}", headers={"User-Agent": "CycloLab"})
            with os.fdopen(fd, "wb") as output, urllib.request.urlopen(request, timeout=120) as response:
                while chunk := response.read(1024 * 1024):
                    output.write(chunk)
            temporary_path = Path(temporary)
            if temporary_path.stat().st_size != entry["size"] or digest(temporary_path) != entry["sha256"]:
                raise RuntimeError(f"Checksum verification failed for {entry['path']}")
            os.replace(temporary, target)
            print(f"Downloaded: {target}")
        finally:
            Path(temporary).unlink(missing_ok=True)
    print(f"Make Toast assets ready: {args.destination.resolve()}")


if __name__ == "__main__":
    main()
