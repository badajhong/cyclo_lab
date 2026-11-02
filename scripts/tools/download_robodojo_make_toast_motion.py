#!/usr/bin/env python3
"""Extract only two-arm motion arrays from a pinned RoboDojo simulation episode."""

import argparse
import hashlib
import json
import os
from pathlib import Path
import tempfile


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--episode", type=int, default=0)
    parser.add_argument("--output_dir", type=Path, default=Path("datasets/robodojo/make_toast"))
    args = parser.parse_args()
    if args.episode < 0:
        parser.error("--episode must be nonnegative")
    import fsspec
    import h5py
    import numpy as np

    root = Path(__file__).resolve().parents[2]
    manifest = json.loads((root / "source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/asset_manifest.json").read_text())
    relative = f"data/RoboDojo/make_toast/arx_x5/data/episode_{args.episode:07d}.hdf5"
    url = f"https://huggingface.co/datasets/{manifest['repository']}/resolve/{manifest['revision']}/{relative}"
    args.output_dir.mkdir(parents=True, exist_ok=True)
    output = args.output_dir / f"episode_{args.episode:07d}_motion.npz"
    provenance = output.with_suffix(".json")
    if output.exists():
        if not provenance.exists() or json.loads(provenance.read_text()).get("sha256") != hashlib.sha256(output.read_bytes()).hexdigest():
            raise RuntimeError(f"Existing motion/provenance does not verify: {output}")
        print(f"Verified existing motion: {output}")
        return
    arrays = {}
    # HDF5's file-object driver issues range reads. Do not fetch RGB frames or the entire 0.6 GB file.
    with fsspec.open(url, mode="rb", block_size=65536, cache_type="blockcache") as remote:
        with h5py.File(remote, "r") as source:
            for side in ("left", "right"):
                for group in ("state", "action"):
                    for suffix in ("ee_poses", "ee_joint_states"):
                        key = f"{group}/{side}_{suffix}"
                        value = source[key][()]
                        if not np.isfinite(value).all():
                            raise ValueError(f"Non-finite motion: {key}")
                        arrays[key.replace("/", "_")] = value
            arrays["frequency"] = source["additional_info/frequency"][()]
            instruction = source["instruction"][()]
            instruction = instruction.decode() if isinstance(instruction, bytes) else str(instruction)
    lengths = {len(value) for key, value in arrays.items() if key != "frequency"}
    if len(lengths) != 1 or next(iter(lengths)) < 2 or float(arrays["frequency"]) <= 0:
        raise ValueError("Inconsistent or empty source episode")
    fd, temporary = tempfile.mkstemp(suffix=".npz", dir=args.output_dir)
    try:
        with os.fdopen(fd, "wb") as stream:
            np.savez_compressed(stream, **arrays)
        os.replace(temporary, output)
    finally:
        Path(temporary).unlink(missing_ok=True)
    provenance.write_text(json.dumps({
        "schema_version": 1, "repository": manifest["repository"], "revision": manifest["revision"],
        "source_path": relative, "source_url": url, "domain": "simulation", "robot": "arx_x5",
        "instruction": instruction, "frequency": float(arrays["frequency"]), "frames": next(iter(lengths)),
        "pose_convention": "environment-relative xyz + quaternion wxyz; ARX link6 wrist frame",
        "gripper_convention": "1=open, 0=closed",
        "sha256": hashlib.sha256(output.read_bytes()).hexdigest(),
        "license": manifest["license"],
    }, indent=2) + "\n")
    print(f"Saved {next(iter(lengths))} motion frames at {float(arrays['frequency']):g} Hz: {output}")
    print("This is reference motion, not an FFW-BG2 training dataset. Retarget and physically validate it first.")


if __name__ == "__main__":
    main()
