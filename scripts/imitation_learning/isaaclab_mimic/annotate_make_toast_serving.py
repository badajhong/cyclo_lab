"""Annotate successful serving recordings from their saved, frame-aligned states.

No simulation is replayed and no images, actions or poses are regenerated. This
keeps the measured grasp trajectory aligned with the original RGB observations.
"""

import argparse
import json
import os
from pathlib import Path
import shutil
import tempfile

import h5py
import numpy as np


def annotate(input_file: Path, output_file: Path):
    if input_file.resolve() == output_file.resolve() or output_file.exists():
        raise FileExistsError(f"Choose a new annotated output path: {output_file}")
    output_file.parent.mkdir(parents=True, exist_ok=True)
    summary = {}
    with tempfile.TemporaryDirectory(prefix=".serving-annotation-", dir=output_file.parent) as directory:
        staged = Path(directory) / "annotated.hdf5"
        shutil.copyfile(input_file, staged)
        with h5py.File(staged, "r+") as dataset:
            if not len(dataset["data"]):
                raise ValueError("No demonstrations to annotate")
            for name, episode in dataset["data"].items():
                if not bool(episode.attrs.get("success", False)):
                    raise ValueError(f"{name} is not a successful source demonstration")
                actions = episode["actions"]
                n = len(actions)
                if actions.shape != (n, 15) or not np.isfinite(actions[:]).all():
                    raise ValueError(f"{name} is not a finite 15-action demonstration")
                info = episode["obs/datagen_info"]
                for frame in ("left", "right", "body"):
                    if info[f"target_eef_pose/{frame}"].shape != (n, 4, 4):
                        raise ValueError(f"Missing aligned {frame} target poses in {name}")
                # Datagen poses are measured before each action; cycle states
                # are recorded after it. Shift the cycle label by one frame.
                phase = np.r_[episode["initial_state/toast_cycle/phase"][()].reshape(-1)[0],
                              episode["states/toast_cycle/phase"][:-1].reshape(-1)]
                first = info["object_pose/bread_0"][:, :3, 3]
                second = info["object_pose/bread_1"][:, :3, 3]
                left = info["eef_pose/left"][:, :3, 3]
                ready = phase == 4
                placed = ready & (first[:, 2] < 1.05)
                standby = np.linalg.norm(left - [.3609426, .3954084, 1.4419125], axis=-1) < .04
                conditions = {
                    "toast_ready": ready,
                    "extract_toast_0": ready & (first[:, 2] > 1.29),
                    "serve_toast_0": placed & standby,
                    "extract_toast_1": placed & (second[:, 2] > 1.29),
                }
                indices = {}
                previous = -1
                group = info.require_group("subtask_term_signals")
                for signal, condition in conditions.items():
                    matches = np.flatnonzero(condition)
                    if not len(matches) or int(matches[0]) <= previous:
                        raise ValueError(f"Missing or out-of-order serving boundary {signal} in {name}")
                    previous = int(matches[0])
                    indices[signal] = previous
                    if signal in group:
                        del group[signal]
                    group.create_dataset(signal, data=np.arange(n) >= previous)
                summary[name] = indices
            dataset.attrs["serving_annotation"] = json.dumps({
                "method": "saved_frame_aligned_physical_states", "boundaries": summary,
            })
        # Publish only after every source episode passes, without overwriting.
        os.link(staged, output_file)
    print(json.dumps({"output": str(output_file), "episodes": summary}, indent=2))


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input_file", type=Path, required=True)
    parser.add_argument("--output_file", type=Path, required=True)
    args = parser.parse_args()
    annotate(args.input_file, args.output_file)
