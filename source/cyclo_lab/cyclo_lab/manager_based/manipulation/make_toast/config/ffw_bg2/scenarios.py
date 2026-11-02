"""Fixed scenario layouts, applied to fresh configs before scene construction.

Standard library only: static layout checks do not require Isaac Sim or torch.
One ID per environment instance (all clones share the layout). Never mutate a
live scene with this module. To switch IDs, construct a fresh config/environment.
"""

import json
import math
from pathlib import Path

MANIFEST = Path(__file__).resolve().parents[8] / "scenario_delivery/manifest.json"
OBJECTS = ("toaster", "plate", "bread_shelf")


def load_manifest(path=None):
    data = json.loads(Path(path or MANIFEST).read_text())
    if data["schema_version"] != 2:
        raise ValueError("Unsupported scenario schema")
    ids = [entry["id"] for entry in data["scenarios"]]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate scenario IDs")
    return data


def get_scenario(scenario_id, manifest_path=None):
    data = load_manifest(manifest_path)
    for entry in data["scenarios"]:
        if entry["id"] == scenario_id:
            return data, entry
    raise ValueError(f"Unknown scenario ID {scenario_id!r}")


def _qmul(a, b):
    w, x, y, z = a
    v, i, j, k = b
    return (w*v-x*i-y*j-z*k, w*i+x*v+y*k-z*j,
            w*j-x*k+y*v+z*i, w*k+x*j-y*i+z*v)


def _normalize(q):
    length = math.sqrt(sum(x*x for x in q))
    return tuple(x / length for x in q)


def apply_scenario(cfg, scenario_id):
    """Update init states only; existing reset then restores these defaults.

Bread already contains the rack scale/offset transformation from the original
cfg. Transform all four slices rigidly about the old shelf pivot, so scale is
not applied twice. Stored final poses are validated before any mutation.
"""
    data, scenario = get_scenario(scenario_id, getattr(cfg, "scenario_manifest", None))
    if not getattr(cfg, "serve_to_plate", False):
        raise ValueError("Fixed scenarios require the v2 serving configuration")
    applied = getattr(cfg, "_toast_scenario_applied", None)
    signature = json.dumps(scenario, sort_keys=True)
    if applied is not None:
        if applied != scenario_id or getattr(cfg, "_toast_scenario_signature", None) != signature:
            raise ValueError("Switch IDs using a fresh cfg/environment")
        return
    # Fail before mutation if the original calibration has drifted.
    for name in OBJECTS:
        asset = getattr(cfg.scene, name)
        reference = data["baseline"][name]
        for actual, expected in ((asset.init_state.pos, reference["position_m"]),
                                 (asset.init_state.rot, reference["quaternion_wxyz"])):
            if any(abs(a - b) > 1e-6 for a, b in zip(actual, expected)):
                raise ValueError(f"{name} baseline differs from the measured pilot calibration")
        if name in data["scales"] and tuple(asset.spawn.scale) != tuple(data["scales"][name]):
            raise ValueError(f"{name} scale differs from measured geometry")
    # Store and apply final poses, not seeds or recomputed sampling deltas.
    expected = set(OBJECTS) | {f"bread_{i}" for i in range(4)}
    if set(scenario["poses"]) != expected:
        raise ValueError("Fixed scenario must contain three objects and four slices")
    for name, pose in scenario["poses"].items():
        p, q = pose["position_m"], pose["quaternion_wxyz"]
        if len(p) != 3 or len(q) != 4 or not all(math.isfinite(x) for x in (*p, *q)):
            raise ValueError(f"Invalid pose: {name}")
        if abs(sum(x*x for x in q)-1) > 1e-5:
            raise ValueError(f"Non-unit quaternion: {name}")
        asset = getattr(cfg.scene, name)
        if tuple(pose["scale"]) != tuple(asset.spawn.scale or (1, 1, 1)):
            raise ValueError(f"Scenario may not change {name} scale")
        if abs(p[2] - asset.init_state.pos[2]) > 1e-6:
            raise ValueError(f"Scenario may not change {name} support height")
    # Reject malformed fixed manifests that break rack-relative bread poses.
    old = cfg.scene.bread_shelf.init_state
    shelf = scenario["poses"]["bread_shelf"]
    oq = _normalize(old.rot)
    delta = _qmul(_normalize(shelf["quaternion_wxyz"]), (oq[0], -oq[1], -oq[2], -oq[3]))
    inverse = (delta[0], -delta[1], -delta[2], -delta[3])
    for i in range(4):
        original = getattr(cfg.scene, f"bread_{i}").init_state
        offset = tuple(a-b for a, b in zip(original.pos, old.pos))
        moved = _qmul(_qmul(delta, (0, *offset)), inverse)[1:]
        expected_pos = tuple(a+b for a, b in zip(shelf["position_m"], moved))
        expected_q = _normalize(_qmul(delta, original.rot))
        actual = scenario["poses"][f"bread_{i}"]
        if max(abs(a-b) for a, b in zip(expected_pos, actual["position_m"])) > 1e-6:
            raise ValueError(f"bread_{i} relative position changed")
        alignment = abs(sum(a*b for a, b in zip(expected_q, _normalize(actual["quaternion_wxyz"]))))
        if abs(alignment-1) > 1e-6:
            raise ValueError(f"bread_{i} relative rotation changed")
    for name, pose in scenario["poses"].items():
        state = getattr(cfg.scene, name).init_state
        state.pos, state.rot = tuple(pose["position_m"]), tuple(pose["quaternion_wxyz"])
    cfg._toast_scenario_applied = scenario_id
    cfg._toast_scenario_signature = signature
    return
