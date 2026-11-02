"""Fixed pilot layouts, applied to fresh configs before scene construction.

Standard library only: static layout checks do not require Isaac Sim or torch.
One ID per environment instance (all clones share the layout). Never mutate a
live scene with this module. To switch IDs, construct a fresh config/environment.
"""

import itertools
import json
import math
from pathlib import Path

MANIFEST = Path(__file__).with_name("scenario_pilots.json")
BATCH_MANIFEST = Path(__file__).resolve().parents[8] / "scenarios/make_toast_v2_batch01/manifest.json"
OBJECTS = ("toaster", "plate", "bread_shelf")


def load_manifest(path=None):
    data = json.loads(Path(path or MANIFEST).read_text())
    if data["schema_version"] not in (1, 2):
        raise ValueError("Unsupported scenario schema")
    ids = [entry["id"] for entry in data["scenarios"]]
    if len(set(ids)) != len(ids):
        raise ValueError("Duplicate scenario IDs")
    return data


def get_scenario(scenario_id, manifest_path=None):
    paths = [Path(manifest_path)] if manifest_path else [MANIFEST, BATCH_MANIFEST]
    for path in paths:
        if not path.is_file():
            if manifest_path:
                raise FileNotFoundError(path)
            continue
        data = load_manifest(path)
        for entry in data["scenarios"]:
            if entry["id"] == scenario_id:
                return data, entry
    raise ValueError(f"Unknown scenario ID {scenario_id!r}")


def rotate_z(point, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    x, y, z = point
    return (c * x - s * y, s * x + c * y, z)


def yaw_quaternion(quaternion, yaw):
    """Left multiply by world/environment Z yaw, preserving authored norm."""
    w, x, y, z = quaternion
    c, s = math.cos(yaw / 2), math.sin(yaw / 2)
    return (c * w - s * z, c * x - s * y, c * y + s * x, c * z + s * w)


def transform_pose(position, quaternion, pivot, translation, yaw):
    delta = rotate_z([p - c for p, c in zip(position, pivot)], yaw)
    return (tuple(c + d + t for c, d, t in zip(pivot, delta, translation)),
            yaw_quaternion(quaternion, yaw))


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
not applied twice. Baseline is an exact no-op on every initial state.
"""
    data, scenario = get_scenario(scenario_id, getattr(cfg, "scenario_manifest", None))
    if not getattr(cfg, "serve_to_plate", False):
        raise ValueError("Pilot scenarios require the v2 serving configuration")
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
    if data["schema_version"] == 2:
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
    old_shelf = tuple(cfg.scene.bread_shelf.init_state.pos)
    for name, change in scenario["changes"].items():
        state = getattr(cfg.scene, name).init_state
        state.pos, state.rot = transform_pose(state.pos, state.rot, state.pos,
                                              change["translation"], change["yaw"])
    if "bread_shelf" in scenario["changes"]:
        change = scenario["changes"]["bread_shelf"]
        for i in range(4):
            state = getattr(cfg.scene, f"bread_{i}").init_state
            state.pos, state.rot = transform_pose(state.pos, state.rot, old_shelf,
                                                  change["translation"], change["yaw"])
    cfg._toast_scenario_applied = scenario_id
    cfg._toast_scenario_signature = signature


def static_layout_report(scenario_id, manifest_path=None):
    """Conservative transformed measured AABBs, NOT collision/reachability proof."""
    data, scenario = get_scenario(scenario_id, manifest_path)
    if data["schema_version"] == 2:
        return {"scenario_id": scenario_id, "manifest_schema": 2,
                "candidate_status": scenario["status"], "fixed_poses": scenario["poses"],
                "static_checks": scenario["checks"], "reachability": "NOT_PROVEN",
                "physics": "NOT_TESTED", "reset_runtime": "NOT_TESTED",
                "note": "A layout/reset PASS does not promote this candidate to task-valid."}
    bounds = {}
    for name in OBJECTS:
        change = scenario["changes"].get(name, {"translation": (0, 0, 0), "yaw": 0})
        pivot = data["baseline"][name]["position_m"]
        lo, hi = data["measured_bounds"][name]
        points = [transform_pose(p, (1, 0, 0, 0), pivot, change["translation"], change["yaw"])[0]
                  for p in itertools.product(*zip(lo, hi))]
        bounds[name] = [[min(p[i] for p in points) for i in range(3)],
                        [max(p[i] for p in points) for i in range(3)]]
    table_lo, table_hi = data["table_envelope"]
    margins = {name: min(*(lo[i] - table_lo[i] for i in (0, 1)),
                         *(table_hi[i] - hi[i] for i in (0, 1)))
               for name, (lo, hi) in bounds.items()}
    pairs = {}
    for a, b in itertools.combinations(OBJECTS, 2):
        al, ah = bounds[a]
        bl, bh = bounds[b]
        gaps = [max(bl[i] - ah[i], al[i] - bh[i], 0) for i in (0, 1)]
        pairs[f"{a}/{b}"] = {"aabb_xy_overlap": not any(gaps),
                              "aabb_xy_distance_m": math.hypot(*gaps)}
    return {"scenario_id": scenario_id, "bounds": bounds, "table_envelope_margin_m": margins,
            "pairs": pairs, "reachability": "NOT_TESTED", "physics": "NOT_TESTED",
            "tabletop_containment": "NOT_VERIFIED", "reset_runtime": "NOT_TESTED",
            "note": "AABB overlap is inconclusive; non-overlap only bounds measured visual geometry."}
