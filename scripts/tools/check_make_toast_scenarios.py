"""Inspect fixed v2 pilot layouts and verify reset, without running the expert.

--static uses only Python's standard library. Runtime output is written to a
new directory; existing output directories are never reused or overwritten.
"""

import argparse
import importlib.util
import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / "source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/config/ffw_bg2/scenarios.py"
spec = importlib.util.spec_from_file_location("toast_scenario_static", MODULE)
scenarios = importlib.util.module_from_spec(spec)
spec.loader.exec_module(scenarios)

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--scenario_id", required=True)
parser.add_argument("--scenario_manifest", type=Path, help="Optional fixed batch manifest; pilot IDs remain available")
parser.add_argument("--static", action="store_true", help="Print measured geometry envelope checks only")
parser.add_argument("--output_dir", type=Path, help="Required for simulation; must not already exist")
parser.add_argument("--steps", type=int, default=50, help="Hold steps between resets; no expert or training")
parser.add_argument("--snapshots", action="store_true")
preliminary, _ = parser.parse_known_args()
if preliminary.static:
    args = parser.parse_args()
    print(json.dumps(scenarios.static_layout_report(args.scenario_id, args.scenario_manifest), indent=2))
    raise SystemExit(0)

from isaaclab.app import AppLauncher

AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.output_dir is None or args.steps < 1:
    parser.error("Simulation requires --output_dir and --steps >= 1")
args.output_dir.mkdir(parents=True, exist_ok=False)
args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym
import torch
import cyclo_lab  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

TASK = "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2"
NAMES = ("toaster", "plate", "bread_0", "bread_1", "bread_2", "bread_3")
report = scenarios.static_layout_report(args.scenario_id, args.scenario_manifest)
report.update(task=TASK, samples=[], runtime_reset="RUNNING", reachability="NOT_TESTED")
env = None


def capture():
    result = {}
    for name in (*NAMES, "robot"):
        asset = env.scene[name]
        state = asset.data.root_state_w.clone()
        state[:, :3] -= env.scene.env_origins
        result[name + "/root"] = state
        if name in ("toaster", "robot"):
            result[name + "/joint_pos"] = asset.data.joint_pos.clone()
            result[name + "/joint_vel"] = asset.data.joint_vel.clone()
    pos, quat = env.scene["bread_shelf"].get_world_poses()
    result["bread_shelf/pose"] = torch.cat((pos - env.scene.env_origins, quat), dim=-1).clone()
    for name, value in env.toast_cycle.state().items():
        result["cycle/" + name] = value.clone()
    return result


def save_sample(label, state):
    report["samples"].append({"label": label, "state": {k: v.cpu().tolist() for k, v in state.items()}})
    if args.snapshots:
        from PIL import Image
        env.sim.render()
        env.sim.render()
        for name in ("head_cam", "left_wrist_cam", "right_wrist_cam"):
            camera = env.scene[name]
            camera.update(0, force_recompute=True)
            Image.fromarray(camera.data.output["rgb"][0, :, :, :3].cpu().numpy()).save(
                args.output_dir / f"{label}_{name}.png")


try:
    cfg = parse_env_cfg(TASK, device=args.device, num_envs=1)
    cfg.scenario_id = args.scenario_id
    cfg.scenario_manifest = str(args.scenario_manifest) if args.scenario_manifest else None
    # The environment applies the ID before scene construction. No event,
    # termination, actuator, collision or expert configuration is overridden.
    env = gym.make(TASK, cfg=cfg).unwrapped
    env.reset(seed=1)
    initial = capture()
    for name in (*NAMES, "bread_shelf"):
        configured = getattr(env.cfg.scene, name).init_state
        actual = initial[name + ("/pose" if name == "bread_shelf" else "/root")][:, :7]
        target = torch.tensor([(*configured.pos, *configured.rot)], device=env.device)
        torch.testing.assert_close(actual[:, :3], target[:, :3], atol=5e-5, rtol=0)
        # Quaternion sign equivalence; no change to authored values.
        alignment = (actual[:, 3:] * target[:, 3:]).sum(-1).abs()
        torch.testing.assert_close(alignment, torch.ones_like(alignment), atol=5e-5, rtol=0)
    save_sample("initial", initial)
    # Measure actual runtime bounds too; static evidence is not a collider test.
    from pxr import Usd, UsdGeom
    from isaacsim.core.utils.stage import get_current_stage
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    stage = get_current_stage()
    report["runtime_bounds"] = {}
    for name in ("BreadShelf", "Toaster", "Plate",
                 "PackingTable/SM_CratePacking_Table_A1/SM_HeavyDutyPackingTable_C02_01"):
        box = cache.ComputeWorldBound(stage.GetPrimAtPath("/World/envs/env_0/" + name)).ComputeAlignedRange()
        report["runtime_bounds"][name] = [list(box.GetMin()), list(box.GetMax())]
    action = torch.zeros((1, 15), device=env.device)
    action[:, 12:] = 1  # Open both grippers and hold lift at zero.
    for retry in range(2):
        for _ in range(args.steps):
            _, _, terminated, truncated, _ = env.step(action)
            if bool((terminated | truncated).any()):
                raise RuntimeError("Unexpected automatic reset during layout hold")
        save_sample(f"settled_{retry}", capture())
        # Diagnostic disturbance proves restoration, not just two identical reads.
        bread = env.scene["bread_0"]
        state = bread.data.root_state_w.clone()
        state[:, 0] += .01
        state[:, 2] += .03
        state[:, 7] = .1
        bread.write_root_state_to_sim(state)
        env.reset(seed=retry + 2)
        restored = capture()
        save_sample(f"reset_{retry}", restored)
        for key in initial:
            if key.endswith(("/root", "/pose")):
                torch.testing.assert_close(restored[key][:, :3], initial[key][:, :3], atol=5e-5, rtol=0)
                dot = (restored[key][:, 3:7] * initial[key][:, 3:7]).sum(-1).abs()
                torch.testing.assert_close(dot, torch.ones_like(dot), atol=5e-5, rtol=0)
                torch.testing.assert_close(restored[key][:, 7:], initial[key][:, 7:], atol=5e-5, rtol=0)
            else:
                torch.testing.assert_close(restored[key], initial[key], atol=5e-5, rtol=0)
    report["runtime_reset"] = "PASS"
    report["reset_runtime"] = "PASS"
    print("PASS: configured initial poses and two resets after disturbance, across seeds")
    print("Reachability, collision-free manipulation and task completion: NOT TESTED")
except Exception as error:
    report["runtime_reset"] = "FAIL"
    report["reset_runtime"] = "FAIL"
    report["error"] = repr(error)
    import traceback
    traceback.print_exc()
finally:
    (args.output_dir / "report.json").write_text(json.dumps(report, indent=2) + "\n")
    if env is not None:
        env.close()
    if report["runtime_reset"] != "PASS":
        # Kit fast shutdown may otherwise replace Python failures with exit 0.
        import os
        import sys
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(1)
    app.close()
