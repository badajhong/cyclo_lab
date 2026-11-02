"""Generate FFW-BG2 demonstrations from simulator object poses and robot contact."""

import argparse
import json
import os
from pathlib import Path
import shutil
import tempfile
import traceback

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--dataset_file", type=Path, default=Path("datasets/make_toast_bimanual/dataset_v4.hdf5"))
parser.add_argument("--num_demos", type=int, default=1)
parser.add_argument("--max_attempts", type=int, default=3)
parser.add_argument("--motion_speed", type=float, default=2.0,
                    help="Expert translation/rotation speed and inverse dwell time relative to the original expert.")
parser.add_argument("--check_only", action="store_true", help="Check physics without publishing a dataset.")
parser.add_argument("--stop_after", choices=("grasp", "handover"), help="Diagnostic only; requires --check_only.")
parser.add_argument("--report", type=Path, default=Path("logs/make_toast_expert/report.json"))
parser.add_argument("--snapshots", action="store_true")
parser.add_argument("--resume_debug_state", type=Path, help="Resume a saved handover for diagnostics; requires --check_only.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.num_demos < 1 or args.max_attempts < 1:
    parser.error("Demo and attempt counts must be positive")
if not .5 <= args.motion_speed <= 2.0:
    parser.error("--motion_speed must be between 0.5 and 2.0")
if args.stop_after and not args.check_only:
    parser.error("--stop_after requires --check_only; partial demonstrations are never training data")
if args.resume_debug_state and not args.check_only:
    parser.error("Restored debug states cannot be exported as full demonstrations; use --check_only")
if args.dataset_file.exists() and not args.check_only:
    parser.error(f"Dataset exists; choose another output path: {args.dataset_file}")
args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym
from PIL import Image
import torch

import cyclo_lab  # noqa: F401
from isaaclab.managers import DatasetExportMode
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from make_toast_expert import ExpertFailure, SceneExpert
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2.recorders import ToastExpertRecorderCfg

TASK = "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2"


def main():
    cfg = parse_env_cfg(TASK, device=args.device, num_envs=1)
    cfg.env_name = TASK
    cfg.terminations = None
    if args.check_only:
        # Physics runs at the same fixed dt; diagnostic images only need rendering
        # at phase boundaries. Exported training episodes still render every frame.
        cfg.sim.render_interval = 100000
    staging = None
    if not args.check_only:
        args.dataset_file.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix=".expert-", dir=args.dataset_file.parent))
        cfg.recorders = ToastExpertRecorderCfg()
        cfg.recorders.dataset_export_dir_path = str(staging)
        cfg.recorders.dataset_filename = "dataset"
        cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY
        cfg.recorders.export_in_record_pre_reset = False
    args.report.parent.mkdir(parents=True, exist_ok=True)
    report = {"task": TASK, "controller": "simulator_scene_state_expert", "successes": 0, "attempts": [],
              "mode": "diagnostic" if args.check_only else "collection",
              "motion_speed": args.motion_speed, "calibration_version": 4,
              "toast_cycle": {"carriage_stroke_m": .045, "heating_seconds": 5.0,
                              "initial_inserted_fraction": 2 / 3,
                              "popup_extra_height_m": cfg.toast_popup_extra_height},
              "toaster_min_collision_thickness_m": .001,
              "restored_debug_state": str(args.resume_debug_state) if args.resume_debug_state else None,
              "action_order": ["left_wrist_delta_pose_6", "right_wrist_delta_pose_6",
                               "left_gripper_opening", "right_gripper_opening", "body_lift"],
              "rack_scale": list(cfg.scene.bread_shelf.spawn.scale),
              "scene_layout": {name: {"position_m": list(getattr(cfg.scene, name).init_state.pos),
                                      "quaternion_wxyz": list(getattr(cfg.scene, name).init_state.rot)}
                               for name in ("toaster", "bread_shelf", "plate")},
              "plate_scale": list(cfg.scene.plate.spawn.scale), "serving_release_gap_m": .050}
    env = gym.make(TASK, cfg=cfg).unwrapped
    if args.check_only:
        from isaacsim.core.utils.stage import get_current_stage
        from pxr import Usd, UsdGeom
        stage = get_current_stage()
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        report["layout_bounds"] = {}
        for name in ("PackingTable/SM_CratePacking_Table_A1/SM_HeavyDutyPackingTable_C02_01",
                     "BreadShelf", "Toaster", "Plate"):
            prim = stage.GetPrimAtPath(f"/World/envs/env_0/{name}")
            box = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            report["layout_bounds"][name] = [list(box.GetMin()), list(box.GetMax())]
        print("LAYOUT BOUNDS", json.dumps(report["layout_bounds"]), flush=True)
    try:
        for attempt in range(args.max_attempts):
            env.reset(seed=attempt + 1)
            result = {"attempt": attempt, "success": False, "phases": []}
            report["active_attempt"] = result

            def progress(label, metrics):
                print(f"EXPERT {attempt} {label}: {json.dumps(metrics)}", flush=True)
                result["last_phase"] = label
                result["last_metrics"] = metrics
                result["phases"].append({"name": label, **metrics})
                if args.snapshots:
                    if args.check_only:
                        env.sim.render()
                        env.sim.render()
                    for camera in ("head_cam", "left_wrist_cam", "right_wrist_cam"):
                        env.scene[camera].update(0, force_recompute=True)
                        Image.fromarray(env.scene[camera].data.output["rgb"][0, :, :, :3].cpu().numpy()).save(
                            args.report.parent / f"attempt_{attempt}_{label}_{camera}.png"
                        )
                    if label.endswith(("_giver_retreat", "_above_slot", "_serve_standby")) or label in ("lever_preshape_open", "toast_ready"):
                        scene_state = env.scene.get_state(is_relative=True)
                        if env.toast_cycle is not None:
                            scene_state["toast_cycle"] = env.toast_cycle.state()
                        torch.save({"scene": scene_state, "home": expert.home,
                                    "goals": expert.goals, "body_home": expert.body_home,
                                    "opening": expert.opening, "lift": expert.lift,
                                    "milestones": expert.milestones, "steps": expert.steps,
                                    "bread_index": (4 + int(label.split("_")[1]) if label.endswith("_serve_standby")
                                                    else 3 if label == "toast_ready" else 2
                                                    if label == "lever_preshape_open" else int(label.split("_")[1]))},
                                   args.report.parent / f"attempt_{attempt}_{label}.pt")
                args.report.write_text(json.dumps(report, indent=2) + "\n")

            expert = SceneExpert(env, progress=progress, motion_speed=args.motion_speed)
            resume_bread = None
            if args.resume_debug_state:
                saved = torch.load(args.resume_debug_state, map_location=env.device, weights_only=True)
                # A diagnostic may restore an earlier pop-up task's endpoint;
                # only the newly added plate is taken from the calibrated reset.
                if "plate" not in saved["scene"]["rigid_object"]:
                    saved["scene"]["rigid_object"]["plate"] = env.scene.get_state(is_relative=True)["rigid_object"]["plate"]
                env.reset_to(saved["scene"], None, is_relative=True)
                for key in ("home", "goals", "body_home", "opening", "lift", "milestones", "steps"):
                    setattr(expert, key, saved[key])
                resume_bread = saved["bread_index"]
            try:
                result["success"] = expert.run(stop_after=args.stop_after, resume_bread=resume_bread)
            except ExpertFailure as error:
                result["failure"] = str(error)
            result["milestones"] = expert.milestones
            result["steps"] = expert.steps
            result["simulation_seconds"] = expert.steps * env.step_dt
            result["lever_travel_m"] = expert.lever_max - expert.lever_start
            result["peak_shell_contact_n"] = expert.peak_shell_force
            result["peak_lever_contact_n"] = expert.peak_lever_force
            result["peak_plate_contact_n"] = expert.peak_plate_force
            report["attempts"].append(result)
            report.pop("active_attempt", None)
            if result["success"]:
                report["successes"] += 1
                if not args.check_only:
                    env.recorder_manager.set_success_to_episodes(None, torch.tensor([[True]], device=env.device))
                    env.recorder_manager.export_episodes()
            args.report.write_text(json.dumps(report, indent=2) + "\n")
            print("RESULT", json.dumps(result), flush=True)
            if report["successes"] >= args.num_demos or args.stop_after:
                break
    finally:
        env.close()
        if staging:
            if report["successes"]:
                import h5py
                with h5py.File(staging / "dataset.hdf5", "a") as dataset:
                    dataset.attrs["demonstration_controller"] = report["controller"]
                    dataset.attrs["calibration"] = json.dumps({
                        "rack_scale": report["rack_scale"], "control_hz": 25,
                        "calibration_version": report["calibration_version"], "motion_speed": args.motion_speed,
                        "toaster_min_collision_thickness_m": report["toaster_min_collision_thickness_m"],
                        "toast_cycle": report["toast_cycle"],
                        "plate_scale": report["plate_scale"],
                        "serving_release_gap_m": report["serving_release_gap_m"],
                        "scene_layout": report["scene_layout"],
                        "finger_static_friction": 1.0, "finger_dynamic_friction": .8,
                        "object_motion": "robot_and_motorized_carriage_contact_only",
                    })
                # Atomic publication without replacing an existing dataset, including
                # one completed by another collecting process after our startup check.
                os.link(staging / "dataset.hdf5", args.dataset_file)
            shutil.rmtree(staging)
    kind = "physical checks" if args.check_only else "physical demonstrations"
    print(f"Successful {kind}: {report['successes']}. Report: {args.report}", flush=True)
    if not report["successes"] and not (args.stop_after and report["attempts"]
                                       and "failure" not in report["attempts"][-1]):
        raise RuntimeError("No successful demonstration; no training dataset was published")


try:
    main()
except Exception:
    traceback.print_exc()
    # Kit's fast shutdown otherwise replaces Python failures with exit status 0.
    # main() has already closed the environment and finalized/removed its staging file.
    import sys
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)
else:
    app.close()
