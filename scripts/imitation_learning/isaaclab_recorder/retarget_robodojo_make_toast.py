"""Calibrate and physically replay RoboDojo two-arm motion on Cyclo Lab's FFW-BG2."""

import argparse
import json
import os
from pathlib import Path
import shutil
import tempfile
import traceback

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--mode", choices=("fit", "check", "generate"), default="check")
parser.add_argument("--source_file", type=Path, default=Path("datasets/robodojo/make_toast/episode_0000000_motion.npz"))
parser.add_argument("--calibration", type=Path, default=Path("datasets/robodojo/make_toast/ffw_bg2_calibration.json"))
parser.add_argument("--dataset_file", type=Path, default=Path("datasets/make_toast_bimanual/dataset.hdf5"))
parser.add_argument("--report", type=Path, default=Path("logs/make_toast_retarget/report.json"))
parser.add_argument("--num_demos", type=int, default=1)
parser.add_argument("--max_attempts", type=int, default=1)
parser.add_argument("--time_scale", type=int, default=2, help="Simulation steps per 25 Hz reference frame.")
parser.add_argument("--frames_limit", type=int, help="Diagnostic check only; never export partial motion.")
parser.add_argument("--snapshots", action="store_true", help="Save diagnostic camera images beside the report.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
if args.num_demos < 1 or args.max_attempts < 1 or args.time_scale < 1:
    parser.error("Demo, attempt, and time-scale counts must be positive")
if args.frames_limit is not None and (args.mode != "check" or args.frames_limit < 1):
    parser.error("--frames_limit is only allowed for diagnostic --mode check")
if args.mode == "generate" and args.dataset_file.exists():
    parser.error(f"Dataset already exists; choose a new --dataset_file: {args.dataset_file}")

from robodojo_motion import (
    fit_calibration, load_calibration, matrix_pose, pose_matrix, read_motion, retarget_wrist_targets,
)

motion = read_motion(args.source_file)
if args.mode == "fit":
    if args.calibration.exists():
        parser.error(f"Calibration already exists; choose a new path to preserve it: {args.calibration}")
    args.calibration.parent.mkdir(parents=True, exist_ok=True)
    args.calibration.write_text(json.dumps(fit_calibration(motion), indent=2) + "\n")
    print(f"Saved candidate calibration: {args.calibration}")
    print("Run --mode check. This fit is not yet a physically validated FFW-BG2 demonstration.")
    raise SystemExit(0)

calibration = load_calibration(args.calibration, motion)
args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym
import numpy as np
from PIL import Image
import torch

import cyclo_lab  # noqa: F401
import isaaclab.utils.math as math_utils
from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.managers import DatasetExportMode
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2 import bimanual_mdp, mdp
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2.assets import BREAD_OFFSETS, BREAD_ROTATION

TASK = "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v0"


class PhysicalReplayError(RuntimeError):
    pass


def configure_scene(cfg):
    cfg.scene.bread_shelf.spawn.scale = (1.0, 1.0, 1.0)
    shelf = pose_matrix(calibration["objects"]["bread_shelf"])
    cfg.scene.bread_shelf.init_state.pos = tuple(shelf[:3, 3])
    cfg.scene.bread_shelf.init_state.rot = tuple(calibration["objects"]["bread_shelf"][3:])
    for i, offset in enumerate(BREAD_OFFSETS):
        local = pose_matrix([*offset, *BREAD_ROTATION])
        pose = matrix_pose(shelf @ local)
        cfg.scene.__dict__[f"bread_{i}"].init_state.pos = tuple(pose[:3])
        cfg.scene.__dict__[f"bread_{i}"].init_state.rot = tuple(pose[3:])
    pose = calibration["objects"]["toaster"]
    cfg.scene.toaster.init_state.pos = tuple(pose[:3])
    cfg.scene.toaster.init_state.rot = tuple(pose[3:])


def main():
    cfg = parse_env_cfg(TASK, device=args.device, num_envs=1)
    if not np.isclose(float(motion["frequency"]), 1.0 / cfg.sim.dt / cfg.decimation):
        raise ValueError("Reference sample rate does not match the task's simulation step rate")
    configure_scene(cfg)
    cfg.env_name = TASK
    cfg.terminations = None
    args.report.parent.mkdir(parents=True, exist_ok=True)
    staging = None
    if args.mode == "generate":
        args.dataset_file.parent.mkdir(parents=True, exist_ok=True)
        staging = Path(tempfile.mkdtemp(prefix=".retarget-", dir=args.dataset_file.parent))
        cfg.recorders = ActionStateRecorderManagerCfg()
        cfg.recorders.dataset_export_dir_path = str(staging)
        cfg.recorders.dataset_filename = "dataset"
        cfg.recorders.dataset_export_mode = DatasetExportMode.EXPORT_SUCCEEDED_ONLY
        cfg.recorders.export_in_record_pre_reset = False
    env = gym.make(TASK, cfg=cfg).unwrapped
    report = {"task": TASK, "source_file": str(args.source_file), "source_sha256": motion["sha256"],
              "calibration": str(args.calibration), "mode": args.mode, "successful_episodes": 0,
              "attempts": [], "calibration_status": calibration["status"]}
    targets = {side: torch.tensor(retarget_wrist_targets(motion, calibration, side), device=env.device,
                                  dtype=torch.float32) for side in ("left", "right")}
    count = len(targets["left"])
    if args.frames_limit:
        count = min(count, args.frames_limit)
    try:
        for attempt in range(args.max_attempts):
            env.reset()
            home = {name: env.get_robot_eef_pose(name)[0].clone() for name in env.EEF_NAMES}
            errors = []
            lift_target = 0.0
            opened = {name: torch.ones(1, device=env.device) for name in env.EEF_NAMES}
            last = {}

            def move(poses, grips, lift, label, snapshot=False):
                nonlocal lift_target, last
                # Rate-limit the body command to 0.03 m/s and stay within the calibrated clearance band.
                lift_target += float(np.clip(lift - lift_target, -.0012, .0012))
                lift_target = float(np.clip(lift_target, calibration["lift_min_m"], calibration["lift_max_m"]))
                body = home["body"].clone()
                body[2, 3] += lift_target
                all_targets = {**poses, "body": body}
                action = env.target_eef_pose_to_action(all_targets, grips)
                for offset in (0, 6):
                    delta = action[offset:offset + 3]
                    distance = torch.linalg.vector_norm(delta) * .3
                    if distance > .008:
                        action[offset:offset + 3] *= .008 / distance
                    action[offset + 3:offset + 6].clamp_(-.12, .12)
                if not torch.isfinite(action).all():
                    raise PhysicalReplayError("Non-finite IK action")
                env.step(action[None])
                last = {name: float(torch.linalg.vector_norm(
                    env.get_robot_eef_pose(name)[0, :3, 3] - poses[name][:3, 3],
                )) for name in ("left", "right")}
                errors.append(max(last.values()))
                if mdp.bread_dropped(env).any():
                    raise PhysicalReplayError(f"Bread dropped during {label}")
                if snapshot and args.snapshots:
                    for camera in ("head_cam", "left_wrist_cam", "right_wrist_cam"):
                        Image.fromarray(env.obs_buf["policy"][camera][0].cpu().numpy()).save(
                            args.report.parent / f"attempt_{attempt}_{label}_{camera}.png"
                        )

            result = {"attempt": attempt, "success": False}
            try:
                for _ in range(30):
                    move({s: home[s] for s in ("left", "right")}, opened, 0.0, "settle")
                start = {s: targets[s][0] for s in ("left", "right")}
                # Move from the raised standby pose to the retargeted source start, with both hands open.
                for i in range(100):
                    fraction = (i + 1) / 100
                    poses = {}
                    for side in ("left", "right"):
                        pose = home[side].clone()
                        pose[:3, 3] = torch.lerp(home[side][:3, 3], start[side][:3, 3], fraction)
                        q = math_utils.quat_slerp(math_utils.quat_from_matrix(home[side][:3, :3]),
                                                 math_utils.quat_from_matrix(start[side][:3, :3]), fraction)
                        pose[:3, :3] = math_utils.matrix_from_quat(q)
                        poses[side] = pose
                    move(poses, opened, -.06, "approach")
                for frame in range(count):
                    poses = {s: targets[s][frame] for s in ("left", "right")}
                    grips = {s: torch.tensor([2.0 * motion[f"action_{s}_ee_joint_states"][frame, 0] - 1.0],
                                             device=env.device) for s in ("left", "right")}
                    grips["body"] = opened["body"]
                    mean_z = float((poses["left"][2, 3] + poses["right"][2, 3]) / 2)
                    lift = mean_z - calibration["lift_reference_wrist_z_m"]
                    for _ in range(args.time_scale):
                        move(poses, grips, lift, f"frame_{frame}", snapshot=frame % 50 == 0 or frame in (60, 70, 80))
                    if frame % 50 == 0 or frame in (60, 70, 80):
                        robot = env.scene["robot"]
                        print(f"REPLAY {frame}/{count}: wrist error {last}, lift={lift_target:.3f} m; "
                              f"bread={[mdp.object_position(env, f'bread_{i}')[0].tolist() for i in range(2)]}; "
                              f"gripper={robot.data.joint_pos[0, [robot.joint_names.index(f'gripper_r_joint{i}') for i in range(1, 5)]].tolist()}", flush=True)
                if args.frames_limit:
                    result["diagnostic_only"] = True
                else:
                    for _ in range(200):
                        move({s: home[s] for s in ("left", "right")}, opened, 0.0, "return")
                    for _ in range(15):
                        move({s: home[s] for s in ("left", "right")}, opened, 0.0, "final", snapshot=True)
                    result["success"] = bool(bimanual_mdp.task_done(env)[0])
                result["bread_in_slots"] = {
                    f"bread_{i}": [bool(mdp.bread_in_slot(env, f"bread_{i}", slot)[0]) for slot in (0, 1)]
                    for i in range(2)
                }
                result["lever_pressed"] = bool(mdp.lever_pressed(env)[0])
            except PhysicalReplayError as error:
                result["failure"] = str(error)
            result["final_wrist_error_m"] = last
            result["max_wrist_error_m"] = max(errors, default=0.0)
            result["p95_wrist_error_m"] = float(np.percentile(errors, 95)) if errors else None
            result["final_lift_m"] = float(bimanual_mdp.lift_position(env)[0, 0])
            report["attempts"].append(result)
            if result["success"]:
                report["successful_episodes"] += 1
                if args.mode == "generate":
                    env.recorder_manager.set_success_to_episodes(None, torch.tensor([[True]], device=env.device))
                    env.recorder_manager.export_episodes()
            args.report.write_text(json.dumps(report, indent=2) + "\n")
            print("RESULT", json.dumps(result), flush=True)
            if report["successful_episodes"] >= args.num_demos:
                break
    finally:
        env.close()
        if staging:
            if report["successful_episodes"]:
                os.replace(staging / "dataset.hdf5", args.dataset_file)
            shutil.rmtree(staging)
    print(f"Physically successful episodes: {report['successful_episodes']}. Report: {args.report}", flush=True)
    if (not report["successful_episodes"] and not args.frames_limit) or any(
        "failure" in result for result in report["attempts"]
    ):
        raise RuntimeError("Physical validation failed; no FFW-BG2 training dataset was published")


try:
    main()
except Exception:
    traceback.print_exc()
    app.app.post_quit(1)
    raise
finally:
    app.close()
