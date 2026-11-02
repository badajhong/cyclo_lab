"""Check the two-arm/lift action contract and auxiliary Mimic frame in real simulation."""

import argparse
import json
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--task", default="Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v0",
                    choices=("Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v0",
                             "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v1",
                             "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2"))
parser.add_argument("--num_envs", type=int, default=2)
parser.add_argument("--check_bc", action="store_true", help="Check one image BC update and 15D policy inference.")
parser.add_argument("--dataset", type=Path, help="Use an actual successful demo for the BC update.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym
import torch
import cyclo_lab  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

try:
    cfg = parse_env_cfg(args.task, device=args.device, num_envs=args.num_envs)
    cfg.terminations = None
    env = gym.make(args.task, cfg=cfg).unwrapped
    env.reset()
    from isaacsim.core.utils.stage import get_current_stage
    from pxr import Usd, UsdPhysics, UsdShade
    cooked = 0
    for prim in Usd.PrimRange(get_current_stage().GetPrimAtPath("/World/envs/env_0/Toaster")):
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            continue
        approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        prefix = {"convexDecomposition": "physxConvexDecompositionCollision",
                  "convexHull": "physxConvexHullCollision"}.get(approximation)
        if prefix:
            assert abs(prim.GetAttribute(prefix + ":minThickness").Get() - .001) < 1e-7
            cooked += 1
    assert cooked > 0
    print("PASS: toaster convex collision thickness is 1 mm; collision meshes remain enabled", flush=True)
    if getattr(cfg, "serve_to_plate", False):
        toaster = env.scene["toaster"]
        index = toaster.joint_names.index("carriage_joint")
        assert (toaster.data.joint_pos_limits[:, index, 1] - .015).abs().max() < 1e-6
        assert env.get_object_poses()["plate"].shape == (env.num_envs, 4, 4)
        for i in range(env.num_envs):
            visual = get_current_stage().GetPrimAtPath(f"/World/envs/env_{i}/Plate/visual/model")
            material, _ = UsdShade.MaterialBindingAPI(visual).ComputeBoundMaterial()
            shader, _, _ = material.ComputeSurfaceSource()
            color = shader.GetInput("diffuseColor").Get()
            assert max(abs(float(a) - b) for a, b in zip(color, (.025, .075, .14))) < 1e-6
        assert "plate_contact" in env.scene.sensors
        print("PASS: serving plate material/poses, contact sensor, and 15 mm high-lift joint in every clone", flush=True)
    robot = env.scene["robot"]
    mirrored = torch.tensor([1, -1, -1, 1, -1, 1, -1], device=env.device)
    left_ids = [robot.joint_names.index(f"arm_l_joint{i}") for i in range(1, 8)]
    right_ids = [robot.joint_names.index(f"arm_r_joint{i}") for i in range(1, 8)]
    torch.testing.assert_close(robot.data.joint_pos[:, left_ids],
                               robot.data.joint_pos[:, right_ids] * mirrored, atol=2e-5, rtol=0)
    left = env.get_robot_eef_pose("left")[:, :3, 3]
    right = env.get_robot_eef_pose("right")[:, :3, 3]
    torch.testing.assert_close(left, right * torch.tensor([1, -1, 1], device=env.device), atol=2e-5, rtol=0)
    assert (left[:, 2] > 1.43).all() and ((left[:, 1] - right[:, 1]) > .75).all()
    print("PASS: symmetric raised standby joints, wrist height, and hand separation", flush=True)
    action = torch.zeros((env.num_envs, 15), device=env.device)
    action[:, 12:] = 1
    for _ in range(10):
        env.step(action)
    assert env.action_manager.active_terms == ["left_arm", "right_arm", "left_gripper", "right_gripper", "lift"]
    assert env.action_manager.total_action_dim == 15
    for name in ("left_wrist_cam", "right_wrist_cam", "head_cam"):
        assert env.obs_buf["policy"][name].shape == (env.num_envs, 244, 244, 3)
    assert env.obs_buf["policy"]["joint_pos"].shape == (env.num_envs, 15)
    assert env.obs_buf["policy"]["gripper_pos"].shape == (env.num_envs, 2)
    print("PASS: 15D action order, both arms/lift observations, and three cameras", flush=True)
    action[:, :12] = torch.tensor([.01, -.02, .015, .02, -.01, .03] * 2, device=env.device)
    action[:, 12:14] = torch.tensor([1, -1], device=env.device)
    action[:, 14] = .8
    targets = env.action_to_target_eef_pose(action)
    grippers = env.actions_to_gripper_actions(action)
    for i in range(env.num_envs):
        restored = env.target_eef_pose_to_action(
            {name: value[i] for name, value in targets.items()},
            {name: value[i] for name, value in grippers.items()}, env_id=i,
        )
        torch.testing.assert_close(restored, action[i], atol=3e-5, rtol=5e-4)
    print("PASS: full two-arm + lift action/pose round trip across cloned environments", flush=True)
    robot = env.scene["robot"]
    index = robot.joint_names.index("lift_joint")
    baseline = robot.data.joint_pos[:, index].clone()
    action[:, :12] = 0
    action[:, 12:14] = 1
    action[:, 14] = .88  # -0.03 m, within the measured clearance above the table.
    for _ in range(20):
        env.step(action)
    assert (robot.data.joint_pos[:, index] < baseline - .02).all()
    assert (robot.data.joint_pos[:, index] > -.04).all()
    state = env.scene.get_state(is_relative=True)
    state["articulation"]["robot"]["joint_velocity"][:, index] = .02
    env.reset_to(state, None, is_relative=True)
    assert torch.isfinite(robot.data.joint_pos).all()
    torch.testing.assert_close(robot.data.joint_vel, state["articulation"]["robot"]["joint_velocity"])
    assert (robot.data.joint_vel_target == 0).all()
    print("PASS: physical lift movement and state restoration without persistent velocity feedforward", flush=True)
    if args.check_bc:
        from robomimic.algo import algo_factory
        from robomimic.config import config_factory
        import robomimic.utils.obs_utils as obs_utils

        path = gym.spec(args.task).kwargs["robomimic_bc_cfg_entry_point"]
        external = json.loads(Path(path).read_text())
        config = config_factory(external["algo_name"])
        with config.values_unlocked():
            config.update(external)
        obs_utils.initialize_obs_utils_with_config(config)
        observations = {key: obs_utils.process_obs(env.obs_buf["policy"][key][:1].clone(), obs_key=key)
                        for key in config.all_obs_keys}
        model = algo_factory(config.algo_name, config,
                             {key: list(value.shape[1:]) for key, value in observations.items()},
                             ac_dim=15, device=env.device)
        length = config.train.seq_length
        batch = {"obs": {key: value.unsqueeze(1).repeat(1, length, *([1] * (value.ndim - 1)))
                         for key, value in observations.items()},
                 "actions": torch.zeros((1, length, 15), device=env.device)}
        if args.dataset:
            import h5py
            import numpy as np
            with h5py.File(args.dataset, "r") as dataset:
                demo = dataset["data"][next(iter(dataset["data"]))]
                actions = demo["actions"][:]
                assert actions.shape[1] == 15 and len(actions) >= length
                moving = np.flatnonzero(np.linalg.norm(actions[:, :12], axis=-1) > 1e-3)
                start = min(int(moving[0]) if len(moving) else 0, len(actions) - length)
                batch = {
                    "obs": {key: obs_utils.process_obs(
                        torch.as_tensor(demo["obs"][key][start:start + length], device=env.device)[None], obs_key=key,
                    ) for key in config.all_obs_keys},
                    "actions": torch.as_tensor(actions[start:start + length], device=env.device)[None],
                }
            print(f"Using physical demonstration batch: {args.dataset}", flush=True)
        model.set_train()
        info = model.train_on_batch(model.process_batch_for_training(batch), epoch=1)
        assert torch.isfinite(info["losses"]["action_loss"])
        model.set_eval()
        model.reset()
        with torch.no_grad():
            action = model.get_action(observations)
        assert action.shape == (1, 15) and torch.isfinite(action).all()
        print("PASS: bimanual image BC optimizer update and 15D inference", flush=True)
    env.close()
    print("Bimanual Make Toast checks passed.", flush=True)
except Exception:
    traceback.print_exc()
    app.app.post_quit(1)
    raise
finally:
    app.close()
