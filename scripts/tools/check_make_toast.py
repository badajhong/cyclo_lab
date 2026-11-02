"""Exercise Make Toast physics, completion conditions, and Mimic action conversions."""

import argparse
from pathlib import Path
import traceback

from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--num_envs", type=int, default=2)
parser.add_argument("--steps", type=int, default=40)
parser.add_argument("--snapshot_dir", type=Path, help="Optionally save the first environment's camera images.")
parser.add_argument("--check_bc", action="store_true", help="Run one BC update and policy inference on simulator observations.")
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym
import torch

import cyclo_lab  # noqa: F401
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
import isaaclab.utils.math as math_utils
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2 import mdp
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2.assets import LEVER_JOINT
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2.scene import TABLE_FRAME, TABLE_GROUP


def main():
    regular = parse_env_cfg("Cyclo-Make-Toast-FFW-BG2-IK-Rel-v0", device=args.device, num_envs=args.num_envs)
    original = parse_env_cfg("Cyclo-PickPlace-FFW-BG2-IK-Rel-v0", device=args.device, num_envs=args.num_envs)
    cfg = parse_env_cfg("Cyclo-Make-Toast-FFW-BG2-Mimic-v0", device=args.device, num_envs=args.num_envs)
    assert original.scene.robot.spawn.usd_path == regular.scene.robot.spawn.usd_path == cfg.scene.robot.spawn.usd_path
    assert original.actions.arm_action.to_dict() == regular.actions.arm_action.to_dict()
    cfg.terminations.time_out = None
    cfg.terminations.success = None
    cfg.terminations.bread_dropping = None
    env = gym.make("Cyclo-Make-Toast-FFW-BG2-Mimic-v0", cfg=cfg).unwrapped
    try:
        env.reset()
        import omni.usd

        stage = omni.usd.get_context().get_stage()
        for index in range(env.num_envs):
            table_path = f"/World/envs/env_{index}/PackingTable"
            group = stage.GetPrimAtPath(f"{table_path}/{TABLE_GROUP}")
            assert [child.GetName() for child in group.GetChildren()] == [TABLE_FRAME]
            assert not stage.GetPrimAtPath(f"{table_path}/container_h20").IsActive()
            assert not stage.GetPrimAtPath(f"/World/envs/env_{index}/Object").IsValid()
        assert original.scene.table.spawn.func is not regular.scene.table.spawn.func
        print("PASS: all cloned tables contain no basket, crates, boxes, or PickPlace object", flush=True)
        action = torch.zeros((env.num_envs, 7), device=env.device)
        action[:, -1] = 1.0
        for _ in range(args.steps):
            env.step(action)
        robot = env.scene["robot"]
        left = robot.data.body_link_pos_w[:, robot.body_names.index("arm_l_link7")] - env.scene.env_origins
        right = robot.data.body_link_pos_w[:, robot.body_names.index("arm_r_link7")] - env.scene.env_origins
        assert (left[:, 2] > 1.25).all() and (right[:, 2] > 1.25).all(), "Both wrists must start above the table"
        torch.testing.assert_close(left[:, [0, 2]], right[:, [0, 2]], atol=0.01, rtol=0)
        torch.testing.assert_close(left[:, 1], -right[:, 1], atol=0.01, rtol=0)
        lever_direction = math_utils.quat_apply(
            env.scene["toaster"].data.root_quat_w,
            torch.tensor([1.0, 0.0, 0.0], device=env.device).expand(env.num_envs, -1),
        )
        assert (lever_direction[:, 0] < -0.99).all(), "Lever must face the robot"
        print("PASS: both arms raised and toaster lever facing the robot", flush=True)
        assert env.action_manager.total_action_dim == 7
        for name in ("bread_0", "bread_1", "bread_2", "bread_3", "toaster"):
            state = env.scene[name].data.root_state_w
            assert torch.isfinite(state).all(), name
            print(f"{name}: {mdp.object_position(env, name).tolist()}", flush=True)
        assert not mdp.bread_dropped(env).any(), "Bread fell through the rack/table"
        assert mdp.spare_bread_on_shelf(env).all(), "Spare bread did not remain upright on the rack"
        assert not mdp.task_done(env).any(), "Task succeeds before bread insertion"
        for name in ("right_wrist_cam", "head_cam"):
            image = env.obs_buf["policy"][name]
            assert image.shape == (env.num_envs, 244, 244, 3), (name, image.shape)
            assert image.max() > 0, f"Blank camera: {name}"
            if args.snapshot_dir:
                from PIL import Image

                args.snapshot_dir.mkdir(parents=True, exist_ok=True)
                Image.fromarray(image[0].cpu().numpy()).save(args.snapshot_dir / f"{name}.png")
        print("PASS: physics, seven-dimensional action, cameras, negative success", flush=True)

        action[:, :6] = torch.tensor([0.025, -0.015, 0.02, 0.03, -0.02, 0.01], device=env.device)
        targets = env.action_to_target_eef_pose(action)
        for i in range(env.num_envs):
            reconstructed = env.target_eef_pose_to_action(
                {env.EEF_NAME: targets[env.EEF_NAME][i]}, {env.EEF_NAME: action[i, -1:]}, env_id=i,
            )
            torch.testing.assert_close(reconstructed, action[i], atol=2e-5, rtol=2e-4)
        assert set(env.get_object_poses()) == {"bread_0", "bread_1", "bread_2", "bread_3", "toaster"}
        assert len(env.cfg.subtask_configs[env.EEF_NAME]) == 6
        assert len(env.get_subtask_term_signals()) == 5
        print("PASS: Mimic objects, subtasks, scaled IK action round trip", flush=True)

        toaster = env.scene["toaster"]
        rotation = torch.tensor([0.70710678, 0.70710678, 0.0, 0.0], device=env.device).repeat(env.num_envs, 1)

        def place(bread_name, slot):
            bread = env.scene[bread_name]
            pose = bread.data.default_root_state.clone()
            point = torch.tensor([-0.003, -0.026 if slot == 0 else 0.0265, 0.0], device=env.device)
            pose[:, :3] = toaster.data.root_pos_w + math_utils.quat_apply(
                toaster.data.root_quat_w, point.expand(env.num_envs, -1),
            )
            pose[:, 3:7] = math_utils.quat_mul(toaster.data.root_quat_w, rotation)
            pose[:, 7:] = 0
            bread.write_root_state_to_sim(pose)

        place("bread_0", 0)
        place("bread_1", 1)
        robot = env.scene["robot"]
        robot.write_joint_state_to_sim(robot.data.default_joint_pos, torch.zeros_like(robot.data.default_joint_pos))
        lever_id = toaster.joint_names.index(LEVER_JOINT)
        joints = toaster.data.default_joint_pos.clone()
        joints[:, lever_id] = toaster.data.joint_pos_limits[:, lever_id, 1]
        toaster.write_joint_state_to_sim(joints, torch.zeros_like(joints))
        assert mdp.task_done(env).all(), "Valid bread/lever/home configuration rejected"
        place("bread_1", 0)
        assert not mdp.task_done(env).any(), "Two slices in one slot accepted"
        place("bread_1", 1)
        joints[:, lever_id] = toaster.data.joint_pos_limits[:, lever_id, 0]
        toaster.write_joint_state_to_sim(joints, torch.zeros_like(joints))
        assert not mdp.task_done(env).any(), "Raised lever accepted"
        print("PASS: positive success; rejects occupied single slot and raised lever", flush=True)
        # Let inserted bread settle against the actual toaster collision meshes.
        action[:, :6] = 0
        for _ in range(30):
            env.step(action)
        for name in ("bread_0", "bread_1"):
            print(f"Inserted {name}: {mdp.object_position(env, name).tolist()}, "
                  f"quaternion={env.scene[name].data.root_quat_w.tolist()}", flush=True)
        assert mdp.bread_placed(env, "bread_0").all(), "First slice cannot settle in its physical slot"
        assert mdp.bread_placed(env, "bread_1").all(), "Second slice cannot settle in its physical slot"
        force = torch.zeros((env.num_envs, 1, 3), device=env.device)
        force[:, :, 2] = -10
        toaster.set_external_force_and_torque(
            force, torch.zeros_like(force), body_ids=[toaster.body_names.index("link_1")], is_global=True,
        )
        for _ in range(30):
            env.step(action)
        toaster.set_external_force_and_torque(torch.zeros_like(force), torch.zeros_like(force),
                                             body_ids=[toaster.body_names.index("link_1")], is_global=True)
        assert mdp.lever_pressed(env).all(), "Downward force cannot depress the physical lever"
        assert mdp.task_done(env).all(), "Physically settled toast and depressed lever must satisfy the task"
        if args.snapshot_dir:
            Image.fromarray(env.obs_buf["policy"]["head_cam"][0].cpu().numpy()).save(
                args.snapshot_dir / "inserted_head_cam.png"
            )
        print("PASS: physical bread insertion and force-driven lever travel", flush=True)
        state = env.scene.get_state(is_relative=True)
        assert "toaster" in state["articulation"]
        assert all(f"bread_{i}" in state["rigid_object"] for i in range(4))
        env.reset_to(state, None, is_relative=True)
        assert mdp.task_done(env).all(), "Recorded scene state cannot restore the task"
        if env.num_envs > 1:
            others = env.scene["bread_0"].data.root_state_w[1:].clone()
            mdp.reset_objects(env, torch.tensor([0], device=env.device))
            torch.testing.assert_close(env.scene["bread_0"].data.root_state_w[1:], others)
        print("PASS: recorder state restoration and isolated environment reset", flush=True)
        env.reset()
        if args.check_bc:
            check_bc(env)
        print("Make Toast checks passed.", flush=True)
    finally:
        env.close()


def check_bc(env):
    """Verify training/inference compatibility; this does not train a task policy."""
    import json

    from robomimic.algo import algo_factory
    from robomimic.config import config_factory
    import robomimic.utils.obs_utils as obs_utils

    path = gym.spec("Cyclo-Make-Toast-FFW-BG2-IK-Rel-v0").kwargs["robomimic_bc_cfg_entry_point"]
    external = json.loads(Path(path).read_text())
    config = config_factory(external["algo_name"])
    with config.values_unlocked():
        config.update(external)
    obs_utils.initialize_obs_utils_with_config(config)
    observations = {key: obs_utils.process_obs(env.obs_buf["policy"][key][:1].clone(), obs_key=key)
                    for key in config.all_obs_keys}
    model = algo_factory(
        config.algo_name, config, {key: list(value.shape[1:]) for key, value in observations.items()},
        ac_dim=7, device=env.device,
    )
    length = config.train.seq_length
    batch = {"obs": {key: value.unsqueeze(1).repeat(1, length, *([1] * (value.ndim - 1)))
                     for key, value in observations.items()},
             "actions": torch.zeros((1, length, 7), device=env.device)}
    model.set_train()
    info = model.train_on_batch(model.process_batch_for_training(batch), epoch=1)
    assert torch.isfinite(info["losses"]["action_loss"])
    model.set_eval()
    model.reset()
    with torch.no_grad():
        action = model.get_action(observations)
    assert action.shape == (1, 7) and torch.isfinite(action).all()
    print("PASS: existing image BC configuration, one optimizer update, and policy inference", flush=True)


try:
    main()
except Exception:
    traceback.print_exc()
    app.app.post_quit(1)
    raise
finally:
    app.close()
