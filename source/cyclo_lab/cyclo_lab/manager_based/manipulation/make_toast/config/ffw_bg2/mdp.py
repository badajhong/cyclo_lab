"""Reset, observations, and geometric task predicates for Make Toast."""

import math

import torch

import isaaclab.utils.math as math_utils

from .assets import (
    BREAD_OFFSETS, BREAD_ROTATION, LEVER_JOINT, LEVER_PRESSED_RATIO,
    SHELF_POSITION, SLOT_X_BOUNDS, SLOT_Y_BOUNDS, SLOT_Z_BOUNDS, TOASTER_POSITION,
)


def object_position(env, name: str) -> torch.Tensor:
    return env.scene[name].data.root_pos_w - env.scene.env_origins


def object_orientation(env, name: str) -> torch.Tensor:
    return env.scene[name].data.root_quat_w


def lever_position(env) -> torch.Tensor:
    toaster = env.scene["toaster"]
    return toaster.data.joint_pos[:, toaster.joint_names.index(LEVER_JOINT)].unsqueeze(-1)


def lever_pressed(env) -> torch.Tensor:
    toaster = env.scene["toaster"]
    index = toaster.joint_names.index(LEVER_JOINT)
    limits = toaster.data.joint_pos_limits[:, index]
    ratio = (toaster.data.joint_pos[:, index] - limits[:, 0]) / (limits[:, 1] - limits[:, 0])
    return ratio >= LEVER_PRESSED_RATIO


def gripper_open(env) -> torch.Tensor:
    robot = env.scene["robot"]
    return robot.data.joint_pos[:, robot.joint_names.index("gripper_r_joint1")] < 0.15


def bread_in_slot(env, bread: str, slot: int, depth_bounds=None) -> torch.Tensor:
    """Check the slot frame, physical insertion depth, and upright bread orientation."""
    toaster = env.scene["toaster"]
    item = env.scene[bread]
    position, orientation = math_utils.subtract_frame_transforms(
        toaster.data.root_pos_w, toaster.data.root_quat_w,
        item.data.root_pos_w, item.data.root_quat_w,
    )
    x0, x1 = SLOT_X_BOUNDS
    y0, y1 = SLOT_Y_BOUNDS[slot]
    z0, z1 = SLOT_Z_BOUNDS if depth_bounds is None else depth_bounds
    inside = ((position[:, 0] > x0) & (position[:, 0] < x1)
              & (position[:, 1] > y0) & (position[:, 1] < y1)
              & (position[:, 2] > z0) & (position[:, 2] < z1))
    rotation = math_utils.matrix_from_quat(orientation)
    # Bread's local Z is its thin direction; this must align with the slot width (Y).
    aligned = torch.abs(rotation[:, 1, 2]) > math.cos(math.radians(30))
    upright = torch.maximum(torch.abs(rotation[:, 2, 0]), torch.abs(rotation[:, 2, 1])) > math.cos(math.radians(30))
    return inside & aligned & upright


def bread_placed(env, bread: str) -> torch.Tensor:
    item = env.scene[bread]
    stable = torch.linalg.vector_norm(item.data.root_lin_vel_w, dim=-1) < 0.15
    return (bread_in_slot(env, bread, 0) | bread_in_slot(env, bread, 1)) & gripper_open(env) & stable


def bread_grasped(env, bread: str) -> torch.Tensor:
    item = env.scene[bread]
    robot = env.scene["robot"]
    closed = robot.data.joint_pos[:, robot.joint_names.index("gripper_r_joint1")] > 0.2
    eef = env.scene["ee_frame"].data.target_pos_w[:, 0]
    near = torch.linalg.vector_norm(eef - item.data.root_pos_w, dim=-1) < 0.16
    lifted = object_position(env, bread)[:, 2] > item.data.default_root_state[:, 2] + 0.025
    return closed & near & lifted


def robot_at_home(env) -> torch.Tensor:
    robot = env.scene["robot"]
    ids = [robot.joint_names.index(f"arm_r_joint{i}") for i in range(1, 8)]
    error = robot.data.joint_pos[:, ids] - robot.data.default_joint_pos[:, ids]
    return torch.linalg.vector_norm(error, dim=-1) < 0.35


def spare_bread_on_shelf(env) -> torch.Tensor:
    result = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    shelf_pos = torch.tensor(env.cfg.scene.bread_shelf.init_state.pos, device=env.device).expand(env.num_envs, -1)
    for name in ("bread_2", "bread_3"):
        position = object_position(env, name) - shelf_pos
        result &= (torch.abs(position[:, 0]) < 0.085) & (torch.abs(position[:, 1]) < 0.06)
        result &= (position[:, 2] > 0.015) & (position[:, 2] < 0.16)
        rotation = math_utils.matrix_from_quat(env.scene[name].data.root_quat_w)
        result &= torch.maximum(torch.abs(rotation[:, 2, 0]), torch.abs(rotation[:, 2, 1])) > math.cos(math.radians(30))
    return result


def task_done(env) -> torch.Tensor:
    """Two distinct slots occupied, two spare slices on the rack, lever down, arm home."""
    occupied = ((bread_in_slot(env, "bread_0", 0) & bread_in_slot(env, "bread_1", 1))
                | (bread_in_slot(env, "bread_0", 1) & bread_in_slot(env, "bread_1", 0)))
    return (occupied & bread_placed(env, "bread_0") & bread_placed(env, "bread_1")
            & spare_bread_on_shelf(env) & lever_pressed(env) & robot_at_home(env))


def bread_dropped(env) -> torch.Tensor:
    heights = torch.stack([object_position(env, f"bread_{i}")[:, 2] for i in range(4)], dim=-1)
    return (heights < 0.75).any(dim=-1)


def reset_objects(env, env_ids: torch.Tensor):
    """Randomize bread on the rack and the toaster, preserving their relative geometry."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    n = len(env_ids)
    toaster = env.scene["toaster"]
    state = toaster.data.default_root_state[env_ids].clone()
    offset = torch.zeros((n, 3), device=env.device)
    offset[:, :2] = math_utils.sample_uniform(-0.015, 0.015, (n, 2), env.device)
    state[:, :3] = torch.tensor(TOASTER_POSITION, device=env.device) + offset + env.scene.env_origins[env_ids]
    state[:, 7:] = 0.0
    toaster.write_root_state_to_sim(state, env_ids)
    joints = toaster.data.default_joint_pos[env_ids].clone()
    toaster.write_joint_state_to_sim(joints, torch.zeros_like(joints), env_ids=env_ids)
    toaster.set_joint_position_target(joints, env_ids=env_ids)
    toaster.set_joint_velocity_target(torch.zeros_like(joints), env_ids=env_ids)

    # The static shelf stays fixed. A common small Y displacement randomizes slices inside its grooves.
    bread_y = math_utils.sample_uniform(-0.008, 0.008, (n,), env.device)
    for i, offset in enumerate(BREAD_OFFSETS):
        bread = env.scene[f"bread_{i}"]
        state = bread.data.default_root_state[env_ids].clone()
        state[:, :3] = (torch.tensor(SHELF_POSITION, device=env.device)
                       + torch.tensor(offset, device=env.device) + env.scene.env_origins[env_ids])
        state[:, 1] += bread_y
        state[:, 3:7] = torch.tensor(BREAD_ROTATION, device=env.device)
        state[:, 7:] = 0.0
        bread.write_root_state_to_sim(state, env_ids)
