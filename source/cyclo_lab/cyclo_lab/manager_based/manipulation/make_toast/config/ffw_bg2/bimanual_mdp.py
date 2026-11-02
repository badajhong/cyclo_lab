"""Physical completion checks for the two-arm/lift task."""

import torch
import isaaclab.utils.math as math_utils
from . import mdp
from .assets import BIMANUAL_STANDBY_WRIST, SERVING_PLATE_RADIUS
from .bread_geometry import bread_support_vertices


def lift_position(env):
    robot = env.scene["robot"]
    return robot.data.joint_pos[:, robot.joint_names.index("lift_joint")].unsqueeze(-1)


def gripper_positions(env):
    robot = env.scene["robot"]
    return robot.data.joint_pos[:, [robot.joint_names.index(f"gripper_{s}_joint1") for s in ("l", "r")]]


def bread_in_slot(env, bread, slot):
    cycle = getattr(env, "toast_cycle", None)
    if cycle is None:
        return mdp.bread_in_slot(env, bread, slot)
    q = env.scene["toaster"].data.joint_pos[:, cycle.carriage_id]
    return mdp.bread_in_slot(env, bread, slot, depth_bounds=(q + .045, q + .079))


def toast_cycle_state(env):
    cycle = getattr(env, "toast_cycle", None)
    if cycle is None:
        return torch.zeros((env.num_envs, 3), device=env.device)
    return torch.stack((cycle.phase, cycle.heating_time,
                        env.scene["toaster"].data.joint_pos[:, cycle.carriage_id]), dim=-1)


def bread_on_plate(env, name):
    """Both full slices must lie flat within the plate, including the upper slice."""
    plate, bread = env.scene["plate"], env.scene[name]
    pos, quat = math_utils.subtract_frame_transforms(
        plate.data.root_pos_w, plate.data.root_quat_w, bread.data.root_pos_w, bread.data.root_quat_w,
    )
    rotation = math_utils.matrix_from_quat(quat)
    if not hasattr(env, "_toast_bread_support_vertices"):
        env._toast_bread_support_vertices = torch.tensor(bread_support_vertices(), device=env.device)
    corners = env._toast_bread_support_vertices
    points = torch.einsum("nij,vj->nvi", rotation, corners) + pos[:, None]
    contained = torch.linalg.vector_norm(points[:, :, :2], dim=-1).amax(dim=-1) < SERVING_PLATE_RADIUS - .0005
    flat = rotation[:, 2, 2].abs() > .94
    height = (points[:, :, 2].amin(dim=-1) > -.002) & (points[:, :, 2].amax(dim=-1) < .050)
    # Match the task's resting-contact tolerance: PhysX constraint stabilization
    # reports residual gravity velocity even at a stationary supporting contact.
    stable = torch.linalg.vector_norm(bread.data.root_lin_vel_w, dim=-1) < .15
    return contained & flat & height & stable


def task_done(env):
    cycle = getattr(env, "toast_cycle", None)
    serving = getattr(env.cfg, "serve_to_plate", False)
    # A serving episode cannot finish before pop-up. Avoid repeated visual-hull
    # transforms for recorder labels throughout pickup, insertion, and heating.
    if serving and (cycle is None or not (cycle.phase == 4).any()):
        return torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)
    if serving:
        plate = env.scene["plate"]
        upright = math_utils.matrix_from_quat(plate.data.root_quat_w)[:, 2, 2] > .985
        placed = mdp.object_position(env, "plate") - plate.data.default_root_state[:, :3]
        occupied = (bread_on_plate(env, "bread_0") & bread_on_plate(env, "bread_1") & upright
                    & (torch.linalg.vector_norm(placed[:, :2], dim=-1) < .025)
                    & (mdp.object_position(env, "plate")[:, 0] > .419))
    else:
        occupied = ((bread_in_slot(env, "bread_0", 0) & bread_in_slot(env, "bread_1", 1))
                    | (bread_in_slot(env, "bread_0", 1) & bread_in_slot(env, "bread_1", 0)))
    robot = env.scene["robot"]
    standby = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    for side, sign in (("l", 1), ("r", -1)):
        index = robot.body_names.index(f"arm_{side}_link7")
        reference = torch.tensor([BIMANUAL_STANDBY_WRIST[0], sign * BIMANUAL_STANDBY_WRIST[1],
                                  BIMANUAL_STANDBY_WRIST[2]], device=env.device)
        position = robot.data.body_link_pos_w[:, index] - env.scene.env_origins
        standby &= torch.linalg.vector_norm(position - reference, dim=-1) < 0.04
    stable = torch.ones_like(standby)
    for name in ("bread_0", "bread_1"):
        stable &= torch.linalg.vector_norm(env.scene[name].data.root_lin_vel_w, dim=-1) < 0.15
    finished = mdp.lever_pressed(env) if cycle is None else ((cycle.phase == 4) & ~mdp.lever_pressed(env))
    return (occupied & stable & spare_bread_on_shelf(env) & finished
            & (gripper_positions(env) < 0.15).all(dim=-1) & standby & (lift_position(env)[:, 0].abs() < 0.02))


def spare_bread_on_shelf(env):
    shelf = env.cfg.scene.bread_shelf
    position = torch.tensor(shelf.init_state.pos, device=env.device).expand(env.num_envs, -1)
    orientation = torch.tensor(shelf.init_state.rot, device=env.device).expand(env.num_envs, -1)
    scale_x = (shelf.spawn.scale or (1, 1, 1))[0]
    result = torch.ones(env.num_envs, dtype=torch.bool, device=env.device)
    for name in ("bread_2", "bread_3"):
        point, quat = math_utils.subtract_frame_transforms(
            position, orientation, mdp.object_position(env, name), env.scene[name].data.root_quat_w,
        )
        result &= (point[:, 0].abs() < .085 * scale_x) & (point[:, 1].abs() < .06)
        result &= (point[:, 2] > .015) & (point[:, 2] < .16)
        rotation = math_utils.matrix_from_quat(quat)
        result &= torch.maximum(rotation[:, 2, 0].abs(), rotation[:, 2, 1].abs()) > .866
    return result


def reset_objects(env, env_ids):
    """Use configured calibration poses, never teleport objects during a demonstration."""
    if env_ids is None:
        env_ids = torch.arange(env.num_envs, device=env.device)
    names = ["toaster", "bread_0", "bread_1", "bread_2", "bread_3"]
    if getattr(env.cfg, "serve_to_plate", False):
        names.append("plate")
    for name in names:
        asset = env.scene[name]
        state = asset.data.default_root_state[env_ids].clone()
        state[:, :3] += env.scene.env_origins[env_ids]
        asset.write_root_state_to_sim(state, env_ids)
        if name == "toaster":
            joints = asset.data.default_joint_pos[env_ids].clone()
            asset.write_joint_state_to_sim(joints, torch.zeros_like(joints), env_ids=env_ids)
            asset.set_joint_position_target(joints, env_ids=env_ids)
