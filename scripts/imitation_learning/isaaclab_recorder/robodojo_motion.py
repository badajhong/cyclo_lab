"""Motion-only RoboDojo input and explicit, hash-bound cross-robot calibration."""

import hashlib
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def read_motion(path):
    path = Path(path)
    with np.load(path, allow_pickle=False) as archive:
        data = {key: archive[key].copy() for key in archive.files}
    for side in ("left", "right"):
        for group in ("state", "action"):
            pose = data[f"{group}_{side}_ee_poses"]
            grip = data[f"{group}_{side}_ee_joint_states"]
            if pose.ndim != 2 or pose.shape[1] != 7 or grip.shape != (len(pose), 1):
                raise ValueError(f"Invalid {group}/{side} motion shape")
            if not np.isfinite(pose).all() or not np.isfinite(grip).all():
                raise ValueError("Non-finite source motion")
            if not np.allclose(np.linalg.norm(pose[:, 3:], axis=1), 1.0, atol=.01):
                raise ValueError("Source quaternions must use normalized wxyz")
    data["sha256"] = hashlib.sha256(path.read_bytes()).hexdigest()
    return data


def pose_matrix(poses):
    poses = np.asarray(poses)
    matrix = np.broadcast_to(np.eye(4), (*poses.shape[:-1], 4, 4)).copy()
    matrix[..., :3, :3] = Rotation.from_quat(poses[..., [4, 5, 6, 3]].reshape(-1, 4)).as_matrix().reshape(
        *poses.shape[:-1], 3, 3
    )
    matrix[..., :3, 3] = poses[..., :3]
    return matrix


def matrix_pose(matrix):
    q = Rotation.from_matrix(np.asarray(matrix)[..., :3, :3]).as_quat()
    return np.concatenate((np.asarray(matrix)[..., :3, 3], q[..., [3, 0, 1, 2]]), axis=-1)


def make_transform(position, yaw=0.0):
    matrix = np.eye(4)
    matrix[:3, :3] = Rotation.from_euler("z", yaw).as_matrix()
    matrix[:3, 3] = position
    return matrix


def source_tcp(motion, side, offset=(.145, 0, 0), group="state"):
    wrist = pose_matrix(motion[f"{group}_{side}_ee_poses"])
    tool = np.eye(4)
    tool[:3, 3] = offset
    return wrist @ tool


def fit_calibration(motion):
    """Fit a candidate from pickup/slot contact landmarks; physical replay must validate it.

    Public motion files do not contain object poses. This fit is explicitly an estimate,
    not a claim that an evaluation layout is the paired demonstration layout.
    """
    right_grip = motion["action_right_ee_joint_states"][:, 0]
    left_grip = motion["action_left_ee_joint_states"][:, 0]
    pickups = np.where((right_grip[:-1] > .5) & (right_grip[1:] <= .5))[0][:2] + 1
    releases = np.where((left_grip[:-1] <= .5) & (left_grip[1:] > .5))[0][:2] + 1
    if len(pickups) != 2 or len(releases) != 2:
        raise ValueError("Expected a two-slice handover demonstration; contact landmarks not found")
    right_tcp = source_tcp(motion, "right")
    left_tcp = source_tcp(motion, "left")
    difference = right_tcp[pickups[0], :2, 3] - right_tcp[pickups[1], :2, 3]
    shelf_yaw = np.arctan2(difference[1], difference[0])
    shelf_rotation = Rotation.from_euler("z", shelf_yaw)
    shelf_xy = right_tcp[pickups[0], :3, 3] - shelf_rotation.apply([.035 + .0056, -.007, .0832])
    source_table_height = .765
    shelf_xy[2] = source_table_height + .0382
    placed = left_tcp[releases, :3, 3]
    delta = placed[1] - placed[0]
    toaster_yaw = np.arctan2(delta[1], delta[0]) - np.pi / 2
    # Place slot centers symmetrically around the two observed release contacts.
    toaster_x = np.mean(placed[:, 0] - np.array([-.026, .0265]))
    # The lever contact is 0.1087 m along the toaster's +X direction.
    after = min(int(releases[-1] + 10), len(left_tcp) - 1)
    press = after + np.argmin(left_tcp[after:, 2, 3])
    lever_xy = Rotation.from_euler("z", toaster_yaw).apply([.1087, 0, 0])
    toaster_y = left_tcp[press, 1, 3] - lever_xy[1]
    world = make_transform([.65, -.05, 1.0 - source_table_height], -np.pi / 2)
    objects = {
        "bread_shelf": matrix_pose(world @ make_transform(shelf_xy, shelf_yaw)).tolist(),
        "toaster": matrix_pose(world @ make_transform(
            [toaster_x, toaster_y, source_table_height + .0818], toaster_yaw,
        )).tolist(),
    }
    return {
        "schema_version": 1, "source_sha256": motion["sha256"],
        "status": "candidate_requires_physical_validation",
        "source_to_env": world.tolist(),
        "source_wrist_to_tcp": [.145, 0.0, 0.0],
        "target_wrist_to_tcp": [0.0, 0.0, -.17],
        "target_gripper_linkage": {"fixed_depth_m": .151, "link_length_m": .057,
                                   "open_angle_rad": float(np.pi / 6), "closed_joint_rad": 1.10},
        "target_wrist_rotation_in_source_wrist": Rotation.from_euler("y", -90, degrees=True).as_matrix().tolist(),
        "objects": objects,
        "pickup_frames": pickups.tolist(), "release_frames": releases.tolist(), "lever_frame": int(press),
        "pickup_spacing_residual_m": float(abs(np.linalg.norm(difference) - .025)),
        "slot_spacing_residual_m": float(abs(np.linalg.norm(delta[:2]) - .0525)),
        "lift_min_m": -.08, "lift_max_m": 0.0, "lift_reference_wrist_z_m": 1.30,
        "notes": [
            "ARX link6 tool translation follows upstream gripper_bias=0.145 m; source closing axis is +Y.",
            "FFW tool translation is the center of the authored distal finger collision bounds; closing axis is +Y.",
            "Object poses are contact-fitted estimates because the motion HDF5 has no object-state stream.",
            "Only physically successful FFW-BG2 episodes may become training data.",
        ],
    }


def load_calibration(path, motion):
    value = json.loads(Path(path).read_text())
    if value.get("schema_version") != 1 or value.get("source_sha256") != motion["sha256"]:
        raise ValueError("Calibration schema/source hash mismatch; calibrate this exact motion file")
    world = np.asarray(value["source_to_env"], dtype=float)
    if world.shape != (4, 4) or not np.isfinite(world).all():
        raise ValueError("Invalid source-to-environment transform")
    rotation = world[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1e-5) or not np.isclose(np.linalg.det(rotation), 1):
        raise ValueError("Calibration must be a rigid transform, without reflection or scale")
    if not -.5 <= value["lift_min_m"] <= value["lift_max_m"] <= 0:
        raise ValueError("Lift calibration exceeds the authored FFW-BG2 limits")
    return value


def retarget_wrist_targets(motion, calibration, side):
    tcp = source_tcp(motion, side, calibration["source_wrist_to_tcp"], group="action")
    world = np.asarray(calibration["source_to_env"])
    target = world @ tcp
    target[:, :3, :3] = target[:, :3, :3] @ np.asarray(calibration["target_wrist_rotation_in_source_wrist"])
    if "target_gripper_linkage" in calibration:
        linkage = calibration["target_gripper_linkage"]
        opening = motion[f"action_{side}_ee_joint_states"][:, 0].clip(0, 1)
        depth = linkage["fixed_depth_m"] + linkage["link_length_m"] * np.sin(
            linkage["open_angle_rad"] + linkage["closed_joint_rad"] * (1.0 - opening)
        )
        offset = np.zeros((len(target), 3))
        offset[:, 2] = -depth
        target[:, :3, 3] -= np.einsum("nij,nj->ni", target[:, :3, :3], offset)
    else:
        target[:, :3, 3] -= np.einsum("nij,j->ni", target[:, :3, :3], calibration["target_wrist_to_tcp"])
    return target
