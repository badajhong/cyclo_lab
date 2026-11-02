"""Mimic adapters for the existing FFW-BG2 relative IK controller."""

import torch

from isaaclab.envs import ManagerBasedRLMimicEnv
import isaaclab.utils.math as math_utils

from . import mdp


class MakeToastFFWBG2MimicEnv(ManagerBasedRLMimicEnv):
    EEF_NAME = "ffw_bg2"

    def get_robot_eef_pose(self, eef_name, env_ids=None):
        if eef_name != self.EEF_NAME:
            raise ValueError(f"Unknown end effector: {eef_name}")
        ids = slice(None) if env_ids is None else env_ids
        sensor = self.scene["ee_frame"].data
        return math_utils.make_pose(
            sensor.target_pos_w[ids, 0] - self.scene.env_origins[ids],
            math_utils.matrix_from_quat(sensor.target_quat_w[ids, 0]),
        )

    def _eef_base_pose(self):
        pose = self.get_robot_eef_pose(self.EEF_NAME)
        robot = self.scene["robot"]
        return math_utils.subtract_frame_transforms(
            robot.data.root_pos_w - self.scene.env_origins, robot.data.root_quat_w,
            pose[:, :3, 3], math_utils.quat_from_matrix(pose[:, :3, :3]),
        )

    def _action_scale(self):
        # Use exactly the scaling configured on the original Cyclo Lab IK action.
        scale = torch.as_tensor(self.action_manager.get_term("arm_action").cfg.scale, device=self.device)
        return scale.expand(6)

    def action_to_target_eef_pose(self, action):
        position, orientation = self._eef_base_pose()
        target_position, target_orientation = math_utils.apply_delta_pose(
            position, orientation, action[:, :6] * self._action_scale(),
        )
        robot = self.scene["robot"]
        target_position, target_orientation = math_utils.combine_frame_transforms(
            robot.data.root_pos_w - self.scene.env_origins, robot.data.root_quat_w,
            target_position, target_orientation,
        )
        return {self.EEF_NAME: math_utils.make_pose(target_position, math_utils.matrix_from_quat(target_orientation))}

    def target_eef_pose_to_action(self, target_eef_pose_dict, gripper_action_dict,
                                action_noise_dict=None, env_id=0):
        target = target_eef_pose_dict[self.EEF_NAME]
        robot = self.scene["robot"]
        target_pos, target_quat = math_utils.subtract_frame_transforms(
            robot.data.root_pos_w[env_id] - self.scene.env_origins[env_id],
            robot.data.root_quat_w[env_id], target[:3, 3], math_utils.quat_from_matrix(target[:3, :3]),
        )
        current_pos, current_quat = self._eef_base_pose()
        delta_pos, delta_rot = math_utils.compute_pose_error(
            current_pos[env_id:env_id + 1], current_quat[env_id:env_id + 1],
            target_pos.unsqueeze(0), target_quat.unsqueeze(0),
            rot_error_type="axis_angle",
        )
        # Without dividing by 0.3, generated trajectories would systematically undershoot.
        action = torch.cat((delta_pos[0], delta_rot[0])) / self._action_scale()
        if action_noise_dict is not None:
            action = action + float(action_noise_dict[self.EEF_NAME]) * torch.randn_like(action)
        gripper = torch.as_tensor(gripper_action_dict[self.EEF_NAME], device=self.device).reshape(1)
        return torch.cat((action, gripper))

    def actions_to_gripper_actions(self, actions):
        return {self.EEF_NAME: actions[..., -1:]}

    def get_object_poses(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        poses = {}
        # Articulations are omitted by the base Mimic implementation; include the toaster explicitly.
        for name in ("bread_0", "bread_1", "bread_2", "bread_3", "toaster"):
            asset = self.scene[name]
            poses[name] = math_utils.make_pose(
                asset.data.root_pos_w[ids] - self.scene.env_origins[ids],
                math_utils.matrix_from_quat(asset.data.root_quat_w[ids]),
            )
        return poses

    def get_subtask_start_signals(self, env_ids=None):
        return {}

    def get_subtask_term_signals(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        return {
            "grasp_bread_0": mdp.bread_grasped(self, "bread_0")[ids],
            "place_bread_0": mdp.bread_placed(self, "bread_0")[ids],
            "grasp_bread_1": mdp.bread_grasped(self, "bread_1")[ids],
            "place_bread_1": mdp.bread_placed(self, "bread_1")[ids],
            "press_lever": mdp.lever_pressed(self)[ids],
        }
