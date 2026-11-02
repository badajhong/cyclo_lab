"""Mimic API for two wrist frames and an auxiliary body lift frame."""

import torch
import isaaclab.utils.math as math_utils
from isaaclab.envs import ManagerBasedRLMimicEnv


class MakeToastFFWBG2BimanualEnv(ManagerBasedRLMimicEnv):
    EEF_NAMES = ("left", "right", "body")

    def __init__(self, cfg, **kwargs):
        if getattr(cfg, "scenario_id", None) is not None:
            from .scenarios import apply_scenario
            apply_scenario(cfg, cfg.scenario_id)
        self.toast_cycle = None
        self._restoring_cycle = None
        if getattr(cfg, "toast_cycle_enabled", False) and getattr(cfg.recorders, "record_initial_state", None):
            from .recorders import ToastCycleRecorderCfg
            cfg.recorders.record_toast_cycle = ToastCycleRecorderCfg()
        super().__init__(cfg, **kwargs)
        if getattr(cfg, "toast_cycle_enabled", False):
            from .toast_cycle import ToastCycle
            self.toast_cycle = ToastCycle(self)

    def _reset_idx(self, env_ids):
        super()._reset_idx(env_ids)
        if self.toast_cycle is not None:
            self.toast_cycle.reset(env_ids)
            if self._restoring_cycle is not None:
                self.toast_cycle.restore(self._restoring_cycle, env_ids)

    def step(self, action):
        if self.toast_cycle is not None:
            self.toast_cycle.update(self.step_dt)
        return super().step(action)

    def reset_to(self, state, env_ids=None, seed=None, is_relative=False):
        self._restoring_cycle = state.get("toast_cycle")
        try:
            observations = super().reset_to(state, env_ids, seed=seed, is_relative=is_relative)
        finally:
            self._restoring_cycle = None
        # InteractiveScene restores instantaneous joint velocities as drive targets.
        # These actions use position servos: retain the physical state velocity but
        # remove that persistent velocity feedforward before the next control step.
        ids = slice(None) if env_ids is None else env_ids
        for asset in self.scene.articulations.values():
            asset.set_joint_velocity_target(torch.zeros_like(asset.data.joint_vel[ids]), env_ids=env_ids)
        if self.toast_cycle is not None:
            self.toast_cycle.apply_targets()
        self.scene.write_data_to_sim()
        return observations

    def get_robot_eef_pose(self, eef_name, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        if eef_name not in self.EEF_NAMES:
            raise ValueError(eef_name)
        name = {"left": "arm_l_link7", "right": "arm_r_link7", "body": "arm_base_link"}[eef_name]
        robot = self.scene["robot"]
        index = robot.body_names.index(name)
        return math_utils.make_pose(
            robot.data.body_link_pos_w[ids, index] - self.scene.env_origins[ids],
            math_utils.matrix_from_quat(robot.data.body_link_quat_w[ids, index]),
        )

    def action_to_target_eef_pose(self, action):
        result = {}
        for name, offset in (("left", 0), ("right", 6)):
            pose = self.get_robot_eef_pose(name)
            robot = self.scene["robot"]
            pos, quat = math_utils.subtract_frame_transforms(
                robot.data.root_pos_w - self.scene.env_origins, robot.data.root_quat_w,
                pose[:, :3, 3], math_utils.quat_from_matrix(pose[:, :3, :3]),
            )
            pos, quat = math_utils.apply_delta_pose(pos, quat, action[:, offset:offset + 6] * 0.3)
            pos, quat = math_utils.combine_frame_transforms(
                robot.data.root_pos_w - self.scene.env_origins, robot.data.root_quat_w, pos, quat,
            )
            result[name] = math_utils.make_pose(pos, math_utils.matrix_from_quat(quat))
        body = self.get_robot_eef_pose("body").clone()
        robot = self.scene["robot"]
        current = robot.data.joint_pos[:, robot.joint_names.index("lift_joint")]
        body[:, 2, 3] += 0.25 * action[:, 14] - 0.25 - current
        result["body"] = body
        return result

    def target_eef_pose_to_action(self, target_eef_pose_dict, gripper_action_dict,
                                action_noise_dict=None, env_id=0):
        robot = self.scene["robot"]
        actions = []
        for name in ("left", "right"):
            target = target_eef_pose_dict[name]
            current = self.get_robot_eef_pose(name, [env_id])[0]
            base_pos = robot.data.root_pos_w[env_id] - self.scene.env_origins[env_id]
            base_quat = robot.data.root_quat_w[env_id]
            cp, cq = math_utils.subtract_frame_transforms(base_pos, base_quat, current[:3, 3],
                                                        math_utils.quat_from_matrix(current[:3, :3]))
            tp, tq = math_utils.subtract_frame_transforms(base_pos, base_quat, target[:3, 3],
                                                        math_utils.quat_from_matrix(target[:3, :3]))
            dp, dr = math_utils.compute_pose_error(cp[None], cq[None], tp[None], tq[None])
            delta = torch.cat((dp[0], dr[0])) / 0.3
            if action_noise_dict is not None:
                delta += float(action_noise_dict[name]) * torch.randn_like(delta)
            actions.append(delta)
        for name in ("left", "right"):
            actions.append(torch.as_tensor(gripper_action_dict[name], device=self.device).reshape(1))
        current_lift = robot.data.joint_pos[env_id, robot.joint_names.index("lift_joint")]
        body_z = self.get_robot_eef_pose("body", [env_id])[0, 2, 3]
        lift = current_lift + target_eef_pose_dict["body"][2, 3] - body_z
        actions.append(((lift + 0.25) / 0.25).clamp(-1, 1).reshape(1))
        return torch.cat(actions)

    def actions_to_gripper_actions(self, actions):
        return {"left": actions[..., 12:13], "right": actions[..., 13:14],
                "body": torch.zeros_like(actions[..., 14:15])}

    def get_object_poses(self, env_ids=None):
        ids = slice(None) if env_ids is None else env_ids
        names = ["bread_0", "bread_1", "bread_2", "bread_3", "toaster"]
        if getattr(self.cfg, "serve_to_plate", False):
            names.append("plate")
        return {name: math_utils.make_pose(
            self.scene[name].data.root_pos_w[ids] - self.scene.env_origins[ids],
            math_utils.matrix_from_quat(self.scene[name].data.root_quat_w[ids]),
        ) for name in names}

    def get_subtask_start_signals(self, env_ids=None):
        return {}

    def get_subtask_term_signals(self, env_ids=None):
        from . import bimanual_mdp

        ids = slice(None) if env_ids is None else env_ids
        signals = {"task_complete": bimanual_mdp.task_done(self)}
        if getattr(self.cfg, "serve_to_plate", False):
            ready = (self.toast_cycle.phase == 4 if self.toast_cycle is not None
                     else torch.zeros(self.num_envs, dtype=torch.bool, device=self.device))
            position = self.scene["bread_0"].data.root_pos_w - self.scene.env_origins
            left = self.get_robot_eef_pose("left")[:, :3, 3]
            from .assets import BIMANUAL_STANDBY_WRIST
            standby = torch.linalg.vector_norm(
                left - torch.tensor(BIMANUAL_STANDBY_WRIST, device=self.device), dim=-1,
            ) < .04
            first_placed = ready & (position[:, 2] < 1.05)
            signals.update({
                "toast_ready": ready,
                "extract_toast_0": ready & (position[:, 2] > 1.29),
                "serve_toast_0": first_placed & standby,
                "extract_toast_1": first_placed & (
                    self.scene["bread_1"].data.root_pos_w[:, 2] - self.scene.env_origins[:, 2] > 1.29),
            })
        return {name: value[ids] for name, value in signals.items()}
