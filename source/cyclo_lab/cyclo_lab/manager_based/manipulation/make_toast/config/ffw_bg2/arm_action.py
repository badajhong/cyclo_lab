"""Recover redundant arm posture while holding the raised standby wrist pose."""

import torch
from isaaclab.envs.mdp.actions import DifferentialInverseKinematicsActionCfg
from isaaclab.envs.mdp.actions.task_space_actions import DifferentialInverseKinematicsAction
from isaaclab.utils import configclass


class StandbyPostureIKAction(DifferentialInverseKinematicsAction):
    """Keep six Cartesian commands and use the seventh joint's null space at standby."""

    def process_actions(self, actions):
        super().process_actions(actions)
        # Recompute at the 25 Hz action rate, rather than running an SVD for each
        # 100 Hz physics substep. The primary Cartesian IK still runs every substep.
        jacobian = self._compute_frame_jacobian()
        self._posture_projector = (torch.eye(self._num_joints, device=self.device)
                                   - torch.linalg.pinv(jacobian, rtol=1e-4) @ jacobian)

    def apply_actions(self):
        position, orientation = self._compute_frame_pose()
        joints = self._asset.data.joint_pos[:, self._joint_ids]
        jacobian = self._compute_frame_jacobian()
        desired = self._ik_controller.compute(position, orientation, jacobian, joints)
        home_position = torch.tensor(self.cfg.standby_position, device=self.device)
        near_home = (torch.linalg.vector_norm(position - home_position, dim=-1) < .05)
        near_home &= torch.linalg.vector_norm(self._ik_controller.ee_pos_des - home_position, dim=-1) < .05
        # Exact task null space: changing the redundant posture leaves the wrist's
        # commanded position/orientation unchanged to first order.
        projector = self._posture_projector
        limits = self._asset.data.soft_joint_pos_limits[:, self._joint_ids]
        error = (self._asset.data.default_joint_pos[:, self._joint_ids] - joints) * near_home.unsqueeze(-1)
        # Keep spare joint travel during approach/pressing, where the task can
        # otherwise stall at a wrist limit despite having a feasible IK solution.
        repulsion = ((.15 - (joints - limits[..., 0])).clamp_min(0)
                     - (.15 - (limits[..., 1] - joints)).clamp_min(0)) / .15
        correction = (projector @ (error + 3 * repulsion).unsqueeze(-1)).squeeze(-1) * .01
        correction *= (.01 / correction.abs().amax(dim=-1, keepdim=True).clamp_min(.01)).clamp(max=1.0)
        desired += correction
        desired = desired.clamp(min=limits[..., 0], max=limits[..., 1])
        self._asset.set_joint_position_target(desired, self._joint_ids)


@configclass
class StandbyPostureIKActionCfg(DifferentialInverseKinematicsActionCfg):
    class_type: type = StandbyPostureIKAction
    standby_position: tuple[float, float, float] = (0.35621, 0.36224, 1.31594)
