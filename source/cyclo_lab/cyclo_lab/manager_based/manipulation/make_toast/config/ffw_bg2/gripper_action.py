"""One normalized command drives the four authored FFW gripper linkage joints."""

from isaaclab.envs.mdp.actions import BinaryJointPositionAction, BinaryJointPositionActionCfg
from isaaclab.utils import configclass


class CoupledGripperPositionAction(BinaryJointPositionAction):
    """Preserve intermediate openings during cross-robot handover calibration."""

    def process_actions(self, actions):
        self._raw_actions[:] = actions
        opening = ((actions + 1.0) * 0.5).clamp(0.0, 1.0)
        self._processed_actions[:] = opening * self._open_command + (1.0 - opening) * self._close_command


@configclass
class CoupledGripperPositionActionCfg(BinaryJointPositionActionCfg):
    class_type: type = CoupledGripperPositionAction
