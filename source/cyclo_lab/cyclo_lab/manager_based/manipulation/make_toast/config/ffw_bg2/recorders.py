"""Record synchronized Mimic metadata while the scene expert executes physically."""

from isaaclab.envs.mdp.recorders.recorders_cfg import ActionStateRecorderManagerCfg
from isaaclab.managers import RecorderTerm, RecorderTermCfg
from isaaclab.utils import configclass


class ToastDatagenInfoRecorder(RecorderTerm):
    def record_pre_step(self):
        env = self._env
        return "obs/datagen_info", {
            "object_pose": env.get_object_poses(),
            "eef_pose": {name: env.get_robot_eef_pose(name) for name in env.cfg.subtask_configs},
            "target_eef_pose": env.action_to_target_eef_pose(env.action_manager.action),
            "subtask_term_signals": env.get_subtask_term_signals(),
        }


@configclass
class ToastDatagenInfoRecorderCfg(RecorderTermCfg):
    class_type: type[RecorderTerm] = ToastDatagenInfoRecorder


@configclass
class ToastExpertRecorderCfg(ActionStateRecorderManagerCfg):
    record_pre_step_datagen_info = ToastDatagenInfoRecorderCfg()


class ToastCycleRecorder(RecorderTerm):
    """Include the latch/timer so a mid-cycle state can be restored correctly."""

    def record_post_reset(self, env_ids):
        return "initial_state/toast_cycle", self._env.toast_cycle.state(env_ids)

    def record_post_step(self):
        return "states/toast_cycle", self._env.toast_cycle.state()


@configclass
class ToastCycleRecorderCfg(RecorderTermCfg):
    class_type: type[RecorderTerm] = ToastCycleRecorder
