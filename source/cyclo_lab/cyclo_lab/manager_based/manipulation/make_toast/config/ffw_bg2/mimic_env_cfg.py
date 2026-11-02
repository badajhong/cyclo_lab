"""Object-relative grasp, insert, lever press, and retreat segments for Isaac Lab Mimic."""

from isaaclab.envs.mimic_env_cfg import MimicEnvCfg, SubTaskConfig
from isaaclab.utils import configclass

from .env_cfg import MakeToastFFWBG2EnvCfg


@configclass
class MakeToastFFWBG2MimicEnvCfg(MakeToastFFWBG2EnvCfg, MimicEnvCfg):
    def __post_init__(self):
        super().__post_init__()
        self.datagen_config.name = "make_toast_ffw_bg2"
        self.datagen_config.generation_guarantee = False
        self.datagen_config.generation_keep_failed = False
        self.datagen_config.generation_num_trials = 300
        self.datagen_config.generation_select_src_per_subtask = False
        self.datagen_config.generation_select_src_per_arm = False
        self.datagen_config.generation_relative = False
        self.datagen_config.generation_joint_pos = False
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True
        self.datagen_config.seed = 1

        segments = (
            ("bread_0", "grasp_bread_0", "Grasp bread 0 (rightmost slice on the rack)", 0.002),
            ("toaster", "place_bread_0", "Insert bread 0 into an empty slot and release it", 0.0),
            ("bread_1", "grasp_bread_1", "Grasp bread 1 (the next slice on the rack)", 0.002),
            ("toaster", "place_bread_1", "Insert bread 1 into the other slot and release it", 0.0),
            ("toaster", "press_lever", "Press the toaster lever fully down", 0.0),
            (None, None, "Open the gripper and return the right arm to its starting pose", 0.0),
        )
        self.subtask_configs = {"ffw_bg2": [
            SubTaskConfig(
                object_ref=reference,
                subtask_term_signal=signal,
                subtask_term_offset_range=(0, 0),
                selection_strategy="nearest_neighbor_object" if reference else "random",
                selection_strategy_kwargs={"nn_k": 3} if reference else {},
                action_noise=noise,
                num_interpolation_steps=5 if index else 0,
                num_fixed_steps=0,
                apply_noise_during_interpolation=False,
                description=description,
                next_subtask_description=segments[index + 1][2] if index + 1 < len(segments) else None,
            ) for index, (reference, signal, description, noise) in enumerate(segments)
        ]}
