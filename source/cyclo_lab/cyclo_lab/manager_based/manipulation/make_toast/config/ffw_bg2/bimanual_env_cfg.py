"""Two-arm Make Toast with a normalized body lift and synchronized Mimic replay."""

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
import torch
import isaaclab.utils.math as math_utils
from isaaclab.controllers import DifferentialIKControllerCfg
from isaaclab.envs.mdp.actions import JointPositionActionCfg
from isaaclab.envs.mimic_env_cfg import (
    MimicEnvCfg, SubTaskConfig, SubTaskConstraintConfig, SubTaskConstraintType,
    SubTaskConstraintCoordinationScheme,
)
from isaaclab.managers import ObservationTermCfg as ObsTerm, SceneEntityCfg
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.sensors import ContactSensorCfg
from isaaclab.utils import configclass

from .env_cfg import MakeToastFFWBG2EnvCfg
from . import bimanual_mdp
from .gripper_action import CoupledGripperPositionActionCfg
from .assets import (BREAD_OFFSETS, BREAD_ROTATION, TABLE_HEIGHT,
                     BIMANUAL_STANDBY_RIGHT_JOINTS, BIMANUAL_STANDBY_WRIST)
from .scene import spawn_toast_bimanual_robot, spawn_toast_bimanual_toaster
from .arm_action import StandbyPostureIKActionCfg


@configclass
class BimanualActionsCfg:
    # Action order is a dataset contract: left(6), right(6), left/right gripper(1 each), lift(1).
    left_arm = StandbyPostureIKActionCfg(
        asset_name="robot", joint_names=["arm_l_joint[1-7]"], body_name="arm_l_link7",
        controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
        scale=0.3,
        standby_position=BIMANUAL_STANDBY_WRIST,
    )
    right_arm = StandbyPostureIKActionCfg(
        asset_name="robot", joint_names=["arm_r_joint[1-7]"], body_name="arm_r_link7",
        standby_position=(BIMANUAL_STANDBY_WRIST[0], -BIMANUAL_STANDBY_WRIST[1],
                          BIMANUAL_STANDBY_WRIST[2]),
        controller=DifferentialIKControllerCfg(command_type="pose", use_relative_mode=True, ik_method="dls"),
        scale=0.3,
    )
    left_gripper = CoupledGripperPositionActionCfg(
        asset_name="robot", joint_names=["gripper_l_joint[1-4]"],
        open_command_expr={".*": 0.0}, close_command_expr={".*": 1.10},
    )
    right_gripper = CoupledGripperPositionActionCfg(
        asset_name="robot", joint_names=["gripper_r_joint[1-4]"],
        open_command_expr={".*": 0.0}, close_command_expr={".*": 1.10},
    )
    lift = JointPositionActionCfg(
        asset_name="robot", joint_names=["lift_joint"], use_default_offset=False,
        scale=0.25, offset=-0.25, clip={".*": (-0.5, 0.0)},
    )


@configclass
class MakeToastFFWBG2BimanualEnvCfg(MakeToastFFWBG2EnvCfg, MimicEnvCfg):
    """A separate 15D task; existing seven-dimensional recordings remain unchanged."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.robot = self.scene.robot.copy()
        self.scene.robot.spawn.func = spawn_toast_bimanual_robot
        self.actions = BimanualActionsCfg()
        self.scene.robot.spawn.collision_props = sim_utils.CollisionPropertiesCfg(
            contact_offset=.001, rest_offset=0.0,
        )
        # This USD omits mimic constraints; explicitly driven slave links need servo gains.
        self.scene.robot.actuators["gripper_slave"].stiffness = 100.0
        self.scene.robot.actuators["gripper_slave"].damping = 4.0
        pose = self.events.init_ffw_bg2_pose.params["joint_positions"]
        for i, value in enumerate(BIMANUAL_STANDBY_RIGHT_JOINTS, 1):
            pose[f"arm_r_joint{i}"] = value
            pose[f"arm_l_joint{i}"] = value * (1.0 if i in (1, 4, 6) else -1.0)
        pose.update({"head_joint1": 0.60, "head_joint2": 0.0})
        self.scene.robot.init_state.joint_pos.update(pose)
        # RH-P12 fingers need more clearance than the original ARX rack's 25 mm pitch.
        # Widen only the rack; keep bread/toaster geometry and slot clearances unchanged.
        shelf_position = torch.tensor([0.45, -0.25, TABLE_HEIGHT + 0.0382])
        shelf_rotation = torch.tensor([0.70710678, 0.0, 0.0, -0.70710678])
        self.scene.bread_shelf.spawn.scale = (2.0, 1.0, 1.0)
        self.scene.bread_shelf.init_state.pos = tuple(shelf_position.tolist())
        self.scene.bread_shelf.init_state.rot = tuple(shelf_rotation.tolist())
        # Present the lever toward the left hand, within the central arm workspace.
        self.scene.toaster.init_state.pos = (0.55, 0.23, TABLE_HEIGHT + 0.0818)
        self.scene.toaster.init_state.rot = (0.2588190451, 0.0, 0.0, 0.9659258263)  # yaw 150 degrees
        self.scene.toaster.spawn.activate_contact_sensors = True
        self.scene.toaster.spawn.func = spawn_toast_bimanual_toaster
        # Separate shell and lever contacts, so a wrist hitting the housing cannot
        # masquerade as a valid lever press. No collision pairs are disabled.
        robot_contact_paths = []
        for side, letter in (("left", "l"), ("right", "r")):
            links = ([f"arm_{letter}_link{i}" for i in range(1, 8)]
                     + [f"gripper_{letter}_rh_p12_rn_{suffix}" for suffix in ("r1", "r2", "l1", "l2")])
            robot_contact_paths.extend(f"{{ENV_REGEX_NS}}/Robot/ffw_bg2_follower/{side}_arm/{link}"
                                       for link in links)
        for name, body in (("toaster_shell_contact", "base"), ("toaster_lever_contact", "link_1")):
            setattr(self.scene, name, ContactSensorCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Toaster/{body}",
                # Enumerate rigid bodies: wildcard groups did not report these
                # articulated contacts in the Isaac Sim 5.1 tensor backend.
                filter_prim_paths_expr=robot_contact_paths,
                update_period=0.0, history_length=4,
            ))
        for i, offset in enumerate(BREAD_OFFSETS):
            position = torch.tensor([offset[0] * 2, offset[1], offset[2]])
            pose = self.scene.__dict__[f"bread_{i}"].init_state
            pose.pos = tuple((shelf_position + math_utils.quat_apply(shelf_rotation, position)).tolist())
            pose.rot = tuple(math_utils.quat_mul(shelf_rotation, torch.tensor(BREAD_ROTATION)).tolist())
        self.decimation = 4  # RoboDojo demonstrations are sampled at 25 Hz.
        self.sim.render_interval = self.decimation
        self.episode_length_s = 240.0
        self.events.reset_objects.func = bimanual_mdp.reset_objects
        self.terminations.success = DoneTerm(func=bimanual_mdp.task_done)
        self.observations.subtask_terms = None
        policy = self.observations.policy
        joints = [f"arm_{side}_joint{i}" for side in ("l", "r") for i in range(1, 8)] + ["lift_joint"]
        policy.joint_pos = ObsTerm(func=base_mdp.joint_pos_rel,
                                  params={"asset_cfg": SceneEntityCfg("robot", joint_names=joints, preserve_order=True)})
        policy.joint_vel = ObsTerm(func=base_mdp.joint_vel_rel,
                                  params={"asset_cfg": SceneEntityCfg("robot", joint_names=joints, preserve_order=True)})
        policy.gripper_pos = ObsTerm(func=bimanual_mdp.gripper_positions)
        policy.lift_pos = ObsTerm(func=bimanual_mdp.lift_position)
        self.scene.left_wrist_cam = self.scene.right_wrist_cam.copy()
        self.scene.left_wrist_cam.prim_path = self.scene.right_wrist_cam.prim_path.replace(
            "right_arm/arm_r_link7/camera_r_bottom_screw_frame/camera_r_link/right_wrist_cam",
            "left_arm/arm_l_link7/camera_l_bottom_screw_frame/camera_l_link/left_wrist_cam",
        )
        policy.left_wrist_cam = policy.right_wrist_cam.copy()
        policy.left_wrist_cam.params["sensor_cfg"] = SceneEntityCfg("left_wrist_cam")
        # Whole-episode synchronized replay is intentional during cross-robot calibration.
        # The auxiliary 'body' frame carries the lift trajectory through Mimic's pose API.
        self.subtask_configs = {name: [SubTaskConfig(
            object_ref="toaster", subtask_term_signal=None, selection_strategy="random",
            action_noise=0.0, num_interpolation_steps=0, num_fixed_steps=0,
            description="Replay the calibrated two-arm Make Toast motion and return to standby",
        )] for name in ("left", "right", "body")}
        self.task_constraint_configs = [SubTaskConstraintConfig(
            eef_subtask_constraint_tuple=[("left", 0), ("right", 0)],
            constraint_type=SubTaskConstraintType.COORDINATION,
            coordination_scheme=SubTaskConstraintCoordinationScheme.REPLAY,
        )]
        self.datagen_config.name = "make_toast_ffw_bg2_bimanual"
        self.datagen_config.generation_guarantee = False
        self.datagen_config.generation_keep_failed = False
        self.datagen_config.generation_num_trials = 300
        self.datagen_config.generation_select_src_per_subtask = False
        self.datagen_config.generation_select_src_per_arm = False
        self.datagen_config.generation_transform_first_robot_pose = False
        self.datagen_config.generation_interpolate_from_last_target_pose = True


@configclass
class MakeToastFFWBG2BimanualCycleEnvCfg(MakeToastFFWBG2BimanualEnvCfg):
    """Version 1 adds a physical bread carriage and a five-second pop-up cycle."""

    toast_cycle_enabled: bool = True

    def __post_init__(self):
        super().__post_init__()
        from isaaclab.actuators import ImplicitActuatorCfg
        from .scene import spawn_toast_popup_toaster
        self.scene.toaster.spawn.func = spawn_toast_popup_toaster
        self.scene.toaster.init_state.joint_pos["carriage_joint"] = 0.0
        self.scene.toaster.actuators["passive"].joint_names_expr = ["joint_[0-2]"]
        self.scene.toaster.actuators["carriage"] = ImplicitActuatorCfg(
            joint_names_expr=["carriage_joint"], stiffness=8000.0, damping=120.0,
            effort_limit_sim=40.0, velocity_limit_sim=.12,
        )
        self.observations.policy.toast_cycle = ObsTerm(func=bimanual_mdp.toast_cycle_state)


@configclass
class MakeToastFFWBG2BimanualServeEnvCfg(MakeToastFFWBG2BimanualCycleEnvCfg):
    """Version 2 finishes by serving both popped slices on a small dark blue plate."""

    serve_to_plate: bool = True
    toast_popup_extra_height: float = .015
    # Optional fixed layout. Applied before InteractiveScene is constructed.
    scenario_id: str | None = None
    scenario_manifest: str | None = None

    def __post_init__(self):
        super().__post_init__()
        from isaaclab.assets import RigidObjectCfg
        from .assets import SERVING_PLATE_POSITION, SERVING_PLATE_SCALE, asset_path
        from .scene import spawn_toast_serving_plate
        from .scene import spawn_toast_serving_toaster
        from . import mdp
        self.scene.toaster.spawn.func = spawn_toast_serving_toaster
        self.scene.plate = RigidObjectCfg(
            prim_path="{ENV_REGEX_NS}/Plate",
            spawn=sim_utils.UsdFileCfg(
                usd_path=asset_path("plate"), func=spawn_toast_serving_plate,
                scale=SERVING_PLATE_SCALE,
                activate_contact_sensors=True,
                rigid_props=sim_utils.RigidBodyPropertiesCfg(
                    solver_position_iteration_count=16, solver_velocity_iteration_count=4,
                    max_depenetration_velocity=1.0,
                ),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=.001, rest_offset=0),
                mass_props=sim_utils.MassPropertiesCfg(mass=.35),
                semantic_tags=[("class", "plate")],
            ),
            init_state=RigidObjectCfg.InitialStateCfg(pos=SERVING_PLATE_POSITION),
        )
        self.observations.policy.plate_pos = ObsTerm(func=mdp.object_position, params={"name": "plate"})
        self.observations.policy.plate_rot = ObsTerm(func=mdp.object_orientation, params={"name": "plate"})
        self.scene.plate_contact = ContactSensorCfg(
            prim_path="{ENV_REGEX_NS}/Plate", update_period=0.0, history_length=4,
            filter_prim_paths_expr=list(self.scene.toaster_shell_contact.filter_prim_paths_expr),
        )
        # Popped bread can settle at a different angle on each physics replay.
        # Re-anchor extraction to that bread, then placement to the plate, while
        # keeping both arms synchronized and the lift on its recorded trajectory.
        segments = (("toaster", "toast_ready"), ("bread_0", "extract_toast_0"),
                    ("plate", "serve_toast_0"), ("bread_1", "extract_toast_1"), ("plate", None))
        self.subtask_configs = {name: [SubTaskConfig(
            object_ref=(reference if name == "left" or (name == "right" and index == 0) else None),
            subtask_term_signal=signal, subtask_term_offset_range=(0, 0),
            selection_strategy="random", action_noise=0.0,
            num_interpolation_steps=15 if index else 0, num_fixed_steps=0,
        ) for index, (reference, signal) in enumerate(segments)] for name in ("left", "right", "body")}
        self.datagen_config.generation_interpolate_from_last_target_pose = False
        # Equal boundaries and interpolation lengths keep all three streams aligned.
        # During serving the right hand stays at standby; transforming it about
        # a tilted bread frame would move it unnecessarily. This also uses the
        # established object-relative path in IsaacLab 2.3, whose coordinated
        # TRANSFORM path calls an unavailable get_delta_object_pose helper.
        self.task_constraint_configs = [SubTaskConstraintConfig(
            eef_subtask_constraint_tuple=[("left", 0), ("right", 0)],
            constraint_type=SubTaskConstraintType.COORDINATION,
            coordination_scheme=SubTaskConstraintCoordinationScheme.REPLAY,
        )]
