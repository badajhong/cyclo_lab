"""Make Toast objects on the original Cyclo Lab FFW-BG2 PickPlace setup."""

import isaaclab.envs.mdp as base_mdp
import isaaclab.sim as sim_utils
from isaaclab.actuators import ImplicitActuatorCfg
from isaaclab.assets import ArticulationCfg, AssetBaseCfg, RigidObjectCfg
from isaaclab.managers import EventTermCfg as EventTerm
from isaaclab.managers import ObservationGroupCfg as ObsGroup
from isaaclab.managers import ObservationTermCfg as ObsTerm
from isaaclab.managers import TerminationTermCfg as DoneTerm
from isaaclab.utils import configclass

from cyclo_lab.manager_based.manipulation.pick_place.config.ffw_bg2.ik_rel_env_cfg import PickPlaceFFWBG2EnvCfg
from cyclo_lab.manager_based.manipulation.pick_place.config.ffw_bg2.joint_pos_env_cfg import EventCfg as PickPlaceEvents
from cyclo_lab.manager_based.manipulation.pick_place.config.ffw_bg2.pick_place_env_cfg import ObservationsCfg

from . import mdp
from .assets import (
    BREAD_OFFSETS, BREAD_ROTATION, LEVER_OPEN, SHELF_POSITION, TOASTER_POSITION, TOASTER_ROTATION, asset_path,
)
from .scene import spawn_toast_table


@configclass
class MakeToastPolicyCfg(ObservationsCfg.PolicyCfg):
    object_pos = None
    object_rot = None
    bread_0_pos = ObsTerm(func=mdp.object_position, params={"name": "bread_0"})
    bread_0_rot = ObsTerm(func=mdp.object_orientation, params={"name": "bread_0"})
    bread_1_pos = ObsTerm(func=mdp.object_position, params={"name": "bread_1"})
    bread_1_rot = ObsTerm(func=mdp.object_orientation, params={"name": "bread_1"})
    toaster_pos = ObsTerm(func=mdp.object_position, params={"name": "toaster"})
    toaster_rot = ObsTerm(func=mdp.object_orientation, params={"name": "toaster"})
    toaster_lever = ObsTerm(func=mdp.lever_position)


@configclass
class MakeToastSubtasksCfg(ObsGroup):
    grasp_bread_0 = ObsTerm(func=mdp.bread_grasped, params={"bread": "bread_0"})
    place_bread_0 = ObsTerm(func=mdp.bread_placed, params={"bread": "bread_0"})
    grasp_bread_1 = ObsTerm(func=mdp.bread_grasped, params={"bread": "bread_1"})
    place_bread_1 = ObsTerm(func=mdp.bread_placed, params={"bread": "bread_1"})
    press_lever = ObsTerm(func=mdp.lever_pressed)

    def __post_init__(self):
        self.concatenate_terms = False
        self.enable_corruption = False


@configclass
class MakeToastObservationsCfg(ObservationsCfg):
    policy: MakeToastPolicyCfg = MakeToastPolicyCfg()
    subtask_terms: MakeToastSubtasksCfg = MakeToastSubtasksCfg()


@configclass
class MakeToastEventsCfg:
    reset_scene = EventTerm(func=base_mdp.reset_scene_to_default, mode="reset")
    init_ffw_bg2_pose = PickPlaceEvents().init_ffw_bg2_pose
    reset_objects = EventTerm(func=mdp.reset_objects, mode="reset")


@configclass
class MakeToastTerminationsCfg:
    time_out = DoneTerm(func=base_mdp.time_out, time_out=True)
    bread_dropping = DoneTerm(func=mdp.bread_dropped)
    success = DoneTerm(func=mdp.task_done)


@configclass
class MakeToastFFWBG2EnvCfg(PickPlaceFFWBG2EnvCfg):
    """Same robot, table, camera observations, and seven-dimensional IK/gripper action."""

    def __post_init__(self):
        super().__post_init__()
        self.scene.object = None
        self.scene.table.spawn.func = spawn_toast_table
        self.scene.env_spacing = 3.0
        self.episode_length_s = 90.0
        self.events = MakeToastEventsCfg()
        # Mirror the raised right-arm reset pose using the robot's authored joint axes.
        # Y-axis joints keep their sign; X/Z-axis joints change sign across the sagittal plane.
        pose = self.events.init_ffw_bg2_pose.params["joint_positions"]
        for index in range(1, 8):
            pose[f"arm_l_joint{index}"] = pose.get(f"arm_r_joint{index}", 0.0) * (
                1.0 if index in (1, 4, 6) else -1.0
            )
        self.terminations = MakeToastTerminationsCfg()
        # Keep the original named arm observations and cameras configured by the parent.
        original_policy = self.observations.policy
        self.observations = MakeToastObservationsCfg()
        self.observations.policy.joint_pos = original_policy.joint_pos
        self.observations.policy.joint_vel = original_policy.joint_vel
        self.scene.toaster = ArticulationCfg(
            prim_path="{ENV_REGEX_NS}/Toaster",
            spawn=sim_utils.UsdFileCfg(
                usd_path=asset_path("toaster"),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
                articulation_props=sim_utils.ArticulationRootPropertiesCfg(
                    enabled_self_collisions=False, solver_position_iteration_count=16,
                    solver_velocity_iteration_count=4,
                ),
                semantic_tags=[("class", "toaster")],
            ),
            init_state=ArticulationCfg.InitialStateCfg(
                pos=TOASTER_POSITION, rot=TOASTER_ROTATION,
                joint_pos={"joint_0": 0.0, "joint_1": LEVER_OPEN, "joint_2": 0.0},
                joint_vel={".*": 0.0},
            ),
            actuators={"passive": ImplicitActuatorCfg(
                joint_names_expr=[".*"], stiffness=None, damping=None,
            )},
            soft_joint_pos_limit_factor=1.0,
        )
        self.scene.bread_shelf = AssetBaseCfg(
            prim_path="{ENV_REGEX_NS}/BreadShelf",
            spawn=sim_utils.UsdFileCfg(
                usd_path=asset_path("bread_shelf"),
                rigid_props=sim_utils.RigidBodyPropertiesCfg(rigid_body_enabled=False),
                collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
                semantic_tags=[("class", "bread_rack")],
            ),
            init_state=AssetBaseCfg.InitialStateCfg(pos=SHELF_POSITION),
        )
        for i, offset in enumerate(BREAD_OFFSETS):
            setattr(self.scene, f"bread_{i}", RigidObjectCfg(
                prim_path=f"{{ENV_REGEX_NS}}/Bread_{i}",
                spawn=sim_utils.UsdFileCfg(
                    usd_path=asset_path("bread"),
                    rigid_props=sim_utils.RigidBodyPropertiesCfg(
                        max_depenetration_velocity=1.0, solver_position_iteration_count=16,
                        solver_velocity_iteration_count=4,
                    ),
                    mass_props=sim_utils.MassPropertiesCfg(mass=0.07),
                    collision_props=sim_utils.CollisionPropertiesCfg(contact_offset=0.001, rest_offset=0.0),
                    semantic_tags=[("class", "bread"), ("slice", str(i))],
                ),
                init_state=RigidObjectCfg.InitialStateCfg(
                    pos=tuple(p + d for p, d in zip(SHELF_POSITION, offset)), rot=BREAD_ROTATION,
                ),
            ))
