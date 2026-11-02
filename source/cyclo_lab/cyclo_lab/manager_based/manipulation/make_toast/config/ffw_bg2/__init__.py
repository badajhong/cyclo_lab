"""FFW-BG2 Make Toast registrations."""

import os

import gymnasium as gym

from cyclo_lab.manager_based.manipulation.pick_place.config.ffw_bg2 import agents

from .env_cfg import MakeToastFFWBG2EnvCfg
from .mimic_env import MakeToastFFWBG2MimicEnv
from .mimic_env_cfg import MakeToastFFWBG2MimicEnvCfg
from .bimanual_env import MakeToastFFWBG2BimanualEnv
from .bimanual_env_cfg import (
    MakeToastFFWBG2BimanualEnvCfg, MakeToastFFWBG2BimanualCycleEnvCfg, MakeToastFFWBG2BimanualServeEnvCfg,
)


gym.register(
    id="Cyclo-Make-Toast-FFW-BG2-IK-Rel-v0",
    entry_point="isaaclab.envs:ManagerBasedRLEnv",
    disable_env_checker=True,
    kwargs={
        "env_cfg_entry_point": MakeToastFFWBG2EnvCfg,
        "robomimic_bc_cfg_entry_point": os.path.join(agents.__path__[0], "robomimic/bc_rnn_image.json"),
    },
)

gym.register(
    id="Cyclo-Make-Toast-FFW-BG2-Mimic-v0",
    entry_point=f"{__name__}:MakeToastFFWBG2MimicEnv",
    disable_env_checker=True,
    kwargs={"env_cfg_entry_point": MakeToastFFWBG2MimicEnvCfg},
)

for task_id in ("Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v0", "Cyclo-Make-Toast-FFW-BG2-Bimanual-Mimic-v0"):
    gym.register(
        id=task_id, entry_point=f"{__name__}:MakeToastFFWBG2BimanualEnv", disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": MakeToastFFWBG2BimanualEnvCfg,
            "robomimic_bc_cfg_entry_point": os.path.join(os.path.dirname(__file__), "bc_bimanual_image.json"),
        },
    )

for task_id in ("Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v1", "Cyclo-Make-Toast-FFW-BG2-Bimanual-Mimic-v1"):
    gym.register(
        id=task_id, entry_point=f"{__name__}:MakeToastFFWBG2BimanualEnv", disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": MakeToastFFWBG2BimanualCycleEnvCfg,
            "robomimic_bc_cfg_entry_point": os.path.join(os.path.dirname(__file__), "bc_bimanual_image.json"),
        },
    )

for task_id in ("Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2", "Cyclo-Make-Toast-FFW-BG2-Bimanual-Mimic-v2"):
    gym.register(
        id=task_id, entry_point=f"{__name__}:MakeToastFFWBG2BimanualEnv", disable_env_checker=True,
        kwargs={
            "env_cfg_entry_point": MakeToastFFWBG2BimanualServeEnvCfg,
            "robomimic_bc_cfg_entry_point": os.path.join(os.path.dirname(__file__), "bc_bimanual_image.json"),
        },
    )
