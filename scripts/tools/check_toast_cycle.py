"""Check physical carriage contact, insertion depth, timing, reset, and state restore.

This diagnostic places objects and moves the lever to isolate the mechanism.
It never records or exports those fixture states as robot demonstrations.
"""

import argparse
import json
import os
import traceback
from isaaclab.app import AppLauncher

parser = argparse.ArgumentParser(description=__doc__)
AppLauncher.add_app_launcher_args(parser)
args = parser.parse_args()
args.enable_cameras = True
app = AppLauncher(args).app

import gymnasium as gym
import torch
import isaaclab.utils.math as math_utils
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
import cyclo_lab  # noqa: F401
from cyclo_lab.manager_based.manipulation.make_toast.config.ffw_bg2 import bimanual_mdp


def main():
    task = "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v1"
    cfg = parse_env_cfg(task, device=args.device, num_envs=2)
    cfg.terminations = None
    cfg.sim.render_interval = 100000
    env = gym.make(task, cfg=cfg).unwrapped
    try:
        env.reset()
        cycle = env.toast_cycle
        toaster = env.scene["toaster"]
        action = torch.zeros((2, 15), device=env.device)
        action[:, 12:] = 1

        def step(n):
            for _ in range(n):
                env.step(action)

        def bread_local():
            return torch.stack([math_utils.subtract_frame_transforms(
                toaster.data.root_pos_w, toaster.data.root_quat_w,
                env.scene[f"bread_{i}"].data.root_pos_w,
                env.scene[f"bread_{i}"].data.root_quat_w,
            )[0] for i in range(2)], dim=1)

        # Pressing an empty toaster must not start the loaded-bread task cycle.
        q = toaster.data.joint_pos.clone()
        q[:, cycle.lever_id] = .0642
        toaster.write_joint_state_to_sim(q, torch.zeros_like(q))
        step(5)
        assert (cycle.phase == 0).all()
        env.reset()
        for i, y in enumerate((-.026455, .0253585)):
            bread = env.scene[f"bread_{i}"]
            state = bread.data.default_root_state.clone()
            local = torch.tensor([-.003, y + .0056, .14], device=env.device).expand(2, -1)
            rotation = torch.tensor([.70710678, .70710678, 0, 0], device=env.device).expand(2, -1)
            pos, quat = math_utils.combine_frame_transforms(
                toaster.data.root_pos_w, toaster.data.root_quat_w, local, rotation,
            )
            state[:, :3], state[:, 3:7], state[:, 7:] = pos, quat, 0
            bread.write_root_state_to_sim(state)
        step(30)
        raised = bread_local()[:, :, 2].clone()
        assert (raised > .05).all() and (raised < .075).all(), raised
        assert bimanual_mdp.bread_in_slot(env, "bread_0", 0).all()
        assert bimanual_mdp.bread_in_slot(env, "bread_1", 1).all()
        exposed = (raised + .05778116 - .08023291) / .11606213
        assert (exposed > .25).all() and (exposed < .42).all(), exposed
        # Only environment zero is pressed; environment one must stay loaded.
        q = toaster.data.joint_pos[:1].clone()
        q[:, cycle.lever_id] = .0642
        toaster.write_joint_state_to_sim(q, torch.zeros_like(q), env_ids=torch.tensor([0], device=env.device))
        trace = []
        saved = None
        for i in range(250):
            step(1)
            local = bread_local()
            trace.append({"time": (i + 1) * env.step_dt, "phase": cycle.phase.tolist(),
                          "bread_center_z": local[:, :, 2].tolist(),
                          "carriage": toaster.data.joint_pos[:, cycle.carriage_id].tolist(),
                          "heating_time": cycle.heating_time.tolist()})
            if cycle.phase[0] == 2:
                # Both physical bread tops must be completely below the rim.
                assert (local[0, :, 2] + .05778116 < .08023291 + .002).all(), local
                if saved is None and cycle.heating_time[0] >= 1:
                    saved = env.scene.get_state(is_relative=True)
                    saved["toast_cycle"] = cycle.state()
            if cycle.phase[0] == 4:
                step(10)
                break
        if cycle.phase.tolist() != [4, 0]:
            print("CYCLE TRACE", json.dumps(trace), flush=True)
        assert cycle.phase.tolist() == [4, 0], (cycle.phase, trace[-1])
        assert abs(float(cycle.heating_time[0]) - 5) < .045
        final = bread_local()[:, :, 2]
        torch.testing.assert_close(final, raised, atol=.006, rtol=0)
        assert saved is not None
        env.reset_to(saved, None, is_relative=True)
        assert cycle.phase.tolist() == [2, 0]
        assert 1 <= float(cycle.heating_time[0]) < 1.1
        step(160)
        assert cycle.phase.tolist() == [4, 0]
        torch.testing.assert_close(bread_local()[:, :, 2], raised, atol=.006, rtol=0)
        env.reset(env_ids=torch.tensor([0], device=env.device))
        assert cycle.phase.tolist() == [0, 0]
        print("PASS TOAST CYCLE", json.dumps({"exposed_fraction": exposed.tolist(),
              "raised_center_z_m": raised.tolist(), "final_center_z_m": final.tolist(),
              "heating_seconds": 5, "trace": trace}), flush=True)
    finally:
        env.close()


try:
    main()
except Exception:
    traceback.print_exc()
    import sys
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(1)
else:
    app.close()
