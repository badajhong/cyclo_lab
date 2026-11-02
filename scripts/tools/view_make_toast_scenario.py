"""View one fixed toast layout. R resets it; closing the window exits.

No expert, recording, asset download, or diagnostic object disturbance.
Use --steps and --reset_every for a finite headless smoke check.
"""
import argparse


def main():
    from isaaclab.app import AppLauncher

    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--scenario_id", help="Omit for the original task layout")
    parser.add_argument("--scenario_manifest", help="Defaults to scenario_delivery/manifest.json")
    parser.add_argument("--steps", type=int, default=0, help="Exit after N steps; 0 keeps viewing")
    parser.add_argument("--reset_every", type=int, default=0, help="Reset every N steps; 0 disables")
    AppLauncher.add_app_launcher_args(parser)
    args = parser.parse_args()
    if args.steps < 0 or args.reset_every < 0:
        parser.error("Step counts must be nonnegative")
    args.enable_cameras = True
    app = AppLauncher(args).app
    env = None
    keyboard = subscription = input_interface = None
    try:
        import gymnasium as gym
        import torch
        import cyclo_lab  # noqa: F401 — registers tasks
        from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

        task = "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2"
        cfg = parse_env_cfg(task, device=args.device, num_envs=1)
        cfg.scenario_id = args.scenario_id
        cfg.scenario_manifest = args.scenario_manifest
        env = gym.make(task, cfg=cfg).unwrapped

        def reset():
            env.reset()
            # Compare immediate initial poses, not subsequent settling or task success.
            for name in ("toaster", "plate", "bread_shelf", *(f"bread_{i}" for i in range(4))):
                if name == "bread_shelf":
                    position, quaternion = env.scene[name].get_world_poses()
                else:
                    state = env.scene[name].data.root_state_w
                    position, quaternion = state[:, :3], state[:, 3:7]
                initial = getattr(env.cfg.scene, name).init_state
                target = torch.tensor([(*initial.pos, *initial.rot)], device=env.device)
                torch.testing.assert_close(position - env.scene.env_origins, target[:, :3], atol=5e-5, rtol=0)
                dot = (quaternion * target[:, 3:]).sum(-1).abs()
                torch.testing.assert_close(dot, torch.ones_like(dot), atol=5e-5, rtol=0)
            print("RESET_POSES_OK", args.scenario_id or "original", flush=True)

        reset()
        reset_requested = False
        if not args.headless:
            import carb.input
            import omni.appwindow
            input_interface = carb.input.acquire_input_interface()
            keyboard = omni.appwindow.get_default_app_window().get_keyboard()

            def on_keyboard(event, *unused):
                nonlocal reset_requested
                if event.type == carb.input.KeyboardEventType.KEY_PRESS and event.input == carb.input.KeyboardInput.R:
                    reset_requested = True
                return True

            subscription = input_interface.subscribe_to_keyboard_events(keyboard, on_keyboard)
        # Match the established layout hold: zero arm deltas, open grippers, lift zero.
        action = torch.zeros((1, 15), device=env.device)
        action[:, 12:] = 1
        step = 0
        with torch.inference_mode():
            while app.is_running() and (args.steps == 0 or step < args.steps):
                if reset_requested or (args.reset_every and step and step % args.reset_every == 0):
                    reset()
                    reset_requested = False
                _, _, terminated, truncated, _ = env.step(action)
                if bool((terminated | truncated).any()):
                    raise RuntimeError("Task automatically reset during viewing; inspect the environment")
                step += 1
        print(f"VIEW_COMPLETE steps={step}", flush=True)
    finally:
        if subscription is not None:
            input_interface.unsubscribe_to_keyboard_events(keyboard, subscription)
        if env is not None:
            env.close()
        app.close()


if __name__ == "__main__":
    main()
