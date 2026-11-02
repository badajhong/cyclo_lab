"""Headless presentation of saved states with a front camera and original RGB.

This is a dataset visualization, not another physics rollout. Recorded RGB is
pre-action, so overview frame i uses initial_state or post-action state i - 1.
"""

import json
import os
from pathlib import Path
import tempfile

import gymnasium as gym
import h5py
import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont
import torch

import isaaclab.sim as sim_utils
from isaaclab.sensors import CameraCfg


CAMERAS = (("head_cam", "HEAD"), ("left_wrist_cam", "LEFT WRIST"), ("right_wrist_cam", "RIGHT WRIST"))
WIDTH, HEIGHT = 1440, 1248
OVERVIEW_WIDTH, OVERVIEW_HEIGHT = 1152, 648
TILE = 480


def _load_state(group, device):
    return {name: (_load_state(value, device) if isinstance(value, h5py.Group)
                   else torch.as_tensor(value[()], device=device)) for name, value in group.items()}


def _frame(state, index):
    return {name: (_frame(value, index) if isinstance(value, dict) else value[index:index + 1])
            for name, value in state.items()}


def _font(size):
    for path in ("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
                 "/usr/share/fonts/truetype/liberation2/LiberationSans-Regular.ttf"):
        if Path(path).exists():
            return ImageFont.truetype(path, size)
    return ImageFont.load_default()


def _compose(overview, recorded, episode, frame, fps, font):
    canvas = Image.new("RGB", (WIDTH, HEIGHT), (18, 21, 28))
    draw = ImageDraw.Draw(canvas)
    draw.text((144, 10), "FRONT OVERVIEW", font=font, fill=(235, 239, 246))
    draw.text((900, 10), f"{episode}  |  {frame / fps:06.2f} s", font=font, fill=(190, 199, 214))
    canvas.paste(Image.fromarray(overview), (144, 40))
    for index, (rgb, (_, label)) in enumerate(zip(recorded, CAMERAS)):
        x = index * TILE
        draw.text((x + 12, 716), label, font=font, fill=(235, 239, 246))
        canvas.paste(Image.fromarray(rgb).resize((TILE, TILE), Image.Resampling.BILINEAR), (x, 752))
    return np.asarray(canvas)


def export_video(env_name, cfg, args, app):
    output = Path(args.video_file).resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    selected = args.select_episodes
    with h5py.File(args.dataset_file, "r") as dataset:
        names = list(dataset["data"])
        indices = selected or list(range(len(names)))
        if any(index < 0 or index >= len(names) for index in indices):
            raise ValueError(f"Episode index outside dataset range 0..{len(names) - 1}")
        for index in indices:
            episode = dataset[f"data/{names[index]}"]
            n = len(episode["actions"])
            if "states" not in episode or "initial_state" not in episode:
                raise ValueError("Video export requires recorded initial_state and states")
            for key, _ in CAMERAS:
                if f"obs/{key}" not in episode:
                    raise ValueError(f"Video export requires original obs/{key} RGB")
                images = episode[f"obs/{key}"]
                if len(images) != n or images.ndim != 4 or images.shape[-1] != 3 or images.dtype != np.uint8:
                    raise ValueError(f"Expected frame-aligned uint8 RGB for {key}")

        # Original RGB is read from the recording, so only the overview needs
        # a live render product. Task objects, robot, and materials stay intact.
        for key, _ in CAMERAS:
            if hasattr(cfg.scene, key):
                setattr(cfg.scene, key, None)
            if hasattr(cfg.observations.policy, key):
                setattr(cfg.observations.policy, key, None)
        cfg.scene.replay_overview = CameraCfg(
            prim_path="{ENV_REGEX_NS}/ReplayOverview",
            width=OVERVIEW_WIDTH, height=OVERVIEW_HEIGHT, update_period=0,
            data_types=["rgb"],
            spawn=sim_utils.PinholeCameraCfg(
                focal_length=20.0, horizontal_aperture=24.0,
                clipping_range=(.05, 100.0),
            ),
        )
        env = gym.make(env_name, cfg=cfg).unwrapped
        writer = None
        count = 0
        episodes = []
        fps = 1.0 / env.step_dt
        font = _font(23)
        try:
            env.reset()
            camera = env.scene["replay_overview"]
            origin = env.scene.env_origins[:1]
            camera.set_world_poses_from_view(
                torch.tensor([args.video_eye], device=env.device) + origin,
                torch.tensor([args.video_target], device=env.device) + origin,
            )
            # Warm up lighting and render products; no physics is stepped.
            for _ in range(12):
                env.sim.render()
            with tempfile.TemporaryDirectory(prefix=".replay-video-", dir=output.parent) as directory:
                staged = Path(directory) / "video.mp4"
                writer = imageio_ffmpeg.write_frames(
                    str(staged), (WIDTH, HEIGHT), fps=fps, codec="libx264",
                    pix_fmt_in="rgb24", pix_fmt_out="yuv420p", macro_block_size=1,
                    output_params=["-crf", "22", "-preset", "veryfast", "-threads", "4",
                                   "-movflags", "+faststart"],
                )
                writer.send(None)
                with torch.inference_mode():
                    for index in indices:
                        name = names[index]
                        episode = dataset[f"data/{name}"]
                        initial = _load_state(episode["initial_state"], env.device)
                        states = _load_state(episode["states"], env.device)
                        n = len(episode["actions"])
                        episode_frames = 0
                        # Batch HDF5 image reads without loading the entire RGB
                        # recording into GPU or CPU memory.
                        for start in range(0, n, 32):
                            rgb_batch = [episode[f"obs/{key}"][start:start + 32] for key, _ in CAMERAS]
                            for local in range(len(rgb_batch[0])):
                                if not app.is_running():
                                    raise InterruptedError("Video export interrupted")
                                i = start + local
                                state = initial if i == 0 else _frame(states, i - 1)
                                env.scene.reset_to(state, is_relative=True)
                                env.scene.write_data_to_sim()
                                env.sim.render()
                                camera.update(env.step_dt, force_recompute=True)
                                overview = camera.data.output["rgb"][0, :, :, :3].cpu().numpy()
                                frame = _compose(overview, [rgb[local] for rgb in rgb_batch], name, i, fps, font)
                                writer.send(frame)
                                count += 1
                                episode_frames += 1
                                if count == 1:
                                    Image.fromarray(frame).save(output.with_suffix(".preview.png"))
                                if count % 100 == 0:
                                    print(f"VIDEO: {count} frames ({count / fps:.2f} s)", flush=True)
                                if args.video_max_frames and count >= args.video_max_frames:
                                    break
                            if args.video_max_frames and count >= args.video_max_frames:
                                break
                        episodes.append({"episode": name, "frames": episode_frames})
                        if args.video_max_frames and count >= args.video_max_frames:
                            break
                writer.close()
                writer = None
                if not count or staged.stat().st_size < 1024:
                    raise RuntimeError("No valid video was encoded")
                os.link(staged, output)
            report = {
                "video": str(output), "dataset": str(Path(args.dataset_file).resolve()),
                "task": env_name, "mode": "saved_states_and_original_pre_action_RGB",
                "physics_rollout": False, "fps": fps, "frames": count,
                "duration_seconds": count / fps, "resolution": [WIDTH, HEIGHT],
                "overview_eye": args.video_eye, "overview_target": args.video_target,
                "bottom_views": [key for key, _ in CAMERAS], "episodes": episodes,
            }
            output.with_suffix(".json").write_text(json.dumps(report, indent=2) + "\n")
            print(f"Saved MP4: {output} ({count} frames, {count / fps:.2f} s)", flush=True)
        finally:
            if writer is not None:
                writer.close()
            env.close()
