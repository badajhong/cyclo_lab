# cyclo_lab

[![IsaacSim](https://img.shields.io/badge/IsaacSim-5.1.0-silver.svg)](https://docs.isaacsim.omniverse.nvidia.com/latest/index.html)
[![Isaac Lab](https://img.shields.io/badge/IsaacLab-2.3.0-silver)](https://isaac-sim.github.io/IsaacLab/main/index.html)
[![Python](https://img.shields.io/badge/python-3.11-blue.svg)](https://docs.python.org/3/whatsnew/3.11.html)
[![Linux platform](https://img.shields.io/badge/platform-linux--64-orange.svg)](https://releases.ubuntu.com/22.04/)
[![License](https://img.shields.io/badge/license-Apache2.0-yellow.svg)](https://opensource.org/license/apache-2-0)

https://github.com/user-attachments/assets/28347b4b-f90c-4a4f-8916-621f917d86cb

## Overview

**cyclo_lab** is a research-oriented repository based on [Isaac Lab](https://isaac-sim.github.io/IsaacLab), designed to enable reinforcement learning (RL) and imitation learning (IL) experiments using Robotis robots in simulation.
This project provides simulation environments, configuration tools, and task definitions tailored for Robotis hardware, leveraging NVIDIA Isaac Sim’s powerful GPU-accelerated physics engine and Isaac Lab’s modular RL pipeline.

> [!IMPORTANT]
> This repository currently depends on **IsaacLab v2.2.0** or higher.
>

## Installation (Docker)

Docker installation provides a consistent environment with all dependencies pre-installed.

**Prerequisites:**
- Docker and Docker Compose installed
- NVIDIA Container Toolkit installed
- NVIDIA GPU with appropriate drivers
- On ARM64, an NVIDIA DGX Spark with DGX OS 7 for Isaac Sim 5.1.0

**Steps:**

1. Clone cyclo_lab and initialize its direct submodules:

   ```bash
   git clone https://github.com/ROBOTIS-GIT/cyclo_lab.git
   cd cyclo_lab
   git submodule update --init
   ```

2. Build and start the Docker container:

   ```bash
   ./docker/container.sh start
   ```

   On ARM64, this creates `cyclo_lab-arm64` from the native ARM64 Isaac Sim 5.1.0 image. The image and cache volumes are separate from the x86-64 container. The first build downloads the ARM64 base image and installs Cyclo Lab dependencies.

   The original x86-64 image (`robotis/cyclo-lab:latest`) and container (`cyclo_lab`) remain available. On an ARM64 host, select them explicitly with `CYCLOLAB_DOCKER_ARCH=amd64 ./docker/container.sh logs` or another container command. The x86-64 container cannot run natively on an ARM64 host.

3. Enter the container:

   ```bash
   ./docker/container.sh enter
   ```

**Docker Commands:**
- `./docker/container.sh start` - Build and start the container
- `./docker/container.sh recreate` - Recreate the container from the current image
- `./docker/container.sh enter` - Enter the running container
- `./docker/container.sh stop` - Stop the container
- `./docker/container.sh logs` - View container logs
- `./docker/container.sh clean` - Remove container and image

**What's included in the Docker image:**
- Isaac Sim 5.1.0
- Isaac Lab v2.3.0 (from third_party submodule)
- zenoh_ros2_sdk 0.1.8 (from third_party submodule)
- IsaacLab-Arena integration (from third_party submodule)
- LeRobot 0.3.3 (in separate virtual environment at `~/lerobot_env`)
- All required dependencies and configurations

## Try examples

### Sim2Sim
<details>
<summary>Reinforcement learning</summary>

**OMY Reach Task**

```bash
# Train
python scripts/reinforcement_learning/rsl_rl/train.py --task Cyclo-Reach-OMY-v0 --num_envs=512 --headless

# Play
python scripts/reinforcement_learning/rsl_rl/play.py --task Cyclo-Reach-OMY-v0 --num_envs=16
```

**OMY Lift Task**

```bash
# Train
python scripts/reinforcement_learning/rsl_rl/train.py --task Cyclo-Lift-Cube-OMY-v0 --num_envs=512 --headless

# Play
python scripts/reinforcement_learning/rsl_rl/play.py --task Cyclo-Lift-Cube-OMY-v0 --num_envs=16
```

**OMY Open drawer Task**

```bash
# Train
python scripts/reinforcement_learning/rsl_rl/train.py --task Cyclo-Open-Drawer-OMY-v0 --num_envs=512 --headless

# Play
python scripts/reinforcement_learning/rsl_rl/play.py --task Cyclo-Open-Drawer-OMY-v0 --num_envs=16
```

**FFW-BG2 reach Task**

```bash
# Train
python scripts/reinforcement_learning/rsl_rl/train.py --task Cyclo-Reach-FFW-BG2-v0 --num_envs=512 --headless

# Play
python scripts/reinforcement_learning/rsl_rl/play.py --task Cyclo-Reach-FFW-BG2-v0 --num_envs=16
```

</details>

<details>
<summary>Imitation learning</summary>

>
> To control the robot while **recording demonstrations**, use `--teleop_device keyboard` in the `record_demos.py` command. Do not add `--keyboard`; that script does not accept it. Click the Isaac Sim window so it receives key presses. For the FFW-BG2 IK task, these keys control the right arm and gripper, not the mobile base.
>
> ```
> Key bindings:
> =========================== =========================
> Command                     Key
> =========================== =========================
> Toggle gripper (open/close) K      
> Reset recording             R
> Move arm along x-axis       W / S   
> Move arm along y-axis       A / D
> Move arm along z-axis       Q / E
> Rotate arm along x-axis     Z / X
> Rotate arm along y-axis     T / G
> Rotate arm along z-axis     C / V
> =========================== =========================
> ```

**OMY Stack Task** (Stack the blocks in the following order: blue → red → green.)

```bash
# Teleop and record
python scripts/imitation_learning/isaaclab_recorder/record_demos.py --task Cyclo-Stack-Cube-OMY-IK-Rel-v0 --teleop_device keyboard --dataset_file ./datasets/dataset.hdf5 --num_demos 10

# Annotate
python scripts/imitation_learning/isaaclab_mimic/annotate_demos.py --device cuda --task Cyclo-Stack-Cube-OMY-IK-Rel-Mimic-v0 --auto --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5 --headless

# Mimic data
python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
--device cuda --num_envs 100 --generation_num_trials 1000 \
--input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5 --headless

# Train
python scripts/imitation_learning/robomimic/train.py \
--task Cyclo-Stack-Cube-OMY-IK-Rel-v0 --algo bc \
--dataset ./datasets/generated_dataset.hdf5

# Play
python scripts/imitation_learning/robomimic/play.py \
--device cuda --task Cyclo-Stack-Cube-OMY-IK-Rel-v0 --num_rollouts 50 \
--checkpoint /PATH/TO/desired_model_checkpoint.pth
```

**FFW-BG2 Pick and Place Task** (Move the red stick into the basket.)

```bash
# Teleop and record
python scripts/imitation_learning/isaaclab_recorder/record_demos.py --task Cyclo-PickPlace-FFW-BG2-IK-Rel-v0 --teleop_device keyboard --dataset_file ./datasets/dataset.hdf5 --num_demos 10 --enable_cameras

# Annotate
python scripts/imitation_learning/isaaclab_mimic/annotate_demos.py --device cuda --task Cyclo-PickPlace-FFW-BG2-Mimic-v0 --input_file ./datasets/dataset.hdf5 --output_file ./datasets/annotated_dataset.hdf5 --enable_cameras

# Mimic data
python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
--device cuda --num_envs 20 --generation_num_trials 300 \
--input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5 --enable_cameras --headless

# Train
python scripts/imitation_learning/robomimic/train.py \
--task Cyclo-PickPlace-FFW-BG2-IK-Rel-v0 --algo bc \
--dataset ./datasets/generated_dataset.hdf5

# Play
python scripts/imitation_learning/robomimic/play.py \
--device cuda --task Cyclo-PickPlace-FFW-BG2-IK-Rel-v0  --num_rollouts 50 \
--checkpoint /PATH/TO/desired_model_checkpoint.pth --enable_cameras
```
</details>

### FFW-BG2 Make Toast (RoboDojo objects)

The commands in this section use the original seven-action keyboard task.
For automatic two-arm + lift demonstrations and the revised motion, use
[the bimanual workflow below](#ffw-bg2-make-toast-automatic-demonstrations-with-both-arms-and-body-lift).

This separate task keeps Cyclo Lab's FFW-BG2 robot, right-arm relative IK,
gripper, packing table, and wrist/head cameras. It uses only RoboDojo's toaster,
bread, and bread rack assets. Packing crates, boxes, and the tabletop basket are
removed from the Make Toast table, including their colliders. Only toast-related
objects remain on the table. The original PickPlace task remains available.
Both arms start in a mirrored raised standby pose, and the toaster lever faces
the robot. The existing keyboard task controls the right arm and gripper; it
holds the left arm and body lift at their starting positions. The robot's lift
joint spans -0.5 to 0 m, with 0 at its highest position.

Download the pinned assets once, from the repository root (inside Docker is also fine):

```bash
python3 scripts/tools/download_make_toast_assets.py
```

Record in this order: pick `bread_0` (rack slice with the largest world X), insert
it into one toaster slot and release; pick the adjacent `bread_1`, insert it into
the other slot and release; press the lever down; open the gripper and return
the right arm to its initial pose. Leave `bread_2` and `bread_3` upright on the rack.
This ordering is required for the automatic Mimic annotations. Use fresh Make
Toast demonstrations; existing PickPlace demonstrations are incompatible.

```bash
# Teleop and record successful Make Toast demonstrations
python scripts/imitation_learning/isaaclab_recorder/record_demos.py \
  --task Cyclo-Make-Toast-FFW-BG2-IK-Rel-v0 --teleop_device keyboard \
  --dataset_file ./datasets/make_toast/dataset.hdf5 --num_demos 10 --enable_cameras

# Automatically annotate grasp / insert / grasp / insert / lever / return
python scripts/imitation_learning/isaaclab_mimic/annotate_demos.py \
  --device cuda --task Cyclo-Make-Toast-FFW-BG2-Mimic-v0 --auto \
  --input_file ./datasets/make_toast/dataset.hdf5 \
  --output_file ./datasets/make_toast/annotated_dataset.hdf5 --enable_cameras --headless

# Mimic: 300 generation attempts, retaining successful episodes only
python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
  --device cuda --task Cyclo-Make-Toast-FFW-BG2-Mimic-v0 \
  --num_envs 20 --generation_num_trials 300 \
  --input_file ./datasets/make_toast/annotated_dataset.hdf5 \
  --output_file ./datasets/make_toast/generated_dataset.hdf5 --enable_cameras --headless

# Train using the existing FFW-BG2 image BC configuration
python scripts/imitation_learning/robomimic/train.py \
  --task Cyclo-Make-Toast-FFW-BG2-IK-Rel-v0 --algo bc \
  --dataset ./datasets/make_toast/generated_dataset.hdf5

# Play
python scripts/imitation_learning/robomimic/play.py \
  --device cuda --task Cyclo-Make-Toast-FFW-BG2-IK-Rel-v0 --num_rollouts 50 \
  --checkpoint /PATH/TO/desired_model_checkpoint.pth --horizon 1800 --enable_cameras

# Check asset loading, physics, success conditions, and Mimic conversions
python scripts/tools/check_make_toast.py --headless --num_envs 2
# Also check one BC optimizer update and policy inference (no checkpoint saved)
python scripts/tools/check_make_toast.py --headless --num_envs 2 --check_bc
```

The toaster has a physical lever joint. RoboDojo's original asset does **not**
include timed heating or a mechanism that raises the bread automatically.
Successful data generation and policy learning depend on successful source
demonstrations; 300 attempts do not guarantee 300 successful episodes.
See [asset sources and attribution](source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/ASSET_SOURCES.md).

### FFW-BG2 Make Toast: automatic demonstrations with both arms and body lift

The separate `Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2` task has 15 actions:
left/right wrist deltas (6 each), two continuous gripper openings, and one body
lift command. It observes both wrist cameras and the head camera. The existing
seven-action keyboard task and its datasets remain separate.

No recorded demonstration or RoboDojo motion download is required for this option.
The scene-state expert reads the actual simulated bread/toaster poses and
finger contact frames. It picks with the right hand, transfers to the left,
places each slice in a separate slot, presses the lever by robot contact, and
returns both hands to standby while the toaster completes its cycle. The rack is twice as wide along its groove axis
(50 mm slice spacing), and sits closer to the robot, to accommodate FFW-BG2's
larger fingers. Bread and toaster geometry retain their original size.
The toaster is closer to the robot at `(0.55, 0.23)` m with a 150° yaw, presenting
the lever toward the left arm. Insertion and pressing use its actual rotated frame.
The expert uses simulator object poses to label actions; the BC policy consumes
joint/lift observations and three RGB images. This is simulation calibration:
fingertip friction is explicitly set to 1.0 static / 0.8 dynamic, and the four
gripper linkage joints are driven together. These are not measured hardware values.
At standby, a null-space posture controller restores each arm's redundant joint
configuration so the next pickup does not accumulate wrist joint-limit errors.
The current calibration starts both arms in a mirrored pose with fingertips
about 170 mm higher than the original pose. The expert defaults to `--motion_speed 2.0`:
twice the original Cartesian speed with shorter dwell times. It turns the receiving
hand and the held bread above the central workspace before approaching the toaster.
Shell and lever contact sensors record robot contact separately; handover/turning
collisions with the toaster and a missing physical lever stroke reject the episode.
The imported toaster authors a 20 mm minimum convex collision thickness on thin
decorative faces. The bimanual spawner uses 1 mm to keep those collision hulls
from covering the lever. It preserves every mesh and collision pair. The lever
press uses a 60° outward tilt and 90° tool twist, stops at measured 95% travel,
then clears the lever edge before raising the hand.

Only successful, physically executed full episodes are exported. Diagnostic
checks and failed attempts never become training demonstrations. Reports and
optional camera snapshots are written under `logs/make_toast_expert/`.
Successful expert episodes also contain synchronized Mimic pose metadata;
for the serving task, run `annotate_make_toast_serving.py` below before Mimic
generation. It adds the extraction/placement boundaries from the saved states
without changing the original recording. Earlier whole-episode `-v0`/`-v1`
datasets can be passed directly to their matching Mimic tasks.

On the ARM64 DGX Spark / Isaac Sim 5.1 setup, the previous calibration passed a
complete 4,265-frame expert demonstration, one successful 4,267-frame Mimic
generation attempt, and an image BC optimizer update plus 15-action inference
using the recorded data. This checks data and pipeline compatibility; a trained
policy's Make Toast success rate has not been evaluated.
The earlier `dataset.hdf5` and `generated_dataset.hdf5` recordings use the previous
pose, path, and collision calibration. `dataset_v2.hdf5` and
`generated_dataset_v2.hdf5` retain their `Bimanual-…-v0` task and passive toaster.
The new pop-up mechanism uses `Bimanual-…-v1` and fresh `dataset_v3.hdf5` recordings;
its additional carriage joint makes old recordings incompatible with the new task.

Version 2 collected one successful full demonstration with 2,748 frames at 25 Hz
(109.92 simulated seconds, previously 170.60 seconds). Its recorded shell contact
force remained zero throughout, and the lever travelled 78 mm. Both environment
clones passed raised-pose, camera, action/pose, lift, and state-restoration checks.
The 2x setting applies to movement limits and dwell time; total duration also
includes the added clearance path and physical gripper settling.
The review video shows head / left wrist / right wrist cameras at the recorded
25 Hz: `logs/make_toast_expert/preview_v2.mp4`.
One Mimic generation attempt also succeeded, saving a 2,750-frame episode with
15 actions, all three RGB streams, and the lever fully down to
`datasets/make_toast_bimanual/generated_dataset_v2.hdf5`.
See `logs/make_toast_expert/validation_summary_v2.json` for the local validation results.

Calibration version 3 adds two physical support trays inside the original
RoboDojo toaster, driven by a 45 mm prismatic carriage. Approximately two thirds
of each slice rests inside the slots before pressing. A physical lever press
latches the toaster, lowers both slices completely below the rim, holds them
for **5 simulation seconds**, then raises the bread and returns the lever.
The bread moves through contact with the trays; it is not teleported or attached
to them. Success requires completion of this cycle, bread still in both slots,
and both hands back at standby. This is a custom Cyclo Lab extension; the
upstream RoboDojo toaster has no timer or pop-up behavior.
The expert removes the fixed five-second recovery pauses after each slice,
uses a 10 mm capture tolerance for the receiving hand, shortens gripper dwells,
prepares the second pickup while the left arm returns, and closes the empty
hand during the lever approach. Timer/latch state is included in recordings
and restored along with the physical state.

The version-3 full expert recording passed with 2,159 frames at 25 Hz
(86.36 simulation seconds, compared with version 2's 109.92 s), including
the added heating/pop-up cycle. Recorded visual mesh vertices confirmed
32–34% of the bread remained exposed when loaded and after pop-up. Throughout
the 125-frame (5.00 s) heating interval, both complete bread meshes remained
below the rim, with at least 7.3 mm clearance. The review video is
`logs/make_toast_expert/preview_v3.mp4`; the cropped lever/heating/pop-up clip is
`logs/make_toast_expert/toaster_cycle_v3.mp4`.
One Mimic attempt also succeeded, exporting 2,161 frames to
`datasets/make_toast_bimanual/generated_dataset_v3.hdf5`. Its five-second
heating interval and complete bread-mesh clearance passed the same checks.
See `logs/make_toast_expert/validation_summary_v3.json` and
`logs/make_toast_expert/toast_cycle_check_v3.json` for recorded results.

Calibration version 4 / `Bimanual-…-v2` adds a serving stage after pop-up.
It imports RoboDojo's `Rigid/plate/00002` mesh, scales it by `(1.5, 1.5, 0.22)`
to approximately 194 × 194 × 8 mm, and applies a dark blue glaze. The original
asset is a small concave dish; flattening its height and changing its very bright
rendered material are explicit Cyclo Lab adaptations. Bread/toaster sizes and
the FFW-BG2 robot are unchanged. The dynamic plate sits at `(0.43, 0.035)` m,
fully on the table, with approximately 30 mm separation from the bread rack.

The serving toaster pops 15 mm higher than `-v1` to provide fingertip clearance
while retaining the initial two-thirds loading depth and five-second heating.
The left hand pinches each popped slice near its top, extracts it upward, and
turns it above the plate. It lowers the nearest bread edge to approximately
50 mm above the support surface, fully opens the fingers, and waits until the
slice settles flat before withdrawing.
The second slice is placed over the first on the same small plate. Body lift
remains part of the 15-action controller during low placement. Final success
requires both complete bread shapes inside the plate, flat and stable,
plus completed toasting and both hands returned to standby. Robot contacts
with the plate and toaster housing are monitored separately and reject impacts
above 2 N. Previous `-v0` and `-v1` tasks/datasets retain their earlier goals.
Use fresh `dataset_v4.hdf5` recordings with `-v2` for the complete serving task.

Version 4 collected a successful full 3,482-frame demonstration at 25 Hz
(139.28 simulation seconds), including pickup, handover, two-slot loading,
five-second toasting, pop-up, and serving both slices. Independent checks of all
46,487 visual vertices per slice confirmed complete plate containment. The
stack height difference is 11.3 mm; movement during the last half-second is
below 0.02 mm. The plate moved about 1 mm, and robot-to-plate and serving-phase
robot-to-shell contact forces remained zero. Body lift lowered by about 58 mm
during low placement. Two environment clones passed plate material, pop-up
limit, camera, 15-action, pose conversion, lift, and state-restoration checks.
Videos: `logs/make_toast_expert/preview_v4.mp4` (all three cameras) and
`logs/make_toast_expert/serving_v4.mp4` (head-camera serving detail).

The full expert recording is validated. Serving Mimic remains experimental:
full RGB replay has not passed the complete success check, although isolated
serving diagnostics succeeded. Use the successful expert `dataset_v4.hdf5`
directly for BC training; restored diagnostic episodes are not training data.

```bash
# Check the automatic expert without writing a training dataset
python scripts/imitation_learning/isaaclab_recorder/generate_make_toast_demos.py \
  --check_only --max_attempts 1 --snapshots --headless

# Check physical loading depth, full lowering, five-second heating, pop-up, and timer restoration
python scripts/tools/check_toast_cycle.py --headless

# Check action/pose conversions, cameras, lift, and one image BC update
python scripts/tools/check_make_toast_bimanual.py --headless --num_envs 2 --check_bc --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2

# Automatically collect successful demonstrations; choose a fresh output path
python scripts/imitation_learning/isaaclab_recorder/generate_make_toast_demos.py \
  --dataset_file ./datasets/make_toast_bimanual/dataset_v4.hdf5 \
  --motion_speed 2.0 --num_demos 10 --max_attempts 30 --headless

# Watch the collected 15-action demonstrations in Isaac Sim (B: pause, N: resume)
python scripts/imitation_learning/isaaclab_recorder/replay_demos.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 \
  --dataset_file ./datasets/make_toast_bimanual/dataset_v4.hdf5 \
  --num_envs 1 --enable_cameras

# Save an MP4 without opening a GUI: front overview above Head / Left wrist / Right wrist
# Default overview faces the robot/table center directly with closer framing.
# Uses saved states and original RGB, rather than running another physics rollout.
# Choose a fresh output filename when exporting again.
python scripts/imitation_learning/isaaclab_recorder/replay_demos.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 \
  --dataset_file ./datasets/make_toast_bimanual/dataset_v4.hdf5 \
  --num_envs 1 --headless --enable_cameras \
  --video_file ./logs/make_toast_expert/replay_v4_front_center.mp4

# Experimental Mimic: annotate saved serving boundaries (not required for BC below)
python scripts/imitation_learning/isaaclab_mimic/annotate_make_toast_serving.py \
  --input_file ./datasets/make_toast_bimanual/dataset_v4.hdf5 \
  --output_file ./datasets/make_toast_bimanual/annotated_dataset_v4.hdf5

# Generate synchronized trajectories: 300 attempts, save successes only
python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
  --device cuda --task Cyclo-Make-Toast-FFW-BG2-Bimanual-Mimic-v2 \
  --num_envs 20 --generation_num_trials 300 \
  --input_file ./datasets/make_toast_bimanual/annotated_dataset_v4.hdf5 \
  --output_file ./datasets/make_toast_bimanual/generated_dataset_v4.hdf5 --enable_cameras --headless

python scripts/imitation_learning/robomimic/train.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 --algo bc \
  --dataset ./datasets/make_toast_bimanual/dataset_v4.hdf5

python scripts/imitation_learning/robomimic/play.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 --num_rollouts 50 \
  --checkpoint /PATH/TO/desired_model_checkpoint.pth --horizon 6000 --enable_cameras

python scripts/tools/check_make_toast_bimanual.py --headless --num_envs 2 --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2
```

For `-v2`, Mimic keeps the loading/toasting motion synchronized, re-anchors each
extraction to the actual popped bread pose, and re-anchors serving to the plate.
Extraction stays vertical after closing, and placement compensates the measured
grasp difference. Mimic adds 20 mm of release clearance near the plate to keep
the larger finger-opening sweep from pushing the dish, plus a calibrated
20 mm longitudinal offset to center the slices after opening. Short, synchronized
transitions start from the actual hand poses. The lift retains its calibrated
trajectory. The annotation command copies the
successful source without regenerating its actions, images, or measured poses.
The fixed rack/toaster layout remains calibrated; arbitrary scene layouts have
not been validated. Earlier `-v0`/`-v1` Mimic tasks retain whole-episode replay.
Use fresh 15-action demonstrations and checkpoints for this task.

For experimental cross-robot motion calibration, `download_robodojo_make_toast_motion.py`
extracts only motion arrays from a pinned RoboDojo simulation HDF5 using range
reads. `retarget_robodojo_make_toast.py --mode fit` fits contact landmarks and
`--mode check` physically tests the fit. These source files contain ARX wrist
poses but no object-pose stream; the fit is an estimate, and never qualifies as
FFW-BG2 training data until successful physical replay.

### Sim2Real
>
> **Important**
>
> OMY Hardware Setup:
> To run Sim2Real with the real OMY robot, you need to bring up the robot.
>
> This can be done using ROBOTIS’s [open_manipulator repository](https://github.com/ROBOTIS-GIT/open_manipulator.git).
> 
> AI WORKER Hardware Setup:
> To run Sim2Real with the real AI WORKER robot, you need to bring up the robot.
>
> This can be done using ROBOTIS’s [ai_worker repository](https://github.com/ROBOTIS-GIT/ai_worker.git).
>
> The training and inference of the collected dataset should be carried out using physical_ai_tools.
> This can be done using ROBOTIS’s [physical_ai_tools](https://github.com/ROBOTIS-GIT/physical_ai_tools)
>

<details>
<summary>Bringup</summary>

**ROBOTIS HAND VR Teleoperation in Isaac Sim**
[Introduction YouTube](https://www.youtube.com/watch?v=4D8if2lZY7o)

https://github.com/user-attachments/assets/67267195-db02-4dd9-83cf-00830b4bc13f

Launch AI Worker SH5

```bash
# Run SH5 on a ground plane through the Zenoh ROS2 bridge.
python scripts/sim2real/bringup.py \
    --task Cyclo-Bringup-FFW-SH5-v0 \
    --bridge ffw_sh5

# Run SH5 in NVIDIA Simple Warehouse with the operator camera view.
python scripts/sim2real/bringup.py \
    --task Cyclo-Bringup-Simple-Warehouse-FFW-SH5-v0 \
    --bridge ffw_sh5 \
    --camera_view operator \
    --headless
```

Launch FFW SG2 in the Robotis showroom

```bash
# Publish SG2 state and camera topics from the empty 5 m showroom.
python scripts/sim2real/bringup.py \
    --task Cyclo-Real-Showroom-FFW-SG2-v0 \
    --bridge ffw_sg2 \
    --headless \
    --enable_cameras

# Run SG2 in the empty 5 m showroom with the six-panel operator view.
python scripts/sim2real/bringup.py \
    --task Cyclo-Real-Showroom-FFW-SG2-v0 \
    --bridge ffw_sg2 \
    --headless \
    --camera_view operator
```

Launch FFW SG2 in the IsaacLab-Arena Galileo pick-and-place environment

```bash
# Run the upstream Galileo scene with Cyclo Lab's 22D SG2 mobile action.
python scripts/sim2real/bringup.py \
    --task Cyclo-Arena-Galileo-Pick-Place-FFW-SG2-v0 \
    --bridge ffw_sg2 \
    --enable_cameras \
    --headless
```

</details>

<details>
<summary>Reinforcement learning</summary>

**OMY Reach Task**
[Introduction YouTube](https://www.youtube.com/watch?v=pSY0Gb5b5kI)

https://github.com/user-attachments/assets/6c27bdb1-3a6b-4686-a546-8f14f01e4abe

Run Sim2Real Reach Policy on OMY

```bash
# Train
python scripts/reinforcement_learning/rsl_rl/train.py --task Cyclo-Reach-OMY-v0 --num_envs=512 --headless

# Play (You must run rsl_rl play in order to generate the policy file.)
python scripts/reinforcement_learning/rsl_rl/play.py --task Cyclo-Reach-OMY-v0 --num_envs=16

# Sim2Real
python scripts/sim2real/reinforcement_learning/inference/OMY/reach/run_omy_reach.py --model_dir=<2025-07-10_08-47-09>
```

Replace <2025-07-10_08-47-09> with the actual timestamp folder name under:
```bash
logs/rsl_rl/reach_omy/
```
</details>

<details>
<summary>Imitation learning</summary>

**OMY Pick and Place Task**

**Sim2Sim**

https://github.com/user-attachments/assets/a6e75e80-203f-47d1-974b-d4c5435c15bc

**Sim2Real**

https://github.com/user-attachments/assets/8ec9d245-f8e0-4bcc-b683-0ea2864de495


* Teleop and record demos
```bash
python scripts/sim2real/imitation_learning/recorder/record_demos.py --task=Cyclo-Real-Pick-Place-Bottle-OMY-v0 --robot_type OMY --dataset_file ./datasets/omy_pick_place_task.hdf5 --num_demos 10 --enable_cameras

```

* Mimic generate dataset

```bash
# Data convert ee_pose action from joint action
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py --robot_type OMY --input_file ./datasets/omy_pick_place_task.hdf5 --output_file ./datasets/processed_omy_pick_place_task.hdf5 --action_type ik

# Annotate dataset
python scripts/sim2real/imitation_learning/mimic/annotate_demos.py --task Cyclo-Real-Mimic-Pick-Place-Bottle-OMY-v0 --auto --input_file ./datasets/processed_omy_pick_place_task.hdf5 --output_file ./datasets/annotated_dataset.hdf5 --enable_cameras --headless

# Generate dataset
python scripts/sim2real/imitation_learning/mimic/generate_dataset.py --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-Bottle-OMY-v0 --generation_num_trials 500 --input_file ./datasets/annotated_dataset.hdf5 --output_file ./datasets/generated_dataset.hdf5 --enable_cameras --headless

# Data convert joint action from ee_pose action
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py --robot_type OMY --input_file ./datasets/generated_dataset.hdf5 --output_file ./datasets/processed_generated_dataset.hdf5 --action_type joint

```

* Data convert lerobot dataset from IsaacLab hdf dataset
```bash
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
    --task=Cyclo-Real-Pick-Place-Bottle-OMY-v0 \
    --robot_type OMY \
    --dataset_file ./datasets/processed_generated_dataset.hdf5

```

* Inference in simulation
```bash
python scripts/sim2real/imitation_learning/inference/inference_demos.py --task Cyclo-Real-Pick-Place-Bottle-OMY-v0 --robot_type OMY --enable_cameras

```

**FFW SG2 Pick and Place Task**

https://github.com/user-attachments/assets/cdb3afca-f0db-4fb6-bf17-e361f3aa254b

* Teleop and record demos
```bash
python scripts/sim2real/imitation_learning/recorder/record_demos.py --task=Cyclo-Real-Pick-Place-FFW-SG2-v0 --robot_type FFW_SG2 --dataset_file ./datasets/ffw_sg2_raw.hdf5 --num_demos 4 --enable_cameras

```

* Mimic generate dataset
```bash

# Data convert ee_pose action from joint action
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py --robot_type FFW_SG2 --input_file ./datasets/ffw_sg2_raw.hdf5 --output_file ./datasets/ffw_sg2_ik.hdf5 --action_type ik

# Annotate dataset
python scripts/sim2real/imitation_learning/mimic/annotate_demos.py --task Cyclo-Real-Mimic-Pick-Place-FFW-SG2-v0 --auto --input_file ./datasets/ffw_sg2_ik.hdf5 --output_file ./datasets/ffw_sg2_annotate.hdf5 --enable_cameras --headless

# Generate dataset
python scripts/sim2real/imitation_learning/mimic/generate_dataset.py --device cuda --num_envs 10 --task Cyclo-Real-Mimic-Pick-Place-FFW-SG2-v0 --generation_num_trials 500 --input_file ./datasets/ffw_sg2_annotate.hdf5 --output_file ./datasets/ffw_sg2_generate.hdf5 --enable_cameras --headless

# Data convert joint action from ee_pose action
python scripts/sim2real/imitation_learning/mimic/action_data_converter.py --robot_type FFW_SG2 --input_file ./datasets/ffw_sg2_generate.hdf5 --output_file ./datasets/ffw_sg2_final.hdf5 --action_type joint

```

* Data convert lerobot dataset from IsaacLab hdf dataset
```bash
lerobot-python scripts/sim2real/imitation_learning/data_converter/isaaclab2lerobot.py \
    --task=Cyclo-Real-Pick-Place-FFW-SG2-v0 \
    --robot_type FFW_SG2 \
    --dataset_file ./datasets/ffw_sg2_final.hdf5

```

* Inference in simulation
```bash
python scripts/sim2real/imitation_learning/inference/inference_demos.py --task Cyclo-Real-Pick-Place-FFW-SG2-v0  --robot_type FFW_SG2 --enable_cameras

```

## License

This repository is licensed under the **Apache 2.0 License**. See [LICENSE](LICENSE) for details.

### Third-party components

- **Isaac Lab**: BSD-3-Clause License, see [LICENSE-IsaacLab](LICENSE-IsaacLab)
