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

### FFW-BG2 Make Toast: keyboard demonstrations

Click the Isaac Sim viewport and use the key bindings above to control the right arm and gripper.
This command uses the seven-action `v0` task. Its recordings are separate from the
15-action bimanual `v2` datasets below; the default keyboard interface does not control both arms and the lift.

### FFW-BG2 Make Toast: automatic demonstrations with both arms and body lift

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
python scripts/imitation_learning/isaaclab_recorder/replay_demos.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 \
  --dataset_file ./datasets/make_toast_bimanual/dataset_v4.hdf5 \
  --num_envs 1 --headless --enable_cameras \
  --video_file ./logs/make_toast_expert/replay_v4_front_center.mp4

# Teleop and record 
# TODO: add teleoperation code
python scripts/imitation_learning/isaaclab_recorder/record_demos.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 --teleop_device handtracking \
  --dataset_file ./datasets/make_toast_manual/dataset.hdf5 \
  --num_demos 10 --enable_cameras

# Annotate
python scripts/imitation_learning/isaaclab_mimic/annotate_make_toast_serving.py \
  --input_file ./datasets/make_toast_bimanual/dataset_v4.hdf5 \
  --output_file ./datasets/make_toast_bimanual/annotated_dataset_v4.hdf5

# Mimic data
python scripts/imitation_learning/isaaclab_mimic/generate_dataset.py \
  --device cuda --task Cyclo-Make-Toast-FFW-BG2-Bimanual-Mimic-v2 \
  --num_envs 20 --generation_num_trials 300 \
  --input_file ./datasets/make_toast_bimanual/annotated_dataset_v4.hdf5 \
  --output_file ./datasets/make_toast_bimanual/generated_dataset_v4.hdf5 --enable_cameras --headless

# Train
python scripts/imitation_learning/robomimic/train.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 --algo bc \
  --dataset ./datasets/make_toast_bimanual/dataset_v4.hdf5

# Play
python scripts/imitation_learning/robomimic/play.py \
  --task Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2 --num_rollouts 50 \
  --checkpoint /PATH/TO/desired_model_checkpoint.pth --horizon 6000 --enable_cameras
```

## License

This repository is licensed under the **Apache 2.0 License**. See [LICENSE](LICENSE) for details.

### Third-party components

- **Isaac Lab**: BSD-3-Clause License, see [LICENSE-IsaacLab](LICENSE-IsaacLab)
