# Toast Task — 100 Scenarios

## Overview

100 fixed object layouts for the humanoid toast task, with final poses for the toaster, plate, bread rack, and four bread slices. These are placement data, not recorded demonstrations. Loading an ID selects a saved layout; reset restores it rather than sampling another one.

## Scenario Design

Coordinates use the environment frame: **+X is robot forward, +Y is robot left, and +Z is up**. Left-to-right order follows **+Y → −Y**. Positions are in **metres (m)** and quaternions use **wxyz** order.

All six object orders are included:

| Object Order (Robot Left → Right) | Scenarios | operator_A | operator_B |
|---|---:|---:|---:|
| Toaster → Plate → Bread Rack | 17 | 9 | 8 |
| Toaster → Bread Rack → Plate | 17 | 8 | 9 |
| Plate → Toaster → Bread Rack | 17 | 9 | 8 |
| Plate → Bread Rack → Toaster | 17 | 8 | 9 |
| Bread Rack → Toaster → Plate | 16 | 8 | 8 |
| Bread Rack → Plate → Toaster | 16 | 8 | 8 |
| **Total** | **100** | **50** | **50** |

The nominal Y position, `Y_slot`, is **+0.40, 0, or −0.40 m**, according to the object's left, centre, or right slot.

| Object | Nominal position (X, Y, Z), m | X variation | Y variation | Reference yaw | Yaw variation |
|---|---|---|---|---|---|
| Toaster | (0.550, Y_slot, 1.081800) | ±0.015 m | ±0.015 m | 150° | ±5° |
| Plate | (0.490, Y_slot, 1.004223) | ±0.010 m | ±0.015 m | 0° | ±5° |
| Bread Rack | (0.460, Y_slot, 1.038200) | ±0.015 m | ±0.015 m | −90° | ±5° |

Yaw variations are offsets from each object's existing reference orientation, about the environment +Z axis. The table describes the generation ranges; use the final poses stored in `manifest.json`, not newly sampled offsets. Heights are rounded here for readability. Original heights and scales are preserved, as are the four bread slices' poses relative to the rack.

## Download and Setup

Clone the full branch, not only `scenario_delivery`:

```bash
git clone --branch toast-scenarios-100 --single-branch \
  https://github.com/kkkkkmj/cyclo_lab.git cyclo_lab_scenarios
cd cyclo_lab_scenarios
```

This branch includes its third-party sources as regular tracked directories. No additional submodule checkout is required for those directories; do not replace them with newer upstream versions.

The repository targets **Isaac Sim 5.1.0, Isaac Lab 2.3.0, and Python 3.11**. Docker setup requires Docker Compose, NVIDIA Container Toolkit, a supported NVIDIA GPU and drivers. GUI viewing also requires a working display and X11 forwarding (`DISPLAY` and `xauth`). See the original [Docker installation guide](../README.md#installation-docker) and [container script](../docker/container.sh).

Run the following from the cloned repository root on a machine without a conflicting Cyclo Lab container or Compose project:

```bash
python3 scripts/tools/download_make_toast_assets.py
./docker/container.sh build
./docker/container.sh start
```

The [asset downloader](../scripts/tools/download_make_toast_assets.py) downloads pinned toast objects and the plate, verifies downloaded sizes and SHA-256 checksums, and reuses files whose hashes match. It does not supply the robot or table. Keep the repository's robot USD files and make the NVIDIA packing-table asset and its references accessible to Isaac Sim. See [asset attribution](../source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/ASSET_SOURCES.md).

The [Dockerfile](../docker/Dockerfile.base) installs `cyclo_lab` in editable mode. The [Compose configuration](../docker/docker-compose.yaml) mounts the checkout into `/workspace/cyclo_lab`. The commands below use the default x86-64 container name `cyclo_lab`; ARM64 setup uses `cyclo_lab-arm64`.

**Existing installations:** reuse a compatible environment after verifying the mounted checkout and imported source. A separate clone does not isolate container names, the Compose project, images, or volumes. `start` can reuse an existing container; its name alone does not prove that it uses this checkout. Do not stop or replace another installation to follow this guide. Check first:

```bash
docker inspect cyclo_lab --format '{{json .Mounts}}'
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh -c \
  'import importlib.util; print(importlib.util.find_spec("cyclo_lab").origin)'
```

Confirm that the mount source is this clone and the package path belongs to the mounted checkout. If they differ, arrange a separate Docker configuration or use the intended existing checkout before proceeding. If you customize the container name or internal path, adjust all commands below.

## Operator Assignments

**operator_A and operator_B each have 50 scenarios.** A/B identify people responsible for collection, not VR or automatic generation methods.

Use the [assignment tables](ASSIGNMENTS.md) or [assignments.csv](assignments.csv). Each object order is balanced between operators as closely as possible. Assignments are not inferred from odd/even IDs or the first/last 50 IDs.

`--scenario_index` is a **1-based position in that operator's CSV list**, in its existing row order. It is not the numeric suffix of a scenario ID. IDs range from `batch02_toast_000` to `batch02_toast_099`.

## Load a Scenario

Run these host commands from the cloned repository root after setup:

```bash
# Operator A — scenario 1 of 50
docker exec -it -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/view_make_toast_scenario.py \
  --operator operator_A --scenario_index 1
```

```bash
# Operator B — scenario 1 of 50
docker exec -it -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/view_make_toast_scenario.py \
  --operator operator_B --scenario_index 1
```

Change the number from **1 to 50** to select another entry. The program prints the operator, list position, and actual scenario ID. Supply both arguments together; do not combine them with `--scenario_id`.

You can also select an ID directly:

```bash
docker exec -it -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/view_make_toast_scenario.py --scenario_id batch02_toast_000
```

Focus the simulation window and press **R** to restore the selected initial layout. Reset does not advance to another scenario. Objects can settle under physics after reset. Close the window to exit; start a new environment to select another scenario. The viewer holds the arms with open grippers and lift at zero, without deliberately disturbing objects.

**Path notes:** the default manifest and assignment CSV are resolved from the checkout, not the current working directory. Keep the full checkout and its editable installation. Use `--scenario_manifest scenario_delivery/manifest.json` to specify a manifest explicitly from the repository root; relative paths are relative to the process working directory. Operator selection always uses this checkout's assignment CSV and checks its IDs against the selected manifest. Omitting all selection arguments opens the original task layout.

### Optional Headless Check

After checking GPU availability, a finite check can exercise initial placement and two resets:

```bash
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/view_make_toast_scenario.py \
  --operator operator_A --scenario_index 1 --headless --steps 150 --reset_every 50
```

Successful pose checks print `RESET_POSES_OK`; a completed finite run prints `VIEW_COMPLETE`. This is not a full-task test.

## Using the Scenarios for Data Collection

This package provides fixed layouts and code to apply them. The viewer is for scene inspection and does not save demonstrations. Selecting A/B does not start data collection.

Connect the selected ID to your own teleoperation or collection code. After initializing Isaac Sim and registering the task, set these fields **before `gym.make()`**:

```python
from pathlib import Path
import gymnasium as gym
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg

# Run from the repository root. Replace this ID with one from your assignment.
scenario_id = "batch02_toast_000"
manifest = Path("scenario_delivery/manifest.json").resolve()
task = "Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2"
cfg = parse_env_cfg(task, device="cuda:0", num_envs=1)
cfg.scenario_id = scenario_id
cfg.scenario_manifest = str(manifest)
env = gym.make(task, cfg=cfg)
```

This is an integration example, not a claim that an existing recorder accepts these CLI options. Record the operator, actual scenario ID, and SHA-256 of the manifest alongside your collected data.

## Validation

All 100 scenarios were verified for initial placement and restoration across two resets using the earlier layout checker.

A later headless smoke check of the minimal viewer completed for `batch02_toast_000`: `RESET_POSES_OK` appeared three times, followed by `VIEW_COMPLETE steps=150` and exit code 0. That run preceded the operator-selection CLI. The new selection logic has CPU tests; it has not yet been run in simulation. These checks do not establish manipulation or full toast-task success.

## Files

| File | Purpose |
|---|---|
| [README.md](README.md) | Scenario design and loading instructions. |
| [manifest.json](manifest.json) | Fixed IDs and final object poses, with generation and provenance information. |
| [assignments.csv](assignments.csv) | Machine-readable operator assignments and object order. |
| [ASSIGNMENTS.md](ASSIGNMENTS.md) | Separate, readable tables for each operator. |
| [preview.html](preview.html) | Layout diagrams with an operator filter. |

Download `preview.html` and open it in a web browser to view the layouts and filter by operator. It is self-contained and does not need an internet connection. GitHub's file view does not run the HTML preview.
