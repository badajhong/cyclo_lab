# Toast Task — 100 Scenarios

## Overview

A collection of 100 fixed object layouts for the humanoid toast task. Each scenario stores final poses for the toaster, plate, bread rack, and four bread slices. These are scenario placement data, not a recorded demonstration dataset.

Loading an ID restores its saved layout. Resetting restores the same initial layout rather than sampling a new one.

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

## Operator Assignments

**operator_A and operator_B each have 50 scenarios.** The assignments are not the first 50 IDs versus the last 50 IDs: follow the [assignment tables](ASSIGNMENTS.md) or [assignments.csv](assignments.csv).

Each object order is distributed as evenly as possible between the two operators, with a difference of at most one scenario per order. A/B identify the operators; they do not specify equipment or collection methods such as VR or automatic generation.

Scenario IDs range from `batch02_toast_000` to `batch02_toast_099`.

## Load a Scenario

Use the scenario loader on the [`toast-scenarios-100` branch](https://github.com/kkkkkmj/cyclo_lab/tree/toast-scenarios-100), together with the existing Isaac Sim / Isaac Lab environment and required assets. Copying these data files into the original upstream repository alone does not apply the layouts automatically.

The branch has been renamed locally; the new GitHub branch link will become available after it is pushed. The existing remote branch is still `toast-scenarios-batch02`.

The example below assumes the host checkout is `/home/mj/nrf_humanoid/cyclo_lab`, the existing container is named `cyclo_lab`, and the checkout is available inside it at `/workspace/cyclo_lab`. Check GPU availability before running it.

Choose an ID assigned to you. This command loads the layout, checks its initial poses and restoration across two resets, saves reports and snapshots, and then exits. It is a layout/reset diagnostic, not a teleoperation or demonstration-collection command.

```bash
cd /home/mj/nrf_humanoid/cyclo_lab
scenario_id=batch02_toast_000  # Replace with an ID assigned to you.
run_dir=$(mktemp -d "$PWD/logs/delivery_check.XXXXXX")
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/check_make_toast_scenarios.py \
  --scenario_id "$scenario_id" \
  --scenario_manifest /workspace/cyclo_lab/scenario_delivery/manifest.json \
  --headless --snapshots \
  --output_dir "/workspace/cyclo_lab/logs/$(basename "$run_dir")/$scenario_id"
```

Omit `--headless` for GUI inspection; the diagnostic still exits after completing its checks. If you move this folder, update `--scenario_manifest` to its path inside the container. Reset restores the selected ID's initial layout. Create a new environment when switching IDs.

## Validation

All 100 scenarios were verified for initial placement and restoration across two resets.

## Files

| File | Purpose |
|---|---|
| [README.md](README.md) | Scenario design and loading instructions. |
| [manifest.json](manifest.json) | Fixed IDs and final object poses, with generation and provenance information. |
| [assignments.csv](assignments.csv) | Machine-readable operator assignments and object order. |
| [ASSIGNMENTS.md](ASSIGNMENTS.md) | Separate, readable tables for each operator. |
| [preview.html](preview.html) | Layout diagrams with an operator filter. |

Download `preview.html` and open it in a web browser to view the layouts and filter by operator. It is self-contained and does not need an internet connection. GitHub's file view does not run the HTML preview.
