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

Use this fork's `toast-scenarios-100` branch with **Isaac Sim 5.1.0 / Isaac Lab 2.3.0**, the installed `cyclo_lab` package, and the original robot, table, and toast object assets. Keep the repository's Docker and asset download setup. Copying this data folder into the upstream repository alone does not install the scenario loader.

With the repository’s default Docker configuration, the checkout is mounted at `/workspace/cyclo_lab` inside the `cyclo_lab` container, regardless of the host checkout’s directory name. Run from the host; `-w` sets the container working directory to the repository root:

```bash
docker exec -it -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/view_make_toast_scenario.py \
  --scenario_id batch02_toast_000 \
  --scenario_manifest scenario_delivery/manifest.json
```

Choose your ID from the assignment table. The manifest argument is optional: it defaults to this checkout's `scenario_delivery/manifest.json`. An explicit path can select another compatible fixed-pose manifest. Omitting `--scenario_id` preserves the original task layout. The default manifest is resolved from the installed loader’s source location, not the current working directory; use the editable checkout installation provided by the Docker setup. Explicit relative manifest paths are resolved from the process working directory. If you customize `DOCKER_CYCLOLAB_PATH` or `DOCKER_NAME_SUFFIX`, adjust the container path or name in these commands accordingly.

The viewer holds the arms with open grippers and lift at zero. It does not run the expert, record demonstrations, or deliberately disturb objects. GUI display access must be configured for the existing container. Focus the simulation window and press **R** to reset; close the window to exit. Reset restores the same initial poses, after which objects can settle under physics. Start a new environment to change IDs.

For a finite headless check of the viewer and two resets, first check GPU availability, then use:

```bash
docker exec -w /workspace/cyclo_lab cyclo_lab /isaac-sim/python.sh \
  scripts/tools/view_make_toast_scenario.py \
  --scenario_id batch02_toast_000 --headless --steps 150 --reset_every 50
```

Each reset checks the immediate object poses against the selected configuration and prints `RESET_POSES_OK`; a completed finite run prints `VIEW_COMPLETE`. This is not a manipulation or full-task test.

## Validation

All 100 scenarios were verified for initial placement and restoration across two resets.

A separate headless smoke check of the minimal viewer completed for `batch02_toast_000`: initial placement and two resets passed (`RESET_POSES_OK` three times), followed by `VIEW_COMPLETE steps=150` and exit code 0. This check does not cover GUI keyboard handling or manipulation.

## Files

| File | Purpose |
|---|---|
| [README.md](README.md) | Scenario design and loading instructions. |
| [manifest.json](manifest.json) | Fixed IDs and final object poses, with generation and provenance information. |
| [assignments.csv](assignments.csv) | Machine-readable operator assignments and object order. |
| [ASSIGNMENTS.md](ASSIGNMENTS.md) | Separate, readable tables for each operator. |
| [preview.html](preview.html) | Layout diagrams with an operator filter. |

Download `preview.html` and open it in a web browser to view the layouts and filter by operator. It is self-contained and does not need an internet connection. GitHub's file view does not run the HTML preview.
