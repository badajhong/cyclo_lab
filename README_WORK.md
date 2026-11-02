# Toast Scenario Work — Recovery, Fixed Layouts, and Validation

Fork: https://github.com/kkkkkmj/cyclo_lab.git

The local branch is `toast-scenarios-100`; the remote branch is still named
`toast-scenarios-batch02` until the new branch is pushed. This fork contains the
implementation, while [scenario_delivery/README.md](scenario_delivery/README.md)
describes the smaller package for collection operators.

## 1. Restoring the missing Isaac Lab datasets module

The supplied checkout could not import `isaaclab.utils.datasets` because the
module was missing. The root `.gitignore` rule `datasets` excluded both recorded
demonstration data and the Python package with that name.

The four missing files were restored from the official `isaac-sim/IsaacLab`
**v2.3.0** source:

- Tag commit: `3c6e67bb5c7ada942a6d1884ab69338f57596f77`.
- Local reference: `/home/mj/nrf_humanoid/reference_sources/IsaacLab-v2.3.0`.
- [Official source directory](https://github.com/isaac-sim/IsaacLab/tree/3c6e67bb5c7ada942a6d1884ab69338f57596f77/source/isaaclab/isaaclab/utils/datasets).
- Destination: `third_party/IsaacLab/source/isaaclab/isaaclab/utils/datasets/`.
- Files: `__init__.py`, `dataset_file_handler_base.py`, `episode_data.py`, and
  `hdf5_dataset_file_handler.py`.

Each restored file's SHA-256 matches its official local source. Isaac Lab was
not replaced or upgraded, and no stubs were introduced. A matching VERSION file
does not establish that the entire supplied checkout matches the official source.

The existing `datasets` ignore rule remains in place. Exceptions allow tracking
only the package directory above and its `*.py` files; recorded demonstration
data remains excluded. These repairs predate this documentation update.

## 2. Fixed scenario layouts

The target task is `Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2`.

- `cfg.scenario_id` selects final poses from a manifest before scene creation.
- Moving the bread rack transforms all four bread slices with it, preserving
  their relative poses.
- Original Z coordinates and scales are preserved. Existing resets restore the
  selected initial states.
- Final positions and quaternions are stored explicitly, rather than regenerated
  from a seed alone.
- The default `scenario_id=None` behavior and existing pilot IDs are preserved.
- Physics settings, expert behavior, and success conditions are unchanged.

The 100 scenarios in batch02 cover all six object orders, with
17/17/17/17/16/16 scenarios. Each object has a yaw offset of ±5° around its
reference orientation: toaster 150°, plate 0°, and bread rack −90°.
The slots run from environment +Y to −Y at +0.40/0/−0.40 m. Nominal X coordinates
are 0.55 m for the toaster, 0.49 m for the plate, and 0.46 m for the rack.
The plate's X offset is ±0.010 m; all other XY offsets are ±0.015 m.
Each operator has 50 scenarios, with a difference of at most one assignment per
object order. A/B identify collection operators, not equipment or methods.

Existing geometry evidence was reused to check the actual tabletop support
region, footprints accounting for scale and yaw, minimum separation, planar
approach clearance, and conservative reach upper bounds based on the two arms
and lift model. Visual bounding boxes and authored collider bounds are distinct
from PhysX cooked collision geometry and actual contact checks. The margins are
1 mm for metadata uncertainty, 20 mm for table edges, 30 mm between objects,
and 60 mm for planar approach clearance.

## 3. Runtime validation results and limitations

See the [validation summary](scenarios/make_toast_v2_batch02/runtime_review/summary.json)
and [operator validation table](scenarios/make_toast_v2_batch02/runtime_review/operator_validation.csv).

- Original results: `logs/scenario_batch02_six.46bMoE/results` and
  `logs/scenario_batch02_remaining.ucq8XR/results`.
- Both run summaries contain the same manifest SHA-256 as the current batch02 manifest.
- Exactly 100 unique IDs were checked, with no missing or duplicate IDs. All
  exited with code 0 and passed initial pose checks and restoration across two resets.
- All 50 scenarios for operator_A and all 50 for operator_B passed these checks.
- Maximum initial position error was approximately 2.97e-8 m. The maximum scalar
  difference in the recorded reset state was 0.
- Integrity was checked for 1,500 PNG files. A total of 300 head-camera images
  covering each ID's initial view and two settled views were visually reviewed.
  No obvious departures from the table, tipping, or fallen bread were identified
  in visible areas. Cropped edges and occluded contact surfaces remain unconfirmed.

**Actual manipulation, IK, collision-free approach paths, full toast completion,
and successful demonstration collection remain unverified.** The original
expert's single check failed during the plate-serving release phase. Passing
layout/reset checks does not establish expert success. Plates and bread move
downward and tilt during settling; a PASS does not mean objects remain motionless.

The manifest's `CANDIDATES_NOT_GPU_VERIFIED` and `gpu_verified: 0` values record
its state at generation time. The data was preserved, and subsequent runtime
validation is linked through the manifest hash rather than overwriting that history.

## 4. Evidence dependencies and reproducibility limits

Applying the fixed manifest and viewing the HTML preview do not require past
execution logs. Running the environment requires its original dependencies and
robot, object, and table assets.

The following operations cannot be reproduced or independently reviewed in full
without the original evidence:

| Operation | Additional local material required |
|---|---|
| Rerun the batch02 generator | The three existing pilot `report.json` files, pinned metadata and robot USD, and extracted `geometry.json` |
| Rerun the audit | The corresponding `geometry.json`, manifest, and local files used for asset hash checks |
| Run all of `validate_static.py` | batch01, geometry, metadata and robot files, and originals referenced by the preservation record |
| Independently review the 100 historical runtime results | Original reports, results, summaries, logs, and PNGs from both run folders |
| Check preservation hashes | The historical files listed in the record; hashes alone cannot reconstruct the originals |

Hashes and paths in the distributed summaries record provenance; they do not
embed the original files. Static code unit tests must also be distinguished from
checks using actual geometry or simulation.
