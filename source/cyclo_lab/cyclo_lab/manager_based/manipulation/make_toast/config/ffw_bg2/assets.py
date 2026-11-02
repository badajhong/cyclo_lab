"""Paths and dimensions of the pinned RoboDojo objects (all distances in metres)."""

from pathlib import Path

from cyclo_lab.assets.robots import CYCLO_LAB_ASSETS_DATA_DIR


ASSET_DIR = Path(CYCLO_LAB_ASSETS_DATA_DIR) / "objects" / "robodojo_make_toast"
TABLE_HEIGHT = 1.0
TOASTER_POSITION = (0.64, -0.17, TABLE_HEIGHT + 0.0818)
# The lever is on the asset's +X side. Face it toward the robot (-world X).
TOASTER_ROTATION = (0.0, 0.0, 0.0, 1.0)
SHELF_POSITION = (0.42, -0.43, TABLE_HEIGHT + 0.0382)
# Bread stands in the four grooves of the original bread_shelf asset.
BREAD_OFFSETS = ((0.035, -0.007, 0.055), (0.010, -0.007, 0.055),
                 (-0.015, -0.007, 0.055), (-0.039, -0.007, 0.055))
BREAD_ROTATION = (0.7071067811865476, 0.0, 0.7071067811865475, 0.0)
# From toaster/00000/metadata.json: slot frames are attached to the fixed base.
SLOT_X_BOUNDS = (-0.078, 0.072)
SLOT_Y_BOUNDS = ((-0.041, -0.011), (0.014, 0.039))
# The second physical slot settles deeper than the upstream reward's -0.021 m
# cutoff. Include its measured resting depth (-0.035 m), including lever travel.
SLOT_Z_BOUNDS = (-0.045, 0.014)
LEVER_JOINT = "joint_1"
LEVER_OPEN = -0.0138
LEVER_PRESSED_RATIO = 0.85

# Mirrored, raised standby for the 15-action task. Calibrated from physical FK;
# the fingertips are at z=1.429 m, 173 mm above the original keyboard pose.
BIMANUAL_STANDBY_RIGHT_JOINTS = (0.0, -1.8, .03, -2.1, -1.44, .2, -.65)
BIMANUAL_STANDBY_WRIST = (.3609426, .3954084, 1.4419125)

# Flatten the RoboDojo plate mesh; the spawner applies a dark blue glaze.
SERVING_PLATE_SCALE = (1.5, 1.5, .22)
SERVING_PLATE_POSITION = (.43, .035, TABLE_HEIGHT + .0178308 * .22 + .0003)
# Inscribed radius of the pinned plate's actual convex XY outline.
SERVING_PLATE_RADIUS = .0644821164 * SERVING_PLATE_SCALE[0]
BREAD_BOUNDS = ((-.05838766, .05856051), (-.05828097, .05778116), (-.00029194, .01150705))


def asset_path(name: str) -> str:
    """Resolve only when this task is instantiated, so other tasks need no download."""
    path = ASSET_DIR / name / "object.usdz"
    if not path.is_file():
        raise FileNotFoundError(
            f"Missing Make Toast asset: {path}\n"
            "Run from the Cyclo Lab repository: python3 scripts/tools/download_make_toast_assets.py"
        )
    return str(path)
