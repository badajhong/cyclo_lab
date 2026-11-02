# Make Toast object sources

Only these objects are taken from the official RoboDojo distribution:

- `Assets/Object/RoboDojo/Articulation/toaster/00000`
- `Assets/Object/RoboDojo/Rigid/bread/00000`
- `Assets/Object/RoboDojo/Geometry/bread_shelf/00000`
- `Assets/Object/RoboDojo/Rigid/plate/00002` (serving `-v2`)

Source: <https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo>.
The dataset card declares Apache-2.0. Preserve upstream attribution and consult
upstream terms when redistributing assets. This repository downloads the assets
on demand rather than redistributing their binaries. `asset_manifest.json`
pins the upstream revision, file sizes, and SHA-256 checksums.

Task reference: <https://robodojo-benchmark.com/doc/sim-tasks/make-toast/>.
The upstream configuration and completion checks were inspected at commit
`f57e1633ffed59761bc8e568cfe7530c4b33381c` of
<https://github.com/RoboDojo-Benchmark/RoboDojo> (MIT).
This implementation uses the annotated slot X/Y dimensions and lever travel, with
the FFW-BG2 robot and controllers supplied by Cyclo Lab. No RoboDojo robot,
policy, simulator framework, or demonstration dataset is downloaded by the
asset downloader. A separate experimental motion downloader can extract the
two ARX end-effector streams; that is not required by the scene-state expert.
The lower insertion-height tolerance includes the second slot's physical resting
depth in Isaac Sim 5.1, rather than rejecting a slice that has settled inside it.

The standard task has four slices on the rack. Manipulate slices 0 and 1,
leave slices 2 and 3 on the rack, fill both slots, press the lever, and return
the right arm to its reset pose with the gripper open. The imported toaster
does not implement a cooking timer, coupled bread carriage, or automatic pop-up.
Those behaviors are added by the custom bimanual `-v1` and serving `-v2` tasks below.

## Bimanual FFW-BG2 calibration

The separate 15-action task controls two relative wrist poses, two continuous
gripper openings, and the original body lift. It uses three RGB cameras.
Only the rack groove axis is scaled by two (50 mm slice spacing); the bread
and toaster meshes and physical slot widths remain unchanged. The rack is
placed at `(0.45, -0.25)` m, and the toaster at `(0.55, 0.23)` m with a 150° yaw,
presenting the lever toward the left arm.

Calibration version 2 uses a mirrored initial arm posture with wrist height
1.442 m and fingertip height 1.429 m, 173 mm higher than version 1. The default
expert speed multiplier is 2.0; grasp/settling durations are shortened with it.
The handover is raised to 1.43 m. The receiver rotates above the central workspace,
and the bread is turned upright there before travelling above its slot. Shell
and lever contact reports are filtered against the robot. Turning/handover
shell contacts above 2 N reject a demonstration; completion also requires a
measured lever stroke of at least 65 mm and actual robot-to-lever contact.
Lever pressing also rejects shell contact above 2 N, stops at 95% measured
travel, and clears outward before lifting the hand. The pressing pose uses a
60° outward tilt and a 90° twist about the tool axis.

The original toaster authors `minThickness=0.02` on convex collision cooking.
Its thin decorative faces consequently produced contacts about 10 mm outside
the visible front face, covering much of the 13 mm lever projection. The bimanual
spawner sets this minimum thickness to 0.001 m, retaining all visual meshes,
collision meshes, and collision pairs. The main housing SDF is unchanged. The
downloaded USD is unchanged; this override is applied only when spawning the
bimanual task. PhysX defines this field as a distance:
https://docs.omniverse.nvidia.com/kit/docs/usdrt.scenegraph/7.6.1/api/classusdrt_1_1_physx_schema_physx_convex_decomposition_collision_a_p_i.html

The expert derives contact frames from the actual distal finger bodies and
uses observed bread poses while moving. Gripper slave servos use stiffness
100 and damping 4 because the robot USD has no mimic linkage constraints.
Fingertip material coefficients are explicitly set to static friction 1.0
and dynamic friction 0.8. These are simulation assumptions, not measurements
of the physical FFW-BG2. A task-space null-space controller recovers standby
posture and avoids wrist limits without adding policy action dimensions.
State restoration retains actual joint velocities and clears the position
servos' velocity targets, preventing saved transient motion from becoming
persistent velocity feedforward during a replay.

Objects move through simulated robot contact (and carriage contact in `-v1`) during an expert episode;
only environment resets and explicitly marked diagnostic fixtures write
object or joint state directly. Failed or restored partial episodes are
excluded from the exported training data. Whole-episode Mimic replay uses
the calibrated fixed layout; independent object randomization is not claimed.

## Calibration version 3: physical pop-up toaster (`Bimanual-…-v1`)

The original downloaded USD is unchanged. The new task spawns an additional
0.15 kg rigid carriage inside the toaster's articulation, with two 134 × 20 × 3 mm
support trays at slot centers Y = -26.455 / +25.3585 mm. Their raised top faces
are at Z = 8.6 mm in the toaster frame. This height accounts for the source
bread's supporting collision hull, which differs slightly from its visual tips.
The source upright bread visual height is 116.062 mm and the rim Z is 80.233 mm.
Two-environment physical fixture checks measured exposed fractions 0.333–0.335
after settling (about two thirds inserted).

The carriage has a Z prismatic joint with range -45 to 0 mm, a force drive
(stiffness 8000 N/m, damping 120 N s/m, force limit 40 N), and a 0.12 m/s joint
velocity limit. A loaded toaster latches on a physical lever rising edge at
95% travel. The carriage lowers over 0.60 s with a smoothstep target. Once the
measured carriage reaches the lower stop, a 5.00 simulation-second timer starts.
It then raises over 0.80 s with a smoothstep target and returns the lever.
The lever remains passive before latching; its return/latch drive uses stiffness
1500 with the authored damping. These are custom simulation parameters, not
measurements of the original appliance. Bread remains an independent rigid
body throughout; no bread state is directly written during a demonstration.

Completion requires both slices in the raised slots, a finished timer/cycle,
the lever returned, stable bread, spare slices still on the rack, and raised
standby hands with open grippers. The cycle runs in the environment for expert,
Mimic, BC play, and action replay; it is not an expert-only animation.
Its phase, elapsed time, carriage target, previous lever state, and heating time
are recorded under `initial_state/toast_cycle` and `states/toast_cycle` and are
restored by `env.reset_to`. `check_toast_cycle.py` isolates the mechanism with
explicit non-recording fixture states, checking empty-toaster rejection,
environment independence, full hiding, five-second timing, pop-up, and mid-cycle
state restoration.

Version 3 expert timing removes both fixed 5 s posture-recovery pauses and
redundant standby commands. The receiving hand uses a 10 mm capture tolerance
instead of waiting against joint 5's limit for a 6 mm error. The expert shortens
grasp/release dwells, prepares the second pickup during the left arm's return,
and closes the empty hand during its lever approach. The 15-action policy
contract and raised standby geometry remain the same. Existing calibration-2
datasets/checkpoints use the original `Bimanual-…-v0` task; use fresh version-3
data with `Bimanual-…-v1` because the toaster articulation gains a joint.


## Calibration version 4: serve popped toast (`Bimanual-…-v2`)

The plate comes from the same pinned RoboDojo asset revision:
[plate/00002](https://huggingface.co/datasets/RoboDojo-Benchmark/RoboDojo/tree/63fcc045f2e974902eb7800d29de48004cf86016/Assets/Object/RoboDojo/Rigid/plate/00002).
Its USD, metadata, and description are SHA-256 pinned in `asset_manifest.json`.
The upstream description calls it a gray shallow ripple bowl. Its original
bounds are about 129.6 × 129.5 × 35.7 mm; the original material renders very
brightly in this scene. The serving task explicitly scales the mesh by
(1.5, 1.5, 0.22), yielding 194 × 194 × 7.85 mm, and overrides its render material
with dark blue glaze (diffuse RGB 0.025 / 0.075 / 0.14). This is an adapted
RoboDojo mesh, not an unmodified upstream flat blue plate. Downloaded files
remain unchanged. The plate's convex collision minimum thickness is 1 mm.

The plate is a dynamic 0.35 kg rigid body, with friction 0.8 static / 0.6 dynamic,
placed at world XY (0.43, 0.035) m. Its reset bottom starts 0.3 mm above the
nominal table height and settles under gravity onto the original table collider.
The calibrated table frame extends from X=0.319 to 1.081 m. The plate's visual
front edge is X=0.333 m, and its closest Y boundary is about 30 mm from the rack.
Toaster and rack poses are unchanged. Separate plate-to-robot and
shell-to-robot contact sensors use the explicit 22-body robot filters.

For retrieval, the measured physical finger TCP lies along wrist -Z. The wrist
+Z is tilted upward 30 degrees along the bread's long horizontal axis, and the
jaw closing direction follows its actual thin axis, including any slice tilt.
A 15% jaw opening enters above the slice; the TCP pinches 36 mm above its origin
(about 22 mm below the visual top).
This avoids a shallow edge-only grasp that slipped during lowering. The slice
is extracted vertically before any transfer or turnover.

Serving uses a 90-degree yaw and 30-degree roll, keeping the wrist clear of the
table. The expert computes the lowest bread bounding-box point and the current
plate/previous-slice surface, then targets a 50 mm lower-edge release gap.
The fingers open fully and measured joint positions must confirm release.
The slice must already be flat and settled on the plate before the hand
withdraws, preventing a partially held slice from hooking and lifting the dish.
A -4 mm longitudinal and -3 mm lateral compensation centers the slice after
gravity lays it flat. The second slice is stacked over the first with a small lateral offset.
Bread and plate remain independent rigid bodies; only reset and diagnostic
fixtures write their states directly. Final completion checks the exact 3D convex hull of each visual bread mesh
inside the plate's 96.7 mm inscribed radius with a 0.5 mm margin, a flat orientation, support height,
resting velocity, an upright plate near its reset position, completed toasting,
spare bread on the rack, open hands, and standby/lift recovery.

The serving variant raises the carriage to +15 mm after the five-second heating
interval, while retaining its original 0 mm loaded position and -45 mm lowered
position. The 60 mm pop-up uses the same 0.8 s smoothstep rise and 0.12 m/s
velocity limit. Extra exposure gives the large RH-P12 fingers clearance to grip
a tilted slice without striking the rim. The `-v1` toaster retains its original
0 mm pop-up endpoint. Early calibration attempts that struck the housing,
slipped, or left a slice outside the plate are diagnostic failures and are never
exported as demonstrations.

Containment uses the source bread visual mesh's exact 3D convex hull (3,065
vertices), loaded once and cached on the simulation device. Empty corners of an
axis-aligned box and of an extruded 2D footprint produced false failures and are
not used. The plate's inscribed radius is derived from its actual convex XY
outline (64.4821164 mm before scaling), rather than its approximate metadata
radius. All visual bread vertices are independently checked in saved datasets.

Serving Mimic uses separate loading/toasting, extraction, and plate-placement
segments. Extraction transforms follow the measured popped bread pose, including
tilt; placement transforms follow the plate, and the auxiliary lift retains its
recorded trajectory. The right hand stays at standby during serving; identical
unjittered boundaries and equal transition lengths keep all three control
streams frame-aligned. After closure, extraction follows world Z instead of
rotating the lift direction with a tilted slice. Placement compensates the
measured source/current grasp difference and adds 20 mm of clearance near the
plate while the fingers open. A 20 mm longitudinal offset compensates the
measured inward drift during opening/flattening, leaving space when the second
slice lands. These extra offsets fade back near standby. A
fixed whole-episode path missed slices whose orientation changed during pop-up.
`annotate_make_toast_serving.py` derives the boundaries from the source's saved
physical states and pre-action poses, preserving every original action and RGB
frame. It does not synthesize successful states or regenerate observations.
