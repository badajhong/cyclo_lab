"""Keep the original packing table without its unrelated packing props."""

import isaaclab.sim as sim_utils
from isaacsim.core.utils.stage import get_current_stage


TABLE_GROUP = "SM_CratePacking_Table_A1"
TABLE_FRAME = "SM_HeavyDutyPackingTable_C02_01"


@sim_utils.clone
def spawn_toast_serving_plate(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Use the source plate with dark blue glaze and thin collision hulls."""
    root = spawn_toast_bimanual_toaster.__wrapped__(prim_path, cfg, translation, orientation, **kwargs)
    material_path = f"{prim_path}/ContactMaterial"
    material = sim_utils.RigidBodyMaterialCfg(static_friction=.8, dynamic_friction=.6,
                                            restitution=0.0, friction_combine_mode="max")
    material.func(material_path, material)
    sim_utils.bind_physics_material(prim_path, material_path)
    glaze_path = f"{prim_path}/DarkBlueGlaze"
    glaze = sim_utils.PreviewSurfaceCfg(diffuse_color=(.025, .075, .14), roughness=.55, metallic=.05)
    glaze.func(glaze_path, glaze)
    sim_utils.bind_visual_material(prim_path, glaze_path, stronger_than_descendants=True)
    return root


@sim_utils.clone
def spawn_toast_bimanual_toaster(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Keep the original meshes and correct inflated hulls on thin decorative faces."""
    root = sim_utils.spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    from pxr import Usd, UsdPhysics, PhysxSchema
    for prim in Usd.PrimRange(root):
        if not prim.HasAPI(UsdPhysics.MeshCollisionAPI):
            continue
        approximation = UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()
        if approximation == "convexDecomposition":
            PhysxSchema.PhysxConvexDecompositionCollisionAPI.Apply(prim).CreateMinThicknessAttr().Set(.001)
        elif approximation == "convexHull":
            PhysxSchema.PhysxConvexHullCollisionAPI.Apply(prim).CreateMinThicknessAttr().Set(.001)
    return root


@sim_utils.clone
def spawn_toast_popup_toaster(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Add a driven, physical two-slot bread carriage to the original toaster."""
    # Call the undecorated spawner: the outer clone handles all environment copies.
    root = spawn_toast_bimanual_toaster.__wrapped__(prim_path, cfg, translation, orientation, **kwargs)
    from pxr import Gf, UsdGeom, UsdPhysics
    stage = root.GetStage()
    body_path = f"{prim_path}/bread_carriage"
    body = UsdGeom.Xform.Define(stage, body_path)
    UsdPhysics.RigidBodyAPI.Apply(body.GetPrim())
    UsdPhysics.MassAPI.Apply(body.GetPrim()).CreateMassAttr(.15)
    for i, y in enumerate((-.026455, .0253585)):
        tray = UsdGeom.Cube.Define(stage, f"{body_path}/tray_{i}")
        tray.CreateSizeAttr(1.0)
        # Calibrated against the source bread's collision mesh (its visual tips
        # extend beyond its supporting hull): about one third remains visible.
        tray.AddTranslateOp().Set(Gf.Vec3d(-.003, y, .0071))
        tray.AddScaleOp().Set(Gf.Vec3f(.134, .020, .003))
        tray.CreateDisplayColorAttr([Gf.Vec3f(.30, .32, .34)])
        UsdPhysics.CollisionAPI.Apply(tray.GetPrim())
    joint = UsdPhysics.PrismaticJoint.Define(stage, f"{prim_path}/joints/carriage_joint")
    joint.CreateBody0Rel().SetTargets([f"{prim_path}/base"])
    joint.CreateBody1Rel().SetTargets([body_path])
    joint.CreateAxisAttr("Z")
    joint.CreateLocalPos0Attr(Gf.Vec3f(0))
    joint.CreateLocalPos1Attr(Gf.Vec3f(0))
    joint.CreateLocalRot0Attr(Gf.Quatf(1))
    joint.CreateLocalRot1Attr(Gf.Quatf(1))
    joint.CreateLowerLimitAttr(-.045)
    joint.CreateUpperLimitAttr(0.0)
    drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
    drive.CreateTypeAttr("force")
    drive.CreateStiffnessAttr(8000.0)
    drive.CreateDampingAttr(120.0)
    drive.CreateMaxForceAttr(40.0)
    drive.CreateTargetPositionAttr(0.0)
    return root


@sim_utils.clone
def spawn_toast_serving_toaster(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Allow a 15 mm high-lift pop-up for the FFW-BG2 fingertip clearance."""
    root = spawn_toast_popup_toaster.__wrapped__(prim_path, cfg, translation, orientation, **kwargs)
    from pxr import UsdPhysics
    joint = UsdPhysics.PrismaticJoint(root.GetStage().GetPrimAtPath(f"{prim_path}/joints/carriage_joint"))
    joint.CreateUpperLimitAttr(.015)
    return root


@sim_utils.clone
def spawn_toast_bimanual_robot(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Give the two grippers an explicit, configurable rubber contact material."""
    root = sim_utils.spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    stage = get_current_stage()
    sim_utils.make_uninstanceable(prim_path, stage)
    material = sim_utils.RigidBodyMaterialCfg(static_friction=1.0, dynamic_friction=.8,
                                            friction_combine_mode="max", restitution=0.0)
    material_path = f"{prim_path}/FingertipMaterial"
    material.func(material_path, material)
    from pxr import Usd
    for prim in Usd.PrimRange(root):
        path = str(prim.GetPath())
        if "/collisions/" in path and any(f"rh_p12_rn_{tip}/" in path for tip in ("r2", "l2")):
            sim_utils.bind_physics_material(path, material_path)
    return root


@sim_utils.clone
def spawn_toast_table(prim_path, cfg, translation=None, orientation=None, **kwargs):
    """Remove crates, boxes, and the tabletop basket before cloning environments."""
    root = sim_utils.spawn_from_usd(prim_path, cfg, translation, orientation, **kwargs)
    group = root.GetChild(TABLE_GROUP)
    if not group or not group.GetChild(TABLE_FRAME):
        raise RuntimeError("PackingTable asset no longer contains the expected table frame.")
    for child in group.GetChildren():
        if child.GetName() != TABLE_FRAME:
            child.SetActive(False)
    basket = root.GetChild("container_h20")
    if basket:
        # Deactivation removes both rendered meshes and their physics colliders.
        basket.SetActive(False)
    return root
