"""Read the exact convex support vertices of the pinned RoboDojo bread mesh.

Using the actual 3D hull avoids testing empty corners of a rectangular box or
an extruded footprint. Source geometric data: RoboDojo (Apache-2.0 dataset).
"""

from functools import lru_cache


@lru_cache(maxsize=1)
def bread_support_vertices():
    """Return metre-scale vertices in the bread root's frame; cache per process."""
    import numpy as np
    from pxr import Usd, UsdGeom
    from scipy.spatial import ConvexHull
    from .assets import asset_path

    stage = Usd.Stage.Open(asset_path("bread"))
    root = stage.GetDefaultPrim()
    inverse_root = np.linalg.inv(np.asarray(
        UsdGeom.Xformable(root).ComputeLocalToWorldTransform(Usd.TimeCode.Default()),
    ))
    arrays = []
    for prim in Usd.PrimRange(root):
        if not prim.IsA(UsdGeom.Mesh) or "/collision" in str(prim.GetPath()):
            continue
        mesh = UsdGeom.Mesh(prim)
        if mesh.ComputeVisibility() == "invisible":
            continue
        transform = np.asarray(mesh.ComputeLocalToWorldTransform(Usd.TimeCode.Default())) @ inverse_root
        points = np.asarray(mesh.GetPointsAttr().Get())
        arrays.append(points @ transform[:3, :3] + transform[3, :3])
    if not arrays:
        raise RuntimeError("The pinned bread USD contains no visual mesh")
    vertices = np.concatenate(arrays)
    return np.ascontiguousarray(vertices[ConvexHull(vertices).vertices], dtype=np.float32)
