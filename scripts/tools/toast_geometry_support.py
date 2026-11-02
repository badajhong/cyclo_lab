"""Read-only tabletop extraction; never substitute an extent/bbox for mesh faces."""
from collections import Counter
import numpy as np
from toast_batch_lib import area, hull


def top_faces(points, counts, indices, holes=(), top=None):
    """Validate topology and return actual convex top faces plus exclusion counts."""
    points=np.asarray(points,dtype=float)
    if points.ndim!=2 or points.shape[1]!=3 or not len(points) or not np.isfinite(points).all():
        raise ValueError('empty_or_invalid_points')
    if (not counts or any(c<0 for c in counts) or sum(counts)!=len(indices)
            or any(i<0 or i>=len(points) for i in indices)):
        raise ValueError('invalid_face_topology')
    if top is None:top=float(points[:,2].max())
    surfaces=[];excluded=Counter();offset=0
    for number,count in enumerate(counts):
        face=points[indices[offset:offset+count]];offset+=count
        if number in holes:reason='authored_hole'
        elif count<3:reason='fewer_than_three_vertices'
        elif np.ptp(face[:,2])>1e-4:reason='not_horizontal'
        elif face[:,2].min()<top-.002:reason='below_top_band'
        else:
            try:poly=hull(face.tolist())
            except ValueError:poly=[]
            signed=abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(face,np.roll(face,-1,axis=0))))/2
            if area(poly)<=1e-12:reason='degenerate_face'
            elif abs(area(poly)-signed)>1e-8:reason='non_convex_face'
            else:
                surfaces.append(poly)
                continue
        excluded[reason]+=1
    return surfaces,dict(excluded)


def extract_table_support(stage, path, cache):
    from pxr import Usd, UsdGeom, UsdPhysics, Gf
    root=stage.GetPrimAtPath(path)
    diagnostics={'target_path':path,'prims':[], 'default_mesh_count':0,
                 'proxy_mesh_count':0,'selected_mesh_paths':[], 'errors':[]}
    result={'kind':None,'status':'EVIDENCE_INCOMPLETE','top_z_m':None,
            'convex_surface_polygons_xy':[],'mesh_paths':[],'diagnostics':diagnostics,
            'note':'Union of actual convex top faces; no bbox/extent substitution. Authored geometry, not cooked PhysX shapes.'}
    if not root:
        diagnostics['errors'].append('target_prim_missing')
        parent=stage.GetPrimAtPath(path.rsplit('/',1)[0])
        diagnostics['existing_parent_children']=[str(p.GetPath()) for p in parent.GetAllChildren()] if parent else []
        return result
    diagnostics['default_mesh_count']=sum(p.IsA(UsdGeom.Mesh) for p in Usd.PrimRange(root))
    meshes=[];colliders=[]
    # Proxy paths retain the instance's world transform. Traversing the prototype
    # directly would lose that transform; never author SetInstanceable here.
    for prim in Usd.PrimRange(root, Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)):
        row={'path':str(prim.GetPath()),'type':str(prim.GetTypeName()),
             'active':prim.IsActive(),'loaded':prim.IsLoaded(),'defined':prim.IsDefined(),
             'instance':prim.IsInstance(),'instance_proxy':prim.IsInstanceProxy()}
        if prim.IsInstance():row['prototype_path']=str(prim.GetPrototype().GetPath())
        elif prim.IsInstanceProxy():row['prototype_path']=str(prim.GetPrimInPrototype().GetPath())
        if not row['active'] or not row['loaded'] or not row['defined']:
            row['excluded']='inactive_unloaded_or_undefined'
        elif not prim.IsA(UsdGeom.Mesh):row['excluded']='not_UsdGeom_Mesh'
        else:
            meshes.append((prim,row))
            # Retain the original collision-vs-visual selection policy.
            node=prim;collision=False
            while node and node.GetPath().HasPrefix(root.GetPath()):
                if node.HasAPI(UsdPhysics.CollisionAPI):
                    collision=UsdPhysics.CollisionAPI(node).GetCollisionEnabledAttr().Get() is not False
                    break
                node=node.GetParent()
            row['collision_enabled']=collision
            if collision:colliders.append((prim,row))
        diagnostics['prims'].append(row)
    diagnostics['proxy_mesh_count']=len(meshes)
    selected=colliders or meshes
    result['kind']='AUTHORED_COLLISION_FACES' if colliders else 'VISUAL_FACES'
    for prim,row in meshes:
        if colliders and not row['collision_enabled']:row['excluded']='visual_mesh_collision_meshes_preferred'
    diagnostics['selected_mesh_paths']=[str(p.GetPath()) for p,_ in selected]
    if not selected:
        diagnostics['errors'].append('no_meshes_after_instance_proxy_traversal')
        return result
    data=[]
    for prim,row in selected:
        try:
            mesh=UsdGeom.Mesh(prim);raw=mesh.GetPointsAttr().Get()
            if raw is None or not len(raw):raise ValueError('empty_points')
            matrix=cache.GetLocalToWorldTransform(prim)
            points=np.asarray([matrix.Transform(Gf.Vec3d(*map(float,p))) for p in raw])
            counts=list(mesh.GetFaceVertexCountsAttr().Get() or [])
            indices=list(mesh.GetFaceVertexIndicesAttr().Get() or [])
            holes=list(mesh.GetHoleIndicesAttr().Get() or [])
            top_faces(points,counts,indices,holes)  # Validate before computing maximum.
            data.append((prim,row,points,counts,indices,holes))
        except Exception as error:
            row['excluded']='unreadable_or_invalid_mesh';row['error']=repr(error)
            diagnostics['errors'].append(str(prim.GetPath())+': '+repr(error))
    if diagnostics['errors']:
        return result  # Incomplete mesh evidence must not become support evidence.
    top=max(float(points[:,2].max()) for _,_,points,_,_,_ in data)
    result['top_z_m']=top
    for prim,row,points,counts,indices,holes in data:
        polygons,exclusions=top_faces(points,counts,indices,holes,top)
        row['excluded_faces']=exclusions;row['accepted_faces']=len(polygons)
        if not polygons:row['excluded']='no_convex_horizontal_faces_in_top_band'
        else:
            result['convex_surface_polygons_xy'].extend(polygons)
            result['mesh_paths'].append(str(prim.GetPath()))
    if result['convex_surface_polygons_xy']:result['status']='EXTRACTED'
    else:diagnostics['errors'].append('no_convex_horizontal_faces_in_top_band')
    return result
