"""CPU regression tests using synthetic mesh/instance data, not real USD validation."""
import sys
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch
import numpy as np
from toast_geometry_support import extract_table_support, top_faces


class Path(str):
    def HasPrefix(self,other):return self==other or self.startswith(other+'/')


class Prim:
    def __init__(self,path,mesh=False,instance=False,proxy=False,points=None):
        self.path=Path(path);self.mesh=mesh;self.instance=instance;self.proxy=proxy
        self.points=points;self.counts=[4];self.indices=[0,1,2,3]
        self.parent=None
    def GetPath(self):return self.path
    def GetTypeName(self):return 'Mesh' if self.mesh else 'Xform'
    def IsA(self,kind):return self.mesh
    def IsActive(self):return True
    def IsLoaded(self):return True
    def IsDefined(self):return True
    def IsInstance(self):return self.instance
    def IsInstanceProxy(self):return self.proxy
    def GetPrototype(self):return Prim('/__Prototype_1')
    def GetPrimInPrototype(self):return Prim('/__Prototype_1/Mesh')
    def HasAPI(self,api):return False
    def GetParent(self):return self.parent


class SupportTests(unittest.TestCase):
    def extract(self,points,mesh_present=True,root_present=True):
        root=Prim('/Table',instance=True)
        child=Prim('/Table/Mesh',mesh=True,proxy=True,points=points);child.parent=root
        def mesh(p):
            attr=lambda value:NS(Get=lambda:value)
            return NS(GetPointsAttr=lambda:attr(p.points),GetFaceVertexCountsAttr=lambda:attr(p.counts),
                      GetFaceVertexIndicesAttr=lambda:attr(p.indices),GetHoleIndicesAttr=lambda:attr([]))
        # Default traversal sees the instance root only. Proxy traversal sees mesh.
        traversal_calls=[]
        def prim_range(p,*predicate):
            traversal_calls.append(predicate)
            return [root,child] if predicate and mesh_present else [root]
        pxr=NS(Usd=NS(PrimRange=prim_range,TraverseInstanceProxies=lambda pred:('proxy',pred),PrimAllPrimsPredicate='all'),
               UsdGeom=NS(Mesh=mesh),UsdPhysics=NS(CollisionAPI=object()),Gf=NS(Vec3d=lambda *p:p))
        stage=NS(GetPrimAtPath=lambda path:root if root_present and path=='/Table' else None)
        # A translated instance must produce WORLD vertices, not prototype coordinates.
        cache=NS(GetLocalToWorldTransform=lambda prim:NS(Transform=lambda p:np.asarray(p)+[3,4,2]))
        with patch.dict(sys.modules,{'pxr':pxr}):result=extract_table_support(stage,'/Table',cache)
        return result,traversal_calls

    def test_instance_proxy_mesh_is_read_in_world_frame(self):
        r,calls=self.extract([[0,0,0],[1,0,0],[1,1,0],[0,1,0]])
        self.assertEqual(r['status'],'EXTRACTED')
        self.assertEqual(r['top_z_m'],2)
        self.assertEqual(r['convex_surface_polygons_xy'][0],[(3,4),(4,4),(4,5),(3,5)])
        self.assertEqual(r['diagnostics']['default_mesh_count'],0)
        self.assertEqual(r['diagnostics']['proxy_mesh_count'],1)
        self.assertTrue(r['diagnostics']['prims'][1]['instance_proxy'])
        self.assertEqual(calls,[(),(('proxy','all'),)])

    def test_no_mesh_has_diagnostics_no_fabricated_height(self):
        r,_=self.extract([],mesh_present=False)
        self.assertEqual(r['status'],'EVIDENCE_INCOMPLETE')
        self.assertIsNone(r['top_z_m'])
        self.assertEqual(r['convex_surface_polygons_xy'],[])
        self.assertEqual(r['diagnostics']['prims'][0]['excluded'],'not_UsdGeom_Mesh')

    def test_missing_root_has_target_path(self):
        r,_=self.extract([],root_present=False)
        self.assertEqual(r['diagnostics']['target_path'],'/Table')
        self.assertIn('target_prim_missing',r['diagnostics']['errors'])
        self.assertIsNone(r['top_z_m'])

    def test_empty_points_are_incomplete_and_recorded(self):
        r,_=self.extract([])
        self.assertEqual(r['status'],'EVIDENCE_INCOMPLETE')
        self.assertIn('empty_points',r['diagnostics']['prims'][1]['error'])
        self.assertEqual(r['convex_surface_polygons_xy'],[])

    def test_no_horizontal_surface_does_not_pass(self):
        r,_=self.extract([[0,0,0],[1,0,0],[1,1,1],[0,1,1]])
        self.assertEqual(r['status'],'EVIDENCE_INCOMPLETE')
        self.assertEqual(r['diagnostics']['prims'][1]['excluded_faces'],{'not_horizontal':1})

    def test_invalid_indices_are_rejected(self):
        with self.assertRaisesRegex(ValueError,'topology'):
            top_faces([[0,0,0],[1,0,0],[0,1,0]],[3],[0,1,9])

    def test_holes_and_concave_faces_are_not_filled(self):
        points=[[0,0,0],[1,0,0],[1,1,0],[0,1,0]]
        polys,reasons=top_faces(points,[4],[0,1,2,3],holes=[0])
        self.assertEqual(polys,[]);self.assertEqual(reasons,{'authored_hole':1})
        points=[[0,0,0],[2,0,0],[1,.5,0],[2,2,0],[0,2,0]]
        polys,reasons=top_faces(points,[5],[0,1,2,3,4])
        self.assertEqual(polys,[]);self.assertEqual(reasons,{'non_convex_face':1})


if __name__=='__main__':unittest.main()
