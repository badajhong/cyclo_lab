"""Extract tabletop/collider surfaces and FFW-BG2 kinematics; no expert or rollout.

Builds the original scene once to access its referenced USD and resolved robot.
This is new geometry evidence collection, not a repeat of the pilot reset test.
"""
import argparse
import hashlib
import json
from pathlib import Path
from isaaclab.app import AppLauncher

parser=argparse.ArgumentParser(description=__doc__)
parser.add_argument('--output',type=Path,required=True)
AppLauncher.add_app_launcher_args(parser)
args=parser.parse_args()
if args.output.exists(): parser.error('Output exists; use a new evidence file')
args.enable_cameras=True
app=AppLauncher(args).app
import numpy as np
import gymnasium as gym
from pxr import Usd,UsdGeom,UsdPhysics,Gf
from scipy.spatial.transform import Rotation
import cyclo_lab
from isaaclab_tasks.utils.parse_cfg import parse_env_cfg
from isaacsim.core.utils.stage import get_current_stage
from toast_batch_lib import ROOT
from toast_geometry_support import extract_table_support

TASK='Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2'
env=None
output={'schema_version':1,'task':TASK,'scope':'authored USD geometry, not PhysX cooked collision validation'}
completed=False

def digest(p): return hashlib.sha256(p.read_bytes()).hexdigest()
def transform(p,q):
    t=np.eye(4);t[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();t[:3,3]=p;return t
def qlist(q): return [q.GetReal(),*q.GetImaginary()]
def world_vertices(prim,cache):
    mesh=UsdGeom.Mesh(prim); points=mesh.GetPointsAttr().Get()
    matrix=cache.GetLocalToWorldTransform(prim)
    return np.asarray([matrix.Transform(Gf.Vec3d(*map(float,p))) for p in points])
def is_collision(prim,root):
    while prim and str(prim.GetPath()).startswith(str(root.GetPath())):
        if prim.HasAPI(UsdPhysics.CollisionAPI):
            return UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is not False
        prim=prim.GetParent()
    return False

try:
    env=gym.make(TASK,cfg=parse_env_cfg(TASK,device=args.device,num_envs=1)).unwrapped
    env.reset(seed=1)
    stage=get_current_stage();cache=UsdGeom.XformCache(Usd.TimeCode.Default())
    robot_path=ROOT/'source/cyclo_lab/data/robots/FFW/FFW_BG2_without_mimic.usd'
    output['robot_usd_sha256']=digest(robot_path)
    output['asset_manifest_sha256']=digest(ROOT/'source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/asset_manifest.json')
    prefix='/World/envs/env_0/'
    output['object_colliders']={}
    for name,prim_name in [('toaster','Toaster'),('plate','Plate'),('bread_shelf','BreadShelf')]:
        root=stage.GetPrimAtPath(prefix+prim_name)
        pos=getattr(env.cfg.scene,name).init_state.pos;quat=getattr(env.cfg.scene,name).init_state.rot
        inv=np.linalg.inv(transform(pos,quat));vertices=[];paths=[];omitted=[]
        for prim in Usd.PrimRange(root):
            if prim.HasAPI(UsdPhysics.CollisionAPI) and not prim.IsA(UsdGeom.Mesh):
                omitted.append(str(prim.GetPath()))
            if not prim.IsA(UsdGeom.Mesh) or not is_collision(prim,root):continue
            points=world_vertices(prim,cache)
            local=(inv@np.c_[points,np.ones(len(points))].T).T[:,:3]
            # Bounding corners enclose the authored mesh. Preserve all components' bounds.
            import itertools
            vertices.extend(list(itertools.product(*zip(local.min(0).tolist(),local.max(0).tolist()))))
            paths.append(str(prim.GetPath()))
        # Generated carriage cubes are inside the toaster shell envelope; their
        # motion and collision cooking still require runtime physical validation.
        output['object_colliders'][name]={'scaled_local_vertices':vertices,'mesh_paths':paths,
                                         'non_mesh_collision_prims_not_measured':omitted,
                                         'scope':'authored mesh bounds at reset; moving-part sweeps NOT_TESTED'}
    table_path=prefix+'PackingTable/SM_CratePacking_Table_A1/SM_HeavyDutyPackingTable_C02_01'
    output['table_support']={'status':'EVIDENCE_INCOMPLETE',
        'convex_surface_polygons_xy':[], 'diagnostics':{'target_path':table_path}}
    output['table_support']=extract_table_support(stage,table_path,cache)
    if output['table_support']['status']!='EXTRACTED':
        raise RuntimeError('Tabletop evidence incomplete; see table_support.diagnostics in output JSON')
    try:
        robot=env.scene['robot'];rroot=stage.GetPrimAtPath(prefix+'Robot')
        joints=[UsdPhysics.Joint(p) for p in Usd.PrimRange(rroot) if p.IsA(UsdPhysics.Joint)]
        def get_joint(name):
            hits=[j for j in joints if j.GetPrim().GetName()==name]
            if len(hits)!=1: raise ValueError(f'Expected one joint {name}, found {len(hits)}')
            return hits[0]
        def body_pose(path):
            idx=robot.body_names.index(str(path).split('/')[-1])
            return transform(robot.data.body_link_pos_w[0,idx].cpu().tolist(),robot.data.body_link_quat_w[0,idx].cpu().tolist())
        lift=get_joint('lift_joint');parent=body_pose(lift.GetBody0Rel().GetTargets()[0])
        lrot=transform([0,0,0],qlist(lift.GetLocalRot0Attr().Get()))
        axis=parent[:3,:3]@lrot[:3,:3]@np.eye(3)['XYZ'.index(UsdPhysics.PrismaticJoint(lift.GetPrim()).GetAxisAttr().Get())]
        lift_idx=robot.joint_names.index('lift_joint');lift_home=float(robot.data.joint_pos[0,lift_idx])
        # Intersection of authored soft limits and the unmodified action's range.
        lim=robot.data.soft_joint_pos_limits[0,lift_idx].cpu().tolist()
        model={'lift_axis_world':axis.tolist(),'lift_limits':[max(-.5,lim[0]),min(0,lim[1])],'arms':{},'fk_validated':True}
        by_child={str(j.GetBody1Rel().GetTargets()[0]):j for j in joints if j.GetBody1Rel().GetTargets()}
        for side,letter in [('left','l'),('right','r')]:
            chain=[];home=[];previous=None
            for i in range(1,8):
                j=get_joint(f'arm_{letter}_joint{i}');body0=str(j.GetBody0Rel().GetTargets()[0]);body1=str(j.GetBody1Rel().GetTargets()[0])
                if previous is not None and previous!=body0:raise ValueError('Non-serial arm joint chain')
                if i==1:
                    base=body_pose(body0);base[:3,3]-=axis*lift_home
                idx=robot.joint_names.index(f'arm_{letter}_joint{i}')
                chain.append({'pos0':list(j.GetLocalPos0Attr().Get()),'rot0':qlist(j.GetLocalRot0Attr().Get()),
                  'pos1':list(j.GetLocalPos1Attr().Get()),'rot1':qlist(j.GetLocalRot1Attr().Get()),
                  'axis':UsdPhysics.RevoluteJoint(j.GetPrim()).GetAxisAttr().Get(),
                  'limits_rad':robot.data.soft_joint_pos_limits[0,idx].cpu().tolist()})
                home.append(float(robot.data.joint_pos[0,idx]));previous=body1
            wrist=body_pose(previous);fk=base.copy();fk[:3,3]+=axis*lift_home
            for j,q in zip(chain,home):
                r=np.eye(4);r[:3,:3]=Rotation.from_rotvec(np.eye(3)['XYZ'.index(j['axis'])]*q).as_matrix()
                fk=fk@transform(j['pos0'],j['rot0'])@r@np.linalg.inv(transform(j['pos1'],j['rot1']))
            error=float(np.linalg.norm(fk[:3,3]-wrist[:3,3]));angle=float(Rotation.from_matrix(fk[:3,:3]@wrist[:3,:3].T).magnitude())
            model['fk_validated'] &= error<.001 and angle<.01
            tips=[];radii=[]
            for suffix,sign in [('r2',1),('l2',-1)]:
                tipname=f'gripper_{letter}_rh_p12_rn_{suffix}'
                paths=[str(p.GetPath()) for p in Usd.PrimRange(rroot) if p.GetName()==tipname and p.HasAPI(UsdPhysics.RigidBodyAPI)]
                if len(paths)!=1:raise ValueError('Cannot resolve finger rigid body')
                path=paths[0];tip=body_pose(path);off=np.array([0,sign*.0045,.025])
                tips.append(tip[:3,3]+tip[:3,:3]@off);radius=float(np.linalg.norm(off));seen=set()
                while path!=previous:
                    if path in seen:raise ValueError('Cycle in finger joints')
                    seen.add(path);j=by_child[path]
                    radius+=float(np.linalg.norm(j.GetLocalPos0Attr().Get())+np.linalg.norm(j.GetLocalPos1Attr().Get()))
                    path=str(j.GetBody0Rel().GetTargets()[0])
                radii.append(radius)
            tcp=wrist.copy();tcp[:3,3]=np.mean(tips,axis=0)
            radius=sum(float(np.linalg.norm(np.array(chain[i+1]['pos0'])-np.array(chain[i]['pos1']))) for i in range(6))
            radius+=float(np.linalg.norm(chain[-1]['pos1']))
            shoulder=(base@np.array([*chain[0]['pos0'],1]))[:3]
            model['arms'][side]={'joints':chain,'home_joints':home,'parent_at_lift_zero':base.tolist(),
             'shoulder_at_lift_zero':shoulder.tolist(),'wrist_radius_upper_m':radius,'tcp_radius_upper_m':max(radii),
             'tcp_in_wrist':(np.linalg.inv(wrist)@tcp).tolist(),'fk_position_error_m':error,'fk_rotation_error_rad':angle}
        output['robot_model']=model
    except Exception as error:
        output['robot_model']={'fk_validated':False,'error':repr(error)}
    args.output.parent.mkdir(parents=True,exist_ok=True)
    with args.output.open('x') as f:json.dump(output,f,indent=2)
    completed=True
    print('Geometry evidence:',args.output,'model valid:',output['robot_model'].get('fk_validated'))
except Exception as error:
    output['status']='EVIDENCE_INCOMPLETE'
    output['error']=repr(error)
    # Persist missing evidence too, including target paths and per-prim reasons.
    # Never replace missing support with guessed heights or frame bounds.
    args.output.parent.mkdir(parents=True,exist_ok=True)
    if not args.output.exists():
        with args.output.open('x') as f:json.dump(output,f,indent=2)
    import traceback
    traceback.print_exc()
finally:
    if env is not None:env.close()
    if not completed:
        import sys,os
        sys.stdout.flush();sys.stderr.flush()
        os._exit(1)
    app.close()
