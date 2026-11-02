"""Static scenario geometry and model filters. No simulator imports or state writes."""
import itertools
import json
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
CFG = ROOT / 'source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/config/ffw_bg2'
OBJECTS = ('toaster', 'plate', 'bread_shelf')


def area(p):
    return abs(sum(a[0]*b[1]-b[0]*a[1] for a,b in zip(p,p[1:]+p[:1]))) / 2 if len(p)>2 else 0.0


def hull(points):
    pts = sorted(set(tuple(p[:2]) for p in points))
    if len(pts)<3:
        raise ValueError('Degenerate footprint')
    def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    def half(seq):
        out=[]
        for p in seq:
            while len(out)>1 and cross(out[-2],out[-1],p)<=0: out.pop()
            out.append(p)
        return out
    return half(pts)[:-1]+half(pts[::-1])[:-1]


def clip_edge(poly,a,b,inside=True):
    def value(p): return ((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])) * (1 if inside else -1)
    out=[]
    for p,q in zip(poly,poly[1:]+poly[:1]):
        vp,vq=value(p),value(q)
        if vp>=0: out.append(p)
        if (vp>=0)!=(vq>=0):
            t=vp/(vp-vq); out.append((p[0]+t*(q[0]-p[0]),p[1]+t*(q[1]-p[1])))
    return out


def subtract(poly,convex):
    """Disjoint pieces of poly outside a CCW convex polygon."""
    pieces=[]; remaining=poly
    for a,b in zip(convex,convex[1:]+convex[:1]):
        if not remaining: break
        outside=clip_edge(remaining,a,b,False)
        if area(outside)>1e-12: pieces.append(outside)
        remaining=clip_edge(remaining,a,b)
    return pieces


def supported(poly,surfaces):
    remaining=[poly]
    for surface in surfaces:
        remaining=[piece for p in remaining for piece in subtract(p,surface)]
        if not remaining: return True
    return sum(area(p) for p in remaining)<1e-9


def polygon_distance(a,b):
    # SAT first; overlapping/contained polygons have zero clearance.
    separated=False
    for poly in (a,b):
        for p,q in zip(poly,poly[1:]+poly[:1]):
            axis=(p[1]-q[1],q[0]-p[0])
            aa=[x*axis[0]+y*axis[1] for x,y in a]; bb=[x*axis[0]+y*axis[1] for x,y in b]
            if max(aa)<min(bb) or max(bb)<min(aa): separated=True
    if not separated: return 0.0
    def distance(p,v,w):
        d=[w[i]-v[i] for i in (0,1)]; denom=sum(x*x for x in d)
        t=max(0,min(1,sum((p[i]-v[i])*d[i] for i in (0,1))/denom)) if denom else 0
        return math.hypot(*(p[i]-v[i]-t*d[i] for i in (0,1)))
    return min(distance(p,v,w) for aa,bb in ((a,b),(b,a)) for p in aa for v,w in zip(bb,bb[1:]+bb[:1]))


def qmul(a,b):
    w,x,y,z=a; v,i,j,k=b
    return [w*v-x*i-y*j-z*k,w*i+x*v+y*k-z*j,w*j-x*k+y*v+z*i,w*k+x*j-y*i+z*v]


def rotate(q,p):
    n=sum(x*x for x in q); q=[x/math.sqrt(n) for x in q]
    return qmul(qmul(q,[0,*p]),[q[0],-q[1],-q[2],-q[3]])[1:]


def apply_point(pose,point):
    v=rotate(pose['quaternion_wxyz'],point)
    return [a+b for a,b in zip(pose['position_m'],v)]


def footprint(vertices,pose,scale=(1,1,1)):
    return hull([apply_point(pose,[p[i]*scale[i] for i in range(3)]) for p in vertices])


def task_probes(poses):
    """TCP position probes and heuristic wrist orientations, NOT validated grasps.

    Tool axes parallel to environment axes at serving; toaster-local axes at
    insertion/lever. Multi-start IK is a diagnostic, not an expert trajectory.
    """
    result=[]
    for i in (0,1):
        p=poses[f'bread_{i}']; result.append({'name':f'grasp_{i}', 'position':apply_point(p,[-.035,0,.0056]),'quaternion':p['quaternion_wxyz']})
        for stage,z in (('insert',.120),('extract',.18)):
            p=poses['toaster']; result.append({'name':f'{stage}_{i}','position':apply_point(p,[-.003,(-.026455,.0253585)[i],z]),'quaternion':p['quaternion_wxyz']})
    p=poses['toaster']; result.append({'name':'lever','position':apply_point(p,[.125,0,0]),'quaternion':p['quaternion_wxyz']})
    p=poses['plate']; result.append({'name':'serve','position':apply_point(p,[0,0,.10]),'quaternion':[1,0,0,0]})
    return result


def model_filter(poses,evidence):
    model=(evidence or {}).get('robot_model')
    if not model or not model.get('fk_validated'):
        return {'status':'NOT_RUN','reason':'Missing USD model with FK consistency check','rejected':False}
    results=[]
    for probe in task_probes(poses):
        arms={}
        for side,arm in model['arms'].items():
            # Distance to shoulder's lift segment vs conservative chain-length bound.
            p=probe['position']; a=arm['shoulder_at_lift_zero']; axis=model['lift_axis_world']
            t=sum((p[i]-a[i])*axis[i] for i in range(3))
            t=max(model['lift_limits'][0],min(model['lift_limits'][1],t))
            d=math.sqrt(sum((p[i]-a[i]-t*axis[i])**2 for i in range(3)))
            bound=arm['wrist_radius_upper_m']+arm['tcp_radius_upper_m']
            arms[side]={'distance_to_lift_segment_m':d,'radius_upper_m':bound,'outside_upper_bound':d>bound+1e-6}
        results.append({'probe':probe['name'],'arms':arms})
    excluded=any(all(a['outside_upper_bound'] for a in p['arms'].values()) for p in results)
    return {'status':'OUTSIDE_MODEL_UPPER_BOUND' if excluded else 'NOT_EXCLUDED_BY_MODEL_BOUND',
            'rejected':excluded,'probes':results,'reachability':'NOT_PROVEN',
            'limitation':'Only excludes these probe points for a fixed robot base; not a collision-aware reachability proof.'}


def ik_probes(poses,evidence):
    """Bounded numerical FK/IK on exported USD joints; no simulator required."""
    import numpy as np
    from scipy.optimize import least_squares
    from scipy.spatial.transform import Rotation
    model=evidence['robot_model']
    if not model.get('fk_validated'): return {'status':'NOT_RUN','reason':'FK consistency check failed'}
    def transform(p,q):
        t=np.eye(4); t[:3,:3]=Rotation.from_quat([*q[1:],q[0]]).as_matrix();t[:3,3]=p;return t
    result=[]
    for probe in task_probes(poses):
        arms={}
        for side,arm in model['arms'].items():
            target=transform(probe['position'],probe['quaternion'])
            # Approximate wrist goal from measured OPEN-gripper TCP transform.
            target=target@np.linalg.inv(np.asarray(arm['tcp_in_wrist']))
            def fk(q):
                t=np.asarray(arm['parent_at_lift_zero']).copy();t[:3,3]+=np.asarray(model['lift_axis_world'])*q[-1]
                for joint,angle in zip(arm['joints'],q[:-1]):
                    r=np.eye(4);axis=np.eye(3)['XYZ'.index(joint['axis'])]
                    r[:3,:3]=Rotation.from_rotvec(axis*angle).as_matrix()
                    t=t@transform(joint['pos0'],joint['rot0'])@r@np.linalg.inv(transform(joint['pos1'],joint['rot1']))
                return t
            def error(q):
                t=fk(q)
                return np.r_[t[:3,3]-target[:3,3], .2*Rotation.from_matrix(target[:3,:3]@t[:3,:3].T).as_rotvec()]
            lo=np.array([j['limits_rad'][0] for j in arm['joints']]+[model['lift_limits'][0]])
            hi=np.array([j['limits_rad'][1] for j in arm['joints']]+[model['lift_limits'][1]])
            seeds=[np.array(arm['home_joints']+[0.0]),(lo+hi)/2]
            rng=np.random.default_rng(240311);seeds += [rng.uniform(lo,hi) for _ in range(2)]
            best=None
            for seed in seeds:
                opt=least_squares(error,np.clip(seed,lo+1e-8,hi-1e-8),bounds=(lo,hi),max_nfev=150)
                e=error(opt.x); row={'position_error_m':float(np.linalg.norm(e[:3])),
                    'orientation_error_rad':float(np.linalg.norm(e[3:])/.2),'joints_and_lift':opt.x.tolist()}
                if best is None or np.linalg.norm(e)<best[0]:best=(float(np.linalg.norm(e)),row)
                if row['position_error_m']<.005 and row['orientation_error_rad']<.05:break
            row=best[1];row['status']='IK_FOUND' if row['position_error_m']<.005 and row['orientation_error_rad']<.05 else 'IK_NOT_FOUND'
            arms[side]=row
        result.append({'probe':probe['name'],'arms':arms})
    return {'status':'PROBES_CHECKED','probes':result,'reachability':'NOT_PROVEN',
        'limitations':['Heuristic target orientations/open-gripper TCP; not validated task grasps',
                      'Independent arm/lift solutions; no shared-lift bimanual coordination or path/collision test',
                      'IK_NOT_FOUND is not physical impossibility']}
