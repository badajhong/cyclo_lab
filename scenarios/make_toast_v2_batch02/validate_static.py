"""Read-only static validation of this delivery. Does not launch Isaac Sim."""
import ast
from collections import Counter
import csv
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import sys
from types import SimpleNamespace as NS

ROOT=Path(__file__).resolve().parents[2]
HERE=Path(__file__).resolve().parent
sys.dont_write_bytecode=True
sys.path.insert(0,str(ROOT/'scripts/tools'))
from toast_batch_lib import footprint, hull, polygon_distance


def validate():
    read=lambda p:json.loads(p.read_text())
    digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
    m=read(HERE/'manifest.json');audit=read(HERE/'audit.json')
    old=read(ROOT/'scenarios/make_toast_v2_batch01/manifest.json')
    evidence=ROOT/m['provenance']['geometry_evidence']['path'];g=read(evidence)
    assert digest(HERE/'manifest.json')==audit['manifest_sha256']
    assert digest(evidence)==audit['evidence_sha256']==m['provenance']['geometry_evidence']['sha256']
    assert len(m['scenarios'])==100
    ids=[s['id'] for s in m['scenarios']]
    assert len(set(ids))==100 and not(set(ids)&{s['id'] for s in old['scenarios']})
    assert len({s['pose_signature_sha256'] for s in m['scenarios']})==100
    assert not({s['pose_signature_sha256'] for s in m['scenarios']}&{s['pose_signature_sha256'] for s in old['scenarios']})
    assert all(s['status']=='NOT_EXCLUDED' for s in audit['results']) and {s['id'] for s in audit['results']}==set(ids)
    counts=Counter(s['operator'] for s in m['scenarios']);assert counts=={'operator_A':50,'operator_B':50}
    groups={tuple(s['order_positive_to_negative_y']) for s in m['scenarios']};assert len(groups)==6
    distribution=[]
    for group in sorted(groups):
        c=Counter(s['operator'] for s in m['scenarios'] if tuple(s['order_positive_to_negative_y'])==group)
        assert abs(c['operator_A']-c['operator_B'])<=1
        distribution.append({'order':group,'operators':dict(c)})
    assert sorted(sum(x['operators'].values()) for x in distribution)==[16,16,17,17,17,17]
    rows=list(csv.DictReader((HERE/'assignments.csv').open()))
    assert {r['scenario_id']:r['operator'] for r in rows}=={s['id']:s['operator'] for s in m['scenarios']}
    for key in ['table_edge_margin_m','metadata_rounding_margin_m','minimum_object_gap_m','approach_gap_m','yaw_jitter_degrees','y_anchors_m']:
        assert m['generation'][key]==old['generation'][key]
    path=ROOT/'source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/config/ffw_bg2/scenarios.py'
    spec=importlib.util.spec_from_file_location('scenario_config',path);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    min_edge=math.inf;min_visual_gap=math.inf
    for s in m['scenarios']:
        scene=NS()
        for name,pose in {**m['baseline'],**m['baseline_bread']}.items():
            setattr(scene,name,NS(init_state=NS(pos=tuple(pose['position_m']),rot=tuple(pose['quaternion_wxyz'])),spawn=NS(scale=tuple(m['scales'].get(name,[1,1,1])))))
        cfg=NS(scene=scene,serve_to_plate=True,scenario_manifest=str(HERE/'manifest.json'))
        module.apply_scenario(cfg,s['id'])  # Checks unchanged height/scale and rigid bread-relative poses.
        module.apply_scenario(cfg,s['id'])  # Idempotency, not a simulator reset.
        for name,p in s['poses'].items():assert tuple(p['position_m'])==getattr(scene,name).init_state.pos
        polygons={}
        for name in ['toaster','plate','bread_shelf']:
            pose=s['poses'][name];x,y,z=pose['position_m'];rank=s['order_positive_to_negative_y'].index(name)
            assert abs(x-({'toaster':.55,'plate':.49,'bread_shelf':.46}[name]))<=({'plate':.01}.get(name,.015))+1e-12
            assert abs(y-[.4,0,-.4][rank])<=.015+1e-12
            q=pose['quaternion_wxyz'];b=m['baseline'][name]['quaternion_wxyz']
            dot=abs(sum(a*c for a,c in zip(q,b)))/math.sqrt(sum(a*a for a in q)*sum(a*a for a in b))
            assert 2*math.acos(min(1,dot))<=math.radians(5)+1e-8
            meta=read(ROOT/f'source/cyclo_lab/data/objects/robodojo_make_toast/{name}/metadata.json')
            poly=footprint(meta['geometry']['aligned_bbox']['vertices'],pose,pose['scale'])
            poly=hull([(px+dx,py+dy) for px,py in poly for dx in [-.001,.001] for dy in [-.001,.001]])
            if name=='bread_shelf':
                bread=read(ROOT/'source/cyclo_lab/data/objects/robodojo_make_toast/bread/metadata.json')
                poly=hull(poly+[v for i in range(4) for v in footprint(bread['geometry']['aligned_bbox']['vertices'],s['poses'][f'bread_{i}'])])
            assert json.loads(json.dumps(poly))==s['footprints_xy'][name]
            polygons[name]=poly
            # Independent half-plane test: this evidence contains one convex top face.
            assert len(g['table_support']['convex_surface_polygons_xy'])==1
            surface=g['table_support']['convex_surface_polygons_xy'][0]
            expanded=[(px+dx,py+dy) for px,py in poly for dx in [-.02,.02] for dy in [-.02,.02]]
            clearance=min(((v[0]-u[0])*(py-u[1])-(v[1]-u[1])*(px-u[0]))/math.hypot(v[0]-u[0],v[1]-u[1]) for u,v in zip(surface,surface[1:]+surface[:1]) for px,py in expanded)
            assert clearance>=0;min_edge=min(min_edge,clearance)
        import itertools
        min_visual_gap=min(min_visual_gap,*(polygon_distance(polygons[a],polygons[b]) for a,b in itertools.combinations(polygons,2)))
        assert s['checks']['reachability']=='NOT_PROVEN' and s['checks']['runtime_layout_reset']=='NOT_TESTED'
    assert min_visual_gap>=.06
    preserved=read(HERE/'preservation_before.json')['sha256']
    assert all(digest(ROOT/p)==h for p,h in preserved.items())
    assert m['summary']['gpu_verified']==0
    ast.parse((ROOT/'scripts/tools/generate_make_toast_scenarios.py').read_text())
    return {'status':'PASS','scope':'CPU only; no simulation or pilot rerun','candidates':100,'audit_NOT_EXCLUDED':100,
            'operators':dict(counts),'permutations':distribution,'protected_files_unchanged':len(preserved),
            'minimum_extra_table_edge_clearance_after_20mm_m':min_edge,'minimum_visual_gap_m':min_visual_gap,
            'simulator_layout_reset_verified':0,'reachability':'NOT_PROVEN','ik':'NOT_RUN','task_success':'NOT_TESTED'}

if __name__=='__main__':print(json.dumps(validate(),indent=2))
