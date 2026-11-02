"""Audit existing fixed poses against new USD evidence without changing any pose/ID.

Output is independent of the immutable manifest. NOT_EXCLUDED is not task PASS.
"""
import argparse
import hashlib
import itertools
import json
from pathlib import Path
from toast_batch_lib import ROOT, CFG, OBJECTS, footprint, hull, supported, polygon_distance, model_filter, ik_probes


def audit(manifest_path, evidence_path, output, run_ik=False):
    if output.exists():
        raise ValueError('Choose a new audit output file')
    raw=manifest_path.read_bytes(); manifest=json.loads(raw)
    evidence=json.loads(evidence_path.read_bytes())
    def digest(p):return hashlib.sha256(p.read_bytes()).hexdigest()
    if manifest['schema_version']!=2 or evidence['task']!=manifest['task']:
        raise ValueError('Wrong schema or task')
    for key,path in [('robot_usd_sha256',ROOT/'source/cyclo_lab/data/robots/FFW/FFW_BG2_without_mimic.usd'),
                     ('asset_manifest_sha256',CFG.parent.parent/'asset_manifest.json')]:
        if evidence[key]!=digest(path):raise ValueError('Evidence asset mismatch: '+key)
    if run_ik and not evidence.get('robot_model',{}).get('fk_validated'):
        raise ValueError('--ik requires FK-validated model evidence')
    rows=[]
    for scenario in manifest['scenarios']:
        rejected=[];missing=[];checks={};poses=scenario['poses']
        surfaces=evidence.get('table_support',{}).get('convex_surface_polygons_xy',[])
        if surfaces:
            margin=manifest['generation']['table_edge_margin_m']
            cover={name:supported(hull([(x+dx,y+dy) for x,y in poly
                      for dx,dy in itertools.product((-margin,margin),repeat=2)]),surfaces)
                   for name,poly in scenario['footprints_xy'].items()}
            checks['table_support']={'kind':evidence['table_support']['kind'],'covered':cover}
            if not all(cover.values()):rejected.append('outside_extracted_support_surface')
        else:missing.append('table_support')
        colliders=evidence.get('object_colliders',{})
        if all(colliders.get(n,{}).get('scaled_local_vertices') for n in OBJECTS):
            polygons={n:footprint(colliders[n]['scaled_local_vertices'],poses[n]) for n in OBJECTS}
            gaps={a+'/'+b:polygon_distance(polygons[a],polygons[b]) for a,b in itertools.combinations(OBJECTS,2)}
            checks['authored_collision_bounds']={'gaps_m':gaps,'scope':'authored mesh bounds, NOT cooked or moving-part collision validation'}
            if min(gaps.values())<manifest['generation']['minimum_object_gap_m']:rejected.append('authored_collision_bound_gap')
        else:missing.append('authored_collision_bounds')
        checks['model_prefilter']=model_filter(poses,evidence)
        if checks['model_prefilter']['status']=='NOT_RUN':missing.append('robot_model')
        if checks['model_prefilter']['rejected']:rejected.append('outside_robot_model_bound')
        checks['plate_x_conflict']=poses['plate']['position_m'][0]<=.419
        if checks['plate_x_conflict']:rejected.append('plate_x_success_conflict')
        checks['ik']=ik_probes(poses,evidence) if run_ik else {'status':'NOT_RUN'}
        rows.append({'id':scenario['id'],'status':'REJECTED_BY_FILTER' if rejected else ('EVIDENCE_INCOMPLETE' if missing else 'NOT_EXCLUDED'),
                     'rejection_reasons':rejected,'missing_evidence':missing,'checks':checks,
                     'reachability':'NOT_PROVEN','runtime_layout_reset':'NOT_TESTED','task_success':'NOT_TESTED'})
    result={'manifest_sha256':hashlib.sha256(raw).hexdigest(),'evidence_sha256':digest(evidence_path),
            'results':rows,'eligible_for_layout_test':[r['id'] for r in rows if r['status']=='NOT_EXCLUDED'],
            'note':'Eligibility is a prefilter only. IK_NOT_FOUND is not physical impossibility; IK_FOUND is not collision-free task feasibility.'}
    output.parent.mkdir(parents=True,exist_ok=True)
    with output.open('x') as f:json.dump(result,f,indent=2);f.write('\n')
    return result


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__)
    p.add_argument('--manifest',type=Path,default=ROOT/'scenarios/make_toast_v2_batch01/manifest.json')
    p.add_argument('--evidence',type=Path,required=True);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--ik',action='store_true');a=p.parse_args()
    r=audit(a.manifest,a.evidence,a.output,a.ik)
    from collections import Counter
    print(dict(Counter(x['status'] for x in r['results'])))
