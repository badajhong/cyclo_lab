"""Generate immutable v2 candidate poses, assignments and a self-contained preview.

Without --evidence, outputs are explicitly provisional: tabletop/collision/model
checks are NOT_RUN. Never converts a distance bound or IK result into task PASS.
"""
import argparse
import collections
import hashlib
import itertools
import json
import math
from pathlib import Path
import random
import subprocess
from toast_batch_lib import (ROOT,CFG,OBJECTS,hull,footprint,polygon_distance,supported,
                             qmul,rotate,model_filter,ik_probes)


def sha(p): return hashlib.sha256(p.read_bytes()).hexdigest()

def generate(output,seed=20261102,evidence_path=None,check_ik=False,max_tries=500,profile="batch01"):
    if profile not in ("batch01","batch02"):raise ValueError("Unknown generation profile")
    targets=['manifest.json','assignments.csv','preview.html','generation_report.json']
    if any((output/f).exists() for f in targets): raise ValueError('Immutable delivery already exists; choose a new output directory')
    pilots=json.loads((CFG/'scenario_pilots.json').read_text())
    pilot_dir=ROOT/'logs/scenario_pilots.mlY0sM'
    verified={}
    for name in ('baseline','small_offset','swapped_candidate'):
        p=pilot_dir/name/'report.json';report=json.loads(p.read_text())
        if report.get('runtime_reset')!='PASS':raise ValueError('Expected prior pilot verification')
        verified[name]={'path':str(p.relative_to(ROOT)),'sha256':sha(p),'scope':'initial poses and two resets only; reused, not rerun'}
    baseline_report=json.loads((pilot_dir/'baseline/report.json').read_text())
    state=next(s['state'] for s in baseline_report['samples'] if s['label']=='initial')
    baseline_bread={f'bread_{i}':{'position_m':state[f'bread_{i}/root'][0][:3],
      'quaternion_wxyz':state[f'bread_{i}/root'][0][3:7]} for i in range(4)}
    asset_dir=ROOT/'source/cyclo_lab/data/objects/robodojo_make_toast'
    asset_manifest_path=CFG.parent.parent/'asset_manifest.json'
    pinned=json.loads(asset_manifest_path.read_text())
    expected={x['path']:x['sha256'] for x in pinned['files']}
    vertices={};hashes={}
    for name in (*OBJECTS,'bread'):
        p=asset_dir/name/'metadata.json';hashes[str(p.relative_to(ROOT))]=sha(p)
        if sha(p)!=expected[name+'/metadata.json']:raise ValueError('Pinned metadata mismatch: '+name)
        vertices[name]=json.loads(p.read_text())['geometry']['aligned_bbox']['vertices']
    evidence=json.loads(evidence_path.read_text()) if evidence_path else None
    robot_file=ROOT/'source/cyclo_lab/data/robots/FFW/FFW_BG2_without_mimic.usd'
    if evidence:
        if evidence['robot_usd_sha256']!=sha(robot_file) or evidence['asset_manifest_sha256']!=sha(asset_manifest_path):
            raise ValueError('Geometry evidence does not match local assets')
        if evidence['task']!='Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2':raise ValueError('Wrong evidence task')
    if profile=='batch02':
        if not evidence or evidence.get('table_support',{}).get('status')!='EXTRACTED' or not evidence['table_support'].get('convex_surface_polygons_xy'):
            raise ValueError('batch02 requires extracted support evidence')
        if not all(evidence.get('object_colliders',{}).get(n,{}).get('scaled_local_vertices') for n in OBJECTS):
            raise ValueError('batch02 requires all three authored collider bounds')
        if not evidence.get('robot_model',{}).get('fk_validated'):
            raise ValueError('batch02 requires FK-validated robot model')
    if check_ik and not (evidence and evidence.get('robot_model',{}).get('fk_validated')):
        raise ValueError('--ik requires FK-validated model evidence')
    # Design choices, not physical feasibility claims. Y ranks use +Y to -Y.
    config={'seed':seed,'target_count':100,'max_tries_per_slot':max_tries,
      'order_frame':'environment frame, descending Y (+Y/robot left to -Y/robot right); not camera left/right',
      'y_anchors_m':[.40,0,-.40],'x_anchors_m':{'toaster':.55,'plate':.46,'bread_shelf':.46},
      'xy_jitter_m':.015,'yaw_jitter_degrees':5,'table_edge_margin_m':.02,
      'metadata_rounding_margin_m':.001,'minimum_object_gap_m':.03,'approach_gap_m':.06,
      'reasoning':{'y_spacing':'0.40 m exceeds largest scaled half-width pair (~0.16+0.13), 30 mm combined jitter and 60 mm planar access gap; filter checks actual rotated footprints',
       'x_anchors':'Near calibrated object X; plate +30 mm and shelf +10 mm give table-edge margin after yaw/jitter',
       'approach_gap':'60 mm planar corridor proxy: 106 mm expert full-jaw opening / 2 + 7 mm margin; NOT a swept arm/hand clearance proof',
       'table':'20 mm erosion of measured frame envelope is only a planning bound until actual support-surface evidence is supplied',
       'yaw':'Pilot verified +/-2 degrees; +/-5 degrees is a new candidate range, not runtime validated',
       'height':'Original cfg root Z and scale unchanged; normal physical settling is unchanged'}}
    config['profile']=profile
    config['id_prefix']='batch02_toast_' if profile=='batch02' else 'toast_'
    config['x_jitter_m']={n:.015 for n in OBJECTS}
    if profile=='batch02':
        config['x_anchors_m']['plate']=.490
        config['x_jitter_m']['plate']=.010
        config['reasoning']['x_anchors']='batch02: plate X=0.490 +/-0.010 m; worst-case +/-5deg bbox needs X>=0.468160 m for 20mm edge margin on extracted tabletop; other anchors unchanged'
        config['reasoning']['table']='Retain planning-envelope filter AND require coverage by extracted support faces with unchanged 20mm margin'
    rng=random.Random(seed);orders=list(itertools.permutations(OBJECTS));quota=[17,17,17,17,16,16]
    accepted=[];rejections=collections.Counter();permutation_stats=[];signatures=set()
    def expand(poly,r):return hull([(x+dx,y+dy) for x,y in poly for dx,dy in itertools.product((-r,r),repeat=2)])
    for order,count in zip(orders,quota):
        before=len(accepted); reasons=collections.Counter();attempts=0
        for _ in range(count):
            for trial in range(max_tries):
                attempts+=1;poses={};angles={}
                for rank,name in enumerate(order):
                    ref=pilots['baseline'][name];yaw=math.radians(rng.uniform(-5,5));angles[name]=yaw
                    poses[name]={'position_m':[config['x_anchors_m'][name]+rng.uniform(-config['x_jitter_m'][name],config['x_jitter_m'][name]),config['y_anchors_m'][rank]+rng.uniform(-.015,.015),ref['position_m'][2]],
                       'quaternion_wxyz':qmul([math.cos(yaw/2),0,0,math.sin(yaw/2)],ref['quaternion_wxyz']),
                       'scale':pilots['scales'].get(name,[1,1,1])}
                old=pilots['baseline']['bread_shelf']['position_m'];new=poses['bread_shelf']['position_m'];yaw=angles['bread_shelf'];delta=[math.cos(yaw/2),0,0,math.sin(yaw/2)]
                for name,ref in baseline_bread.items():
                    offset=rotate(delta,[a-b for a,b in zip(ref['position_m'],old)])
                    poses[name]={'position_m':[a+b for a,b in zip(new,offset)],'quaternion_wxyz':qmul(delta,ref['quaternion_wxyz']),'scale':[1,1,1]}
                polygons={name:expand(footprint(vertices[name],poses[name],poses[name]['scale']),.001) for name in OBJECTS}
                # Shelf + four breads form one layout group. Intended support contacts are not rejected.
                polygons['bread_shelf']=hull(polygons['bread_shelf']+[p for i in range(4) for p in footprint(vertices['bread'],poses[f'bread_{i}'])])
                checks={'collision_geometry':'NOT_RUN','tabletop_support':'NOT_RUN','access_swept_volume':'NOT_TESTED',
                    'reachability':'NOT_PROVEN','ik':{'status':'NOT_RUN'},'runtime_layout_reset':'NOT_TESTED','task_success':'NOT_TESTED',
                    'plate_x_condition':{'threshold_m':.419,'conflict':poses['plate']['position_m'][0]<=.419}}
                failures=[]
                lo,hi=pilots['table_envelope']
                if any(not(lo[0]+.02<=x<=hi[0]-.02 and lo[1]+.02<=y<=hi[1]-.02) for poly in polygons.values() for x,y in poly):failures.append('outside_table_planning_envelope')
                gaps={a+'/'+b:polygon_distance(polygons[a],polygons[b]) for a,b in itertools.combinations(OBJECTS,2)}
                if min(gaps.values())<.03:failures.append('visual_overlap_or_minimum_gap')
                elif min(gaps.values())<.06:failures.append('planar_approach_gap')
                checks['visual_footprint']={'kind':'scaled/rotated pinned metadata boxes + 1mm rounding margin; shelf includes all bread boxes',
                                            'pair_gaps_m':gaps,'status':'PASS' if not failures else 'FAIL'}
                if evidence:
                    support=evidence.get('table_support',{});surfaces=support.get('convex_surface_polygons_xy',[])
                    if surfaces:
                        valid=all(supported(expand(poly,.02),surfaces) for poly in polygons.values())
                        checks['tabletop_support']={'kind':support['kind'],'status':'COVERED' if valid else 'NOT_COVERED'}
                        if not valid:failures.append('outside_extracted_support_surface')
                    colliders=evidence.get('object_colliders',{})
                    if all(colliders.get(n,{}).get('scaled_local_vertices') for n in OBJECTS):
                        cp={n:footprint(colliders[n]['scaled_local_vertices'],poses[n]) for n in OBJECTS}
                        cg={a+'/'+b:polygon_distance(cp[a],cp[b]) for a,b in itertools.combinations(OBJECTS,2)}
                        checks['collision_geometry']={'kind':'authored reset-pose mesh bounding hulls; NOT cooked/swept collisions','pair_gaps_m':cg,'status':'SEPARATED' if min(cg.values())>=.03 else 'REJECTED_BY_BOUND'}
                        if min(cg.values())<.03:failures.append('authored_collision_bound_gap')
                checks['model_prefilter']=model_filter(poses,evidence)
                if checks['model_prefilter']['rejected']:failures.append('outside_robot_model_bound')
                if checks['plate_x_condition']['conflict']:failures.append('plate_x_success_conflict')
                signature=json.dumps({n:[*[round(x,6) for x in p['position_m']],*[round(x,7) for x in p['quaternion_wxyz']]] for n,p in sorted(poses.items())},sort_keys=True)
                if signature in signatures:failures.append('duplicate_pose')
                if failures:
                    reasons.update(failures);rejections.update(failures);continue
                if check_ik:checks['ik']=ik_probes(poses,evidence)
                # No solution to heuristic IK probes means review, not physical impossibility.
                signatures.add(signature)
                accepted.append({'id':f"{config['id_prefix']}{len(accepted):03d}",'order_positive_to_negative_y':list(order),
                  'status':'CANDIDATE_NOT_GPU_VERIFIED','poses':poses,'footprints_xy':polygons,
                  'checks':checks,'pose_signature_sha256':hashlib.sha256(signature.encode()).hexdigest()})
                break
        permutation_stats.append({'order':list(order),'requested':count,'obtained':len(accepted)-before,'attempts':attempts,'rejection_reasons':dict(reasons)})
    # Alternate which operator receives the extra item in odd-sized groups.
    counts=collections.Counter();odd=0;rows=[]
    for order in orders:
        group=[s for s in accepted if s['order_positive_to_negative_y']==list(order)]
        start=odd%2
        if len(group)%2:odd+=1
        for i,s in enumerate(group):
            operator=('operator_A','operator_B')[(i+start)%2];s['operator']=operator;counts[operator]+=1
            rows.append([s['id'],operator,'>'.join(order),s['status']])
    if len(accepted)==100:assert counts=={'operator_A':50,'operator_B':50}
    source_files=[Path(__file__),ROOT/'scripts/tools/toast_batch_lib.py',CFG/'scenarios.py',CFG/'bimanual_env.py',CFG/'bimanual_env_cfg.py',CFG/'bimanual_mdp.py',CFG/'assets.py']
    manifest={'schema_version':2,'task':'Cyclo-Make-Toast-FFW-BG2-Bimanual-IK-Rel-v2','status':'CANDIDATES_NOT_GPU_VERIFIED',
      'units':'metres, radians, quaternion wxyz; environment-relative poses',
      'baseline':pilots['baseline'],'scales':pilots['scales'],'baseline_bread':baseline_bread,
      'generation':config,'provenance':{'repository_commit':subprocess.check_output(['git','rev-parse','HEAD'],cwd=ROOT,text=True).strip(),
        'working_tree_status':subprocess.check_output(['git','status','--short'],cwd=ROOT,text=True),
        'source_sha256':{str(p.relative_to(ROOT)):sha(p) for p in source_files},'metadata_sha256':hashes,
        'robot_usd_sha256':sha(robot_file),'asset_revision':pinned['revision'],'pilot_verifications':verified,
        'geometry_evidence':{'path':str(evidence_path),'sha256':sha(evidence_path)} if evidence_path else None},
      'extracted_table_support':evidence.get('table_support') if evidence else None,
      'planning_table_envelope':pilots['table_envelope'],'permutations':permutation_stats,'scenarios':accepted,
      'summary':{'target':100,'candidates':len(accepted),'gpu_verified':0,'task_validated':0,'assignments':dict(counts),'rejections':dict(rejections),
       'missing_evidence':[] if evidence else ['usable tabletop support polygons','authored/cooked collision geometry','USD kinematic model; reachability and IK not run'],
       'note':'No failed criterion was relaxed. Six permutations considered; acceptance is only within stated static/provisional checks.'}}
    output.mkdir(parents=True,exist_ok=True)
    with (output/'manifest.json').open('x') as f:json.dump(manifest,f,indent=2);f.write('\n')
    import csv
    with (output/'assignments.csv').open('x',newline='') as f:
        writer=csv.writer(f);writer.writerow(['scenario_id','operator','order_positive_to_negative_y','status']);writer.writerows(rows)
    with (output/'generation_report.json').open('x') as f:json.dump({'summary':manifest['summary'],'permutations':permutation_stats},f,indent=2)
    preview(output,manifest)
    return manifest


def preview(output,manifest):
    import html
    cards=[];colors={'toaster':'#e08743','plate':'#3687b5','bread_shelf':'#65a675'}
    for s in manifest['scenarios']:
        # +X upward, +Y left; robot is below the table in this schematic.
        def xy(p):return (160-p[1]*180,240-p[0]*200)
        polygons=[]
        for n,poly in s['footprints_xy'].items():
            points=' '.join(f'{xy(p)[0]:.2f},{xy(p)[1]:.2f}' for p in poly)
            polygons.append(f'<polygon points="{points}" fill="{colors[n]}" fill-opacity=".65" stroke="{colors[n]}"/>')
        lo,hi=manifest['planning_table_envelope'];x,y=xy([hi[0],hi[1]])
        support_polygons=[]
        for poly in (manifest.get('extracted_table_support') or {}).get('convex_surface_polygons_xy',[]):
            points=' '.join(f'{xy(p)[0]:.2f},{xy(p)[1]:.2f}' for p in poly)
            support_polygons.append(f'<polygon points="{points}" fill="none" stroke="#222" stroke-width="2"/>')
        svg=f'<svg viewBox="-70 0 470 260"><rect x="{x}" y="{y}" width="{(hi[1]-lo[1])*180}" height="{(hi[0]-lo[0])*200}" fill="#eee" stroke="#999" stroke-dasharray="4"/>'+''.join(support_polygons)+''.join(polygons)+'</svg>'
        cards.append(f'<article data-operator="{s["operator"]}"><h3>{s["id"]} · {s["operator"]}</h3>{svg}<div>{html.escape(" > ".join(s["order_positive_to_negative_y"]))}</div><small>CANDIDATE · reachability not proven</small></article>')
    page='''<!doctype html><meta charset="utf-8"><title>Toast fixed candidates</title>
<style>body{font:15px system-ui;margin:24px;background:#fafafa}main{display:grid;grid-template-columns:repeat(auto-fit,minmax(300px,1fr));gap:14px}article{padding:12px;background:white;border:1px solid #ccc}svg{width:100%}small{color:#944}select{margin:12px}</style>
<h1>Toast v2 — fixed candidates, NOT GPU verified</h1>
<p>+X up, +Y left (robot left); schematic robot below. Orange: toaster; blue: plate; green: shelf + bread group. Dashed rectangle is the measured table-frame envelope, NOT verified support area. Where present, the solid black outline is the extracted support surface. Layout/reset remains untested.</p>
<label>Operator <select id="filter"><option value="">All</option><option>operator_A</option><option>operator_B</option></select></label><main>'''+''.join(cards)+'''</main><script>document.getElementById('filter').onchange=function(){for(const el of document.querySelectorAll('article'))el.hidden=!!this.value&&el.dataset.operator!==this.value;};</script>'''
    with (output/'preview.html').open('x') as f:f.write(page)


if __name__=='__main__':
    p=argparse.ArgumentParser(description=__doc__);p.add_argument('--output',type=Path,required=True)
    p.add_argument('--seed',type=int,default=20261102);p.add_argument('--evidence',type=Path);p.add_argument('--ik',action='store_true')
    p.add_argument('--profile',choices=['batch01','batch02'],default='batch01')
    p.add_argument('--max_tries',type=int,default=500);a=p.parse_args()
    if a.max_tries<1:p.error('--max_tries must be positive')
    result=generate(a.output,a.seed,a.evidence,a.ik,a.max_tries,a.profile);print(json.dumps(result['summary'],indent=2))
