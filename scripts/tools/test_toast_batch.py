"""CPU tests for fixed-pose delivery, geometry and failure isolation. No simulation."""
import copy
import json
import math
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from types import SimpleNamespace as NS
from test_make_toast_scenarios import m, config_fixture
from toast_batch_lib import ROOT, CFG, OBJECTS, hull, supported, polygon_distance, model_filter, footprint
from generate_make_toast_scenarios import generate

MANIFEST=ROOT/'scenarios/make_toast_v2_batch01/manifest.json'

class BatchTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):cls.manifest=json.loads(MANIFEST.read_text())

    def fixture(self):
        cfg=config_fixture()
        for i in range(4):getattr(cfg.scene,f'bread_{i}').spawn=NS(scale=(1,1,1))
        cfg.scenario_manifest=str(MANIFEST)
        return cfg

    def test_fixed_100_and_balanced_unique_orders(self):
        from collections import Counter
        entries=self.manifest['scenarios']
        self.assertEqual(len(entries),100)
        self.assertEqual(len({s['pose_signature_sha256'] for s in entries}),100)
        self.assertEqual(Counter(s['operator'] for s in entries),{'operator_A':50,'operator_B':50})
        groups={tuple(s['order_positive_to_negative_y']) for s in entries}
        self.assertEqual(len(groups),6)
        for order in groups:
            counts=Counter(s['operator'] for s in entries if tuple(s['order_positive_to_negative_y'])==order)
            self.assertLessEqual(abs(counts['operator_A']-counts['operator_B']),1)
        for s in entries:
            self.assertEqual(s['checks']['model_prefilter']['status'],'NOT_RUN')
            self.assertEqual(s['checks']['reachability'],'NOT_PROVEN')
            self.assertGreaterEqual(min(s['checks']['visual_footprint']['pair_gaps_m'].values()),.06)

    def test_all_final_poses_apply_idempotently_and_preserve_bread(self):
        for s in self.manifest['scenarios']:
            cfg=self.fixture();before=copy.deepcopy(cfg)
            m.apply_scenario(cfg,s['id'])
            for name,pose in s['poses'].items():
                self.assertEqual(getattr(cfg.scene,name).init_state.pos,tuple(pose['position_m']))
                self.assertAlmostEqual(pose['position_m'][2],getattr(before.scene,name).init_state.pos[2],places=6)
            applied=copy.deepcopy(cfg)
            m.apply_scenario(cfg,s['id']);self.assertEqual(cfg,applied)
            self.assertEqual(cfg.events,before.events)
            self.assertEqual(cfg.actions,before.actions)
        self.assertEqual(m.get_scenario('baseline')[0]['schema_version'],1)
        self.assertEqual(m.get_scenario('toast_000')[0]['schema_version'],2)

    def test_bad_bread_rejected_before_any_mutation(self):
        broken=copy.deepcopy(self.manifest);broken['scenarios'][0]['poses']['bread_3']['position_m'][0]+=.01
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'bad.json';path.write_text(json.dumps(broken))
            cfg=self.fixture();cfg.scenario_manifest=str(path);before=copy.deepcopy(cfg)
            with self.assertRaisesRegex(ValueError,'relative position'):m.apply_scenario(cfg,'toast_000')
            self.assertEqual(cfg,before)

    def test_polygon_rotation_gap_and_union_hole(self):
        square=hull([(0,0),(1,0),(1,1),(0,1)])
        self.assertEqual(polygon_distance(square,hull([(.5,.5),(2,.5),(2,2),(.5,2)])),0)
        self.assertAlmostEqual(polygon_distance(square,hull([(2,0),(3,0),(3,1),(2,1)])),1)
        triangles=[hull([(0,0),(1,0),(1,1)]),hull([(0,0),(1,1),(0,1)])]
        self.assertTrue(supported(square,triangles))
        self.assertFalse(supported(square,triangles[:1]))
        strips=[hull([(0,0),(.4,0),(.4,1),(0,1)]),hull([(.6,0),(1,0),(1,1),(.6,1)])]
        self.assertFalse(supported(square,strips))
        diamond=hull([(2,.5),(2.5,0),(3,.5),(2.5,1)])
        self.assertAlmostEqual(polygon_distance(square,diamond),1)

    def test_independently_recomputed_footprint_gaps(self):
        import itertools
        vertices={n:json.loads((ROOT/'source/cyclo_lab/data/objects/robodojo_make_toast'/n/'metadata.json').read_text())['geometry']['aligned_bbox']['vertices'] for n in (*OBJECTS,'bread')}
        for scenario in self.manifest['scenarios']:
            polygons={}
            for n in OBJECTS:
                p=scenario['poses'][n]
                polygons[n]=hull([(x+dx,y+dy) for x,y in footprint(vertices[n],p,p['scale']) for dx,dy in itertools.product((-.001,.001),repeat=2)])
            polygons['bread_shelf']=hull(polygons['bread_shelf']+[v for i in range(4) for v in footprint(vertices['bread'],scenario['poses'][f'bread_{i}'])])
            for a,b in itertools.combinations(OBJECTS,2):
                gap=polygon_distance(polygons[a],polygons[b])
                self.assertAlmostEqual(gap,scenario['checks']['visual_footprint']['pair_gaps_m'][a+'/'+b])
                self.assertGreaterEqual(gap,.06)

    def test_missing_evidence_never_qualifies_for_runtime_filter(self):
        import hashlib
        from audit_make_toast_scenarios import audit
        digest=lambda p:hashlib.sha256(p.read_bytes()).hexdigest()
        evidence={'task':self.manifest['task'],
                  'robot_usd_sha256':digest(ROOT/'source/cyclo_lab/data/robots/FFW/FFW_BG2_without_mimic.usd'),
                  'asset_manifest_sha256':digest(CFG.parent.parent/'asset_manifest.json')}
        with tempfile.TemporaryDirectory() as d:
            e=Path(d)/'evidence.json';e.write_text(json.dumps(evidence))
            r=audit(MANIFEST,e,Path(d)/'audit.json')
            self.assertEqual(r['eligible_for_layout_test'],[])
            self.assertTrue(all(row['status']=='EVIDENCE_INCOMPLETE' for row in r['results']))

    def test_model_bound_only_excludes_not_proves(self):
        poses=self.manifest['scenarios'][0]['poses']
        self.assertEqual(model_filter(poses,None)['status'],'NOT_RUN')
        arm={'shoulder_at_lift_zero':[0,0,0],'wrist_radius_upper_m':100,'tcp_radius_upper_m':0}
        model={'fk_validated':True,'lift_axis_world':[0,0,1],'lift_limits':[-.5,0],'arms':{'left':arm,'right':copy.deepcopy(arm)}}
        r=model_filter(poses,{'robot_model':model});self.assertFalse(r['rejected']);self.assertEqual(r['reachability'],'NOT_PROVEN')
        for a in model['arms'].values():a['wrist_radius_upper_m']=.001
        self.assertTrue(model_filter(poses,{'robot_model':model})['rejected'])

    def test_generator_reproducible_and_refuses_overwrite(self):
        with tempfile.TemporaryDirectory() as d:
            path=Path(d)/'delivery';other=generate(path)
            self.assertEqual(json.loads(json.dumps(other['scenarios'])),self.manifest['scenarios'])
            with self.assertRaises(ValueError):generate(path)
            with self.assertRaisesRegex(ValueError,'FK-validated'):generate(Path(d)/'ik',check_ik=True)

    def test_batch_continues_after_launch_failure(self):
        with tempfile.TemporaryDirectory() as d:
            out=Path(d)/'results'
            process=subprocess.run([sys.executable,'-B',str(ROOT/'scripts/tools/run_make_toast_scenario_batch.py'),
                '--output_root',str(out),'--python','/nonexistent/toast-python','--ids','toast_000','toast_001'],capture_output=True,text=True)
            self.assertEqual(process.returncode,1)
            summary=json.loads((out/'summary.json').read_text())
            self.assertTrue(summary['all_requested_ids_finished'])
            self.assertEqual([r['exit_code'] for r in summary['results']],[127,127])
            self.assertEqual(summary['verified_layout_count'],0)

if __name__=='__main__':unittest.main()
