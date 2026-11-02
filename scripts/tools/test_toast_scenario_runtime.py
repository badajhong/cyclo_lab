"""CPU-only contract tests for the shipped loader; no geometry evidence or simulator."""
import copy
import csv
import importlib.util
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from collections import Counter

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / "source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/config/ffw_bg2/scenarios.py"
spec = importlib.util.spec_from_file_location("scenario_loader", PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def fixture(data):
    scene = NS()
    for name, pose in {**data['baseline'], **data['baseline_bread']}.items():
        setattr(scene, name, NS(init_state=NS(pos=tuple(pose['position_m']), rot=tuple(pose['quaternion_wxyz'])),
                               spawn=NS(scale=tuple(data['scales'].get(name, (1, 1, 1))))))
    return NS(scene=scene, serve_to_plate=True)


class RuntimeContract(unittest.TestCase):
    def test_all_ids_apply_and_preserve_relative_bread(self):
        data = m.load_manifest()
        self.assertEqual(len(data['scenarios']), 100)
        for entry in data['scenarios']:
            cfg = fixture(data)
            m.apply_scenario(cfg, entry['id'])
            for name, pose in entry['poses'].items():
                state = getattr(cfg.scene, name).init_state
                self.assertEqual(state.pos, tuple(pose['position_m']))
                self.assertEqual(state.rot, tuple(pose['quaternion_wxyz']))
            # Loader validates all four rigid bread-relative transforms before applying.
            once = copy.deepcopy(cfg)
            m.apply_scenario(cfg, entry['id'])
            self.assertEqual(cfg, once)
            with self.assertRaises(ValueError):
                m.apply_scenario(cfg, data['scenarios'][(data['scenarios'].index(entry)+1)%100]['id'])

    def test_assignments_match_manifest_in_order(self):
        data = m.load_manifest()
        with (ROOT/'scenario_delivery/assignments.csv').open() as f:
            rows = list(csv.DictReader(f))
        self.assertEqual(Counter(r['operator'] for r in rows), {'operator_A': 50, 'operator_B': 50})
        self.assertEqual(len({r['scenario_id'] for r in rows}), 100)
        self.assertEqual([r['scenario_id'] for r in rows], [s['id'] for s in data['scenarios']])
        for row, entry in zip(rows, data['scenarios']):
            self.assertEqual(row['operator'], entry['operator'])
            self.assertEqual(row['order_positive_to_negative_y'], '>'.join(entry['order_positive_to_negative_y']))

    def test_invalid_id_and_baseline_rejected_without_mutation(self):
        cfg = fixture(m.load_manifest())
        before = copy.deepcopy(cfg)
        with self.assertRaises(ValueError):m.apply_scenario(cfg, 'baseline')
        self.assertEqual(cfg, before)
        cfg.scene.plate.init_state.pos = (99, 0, 0)
        before = copy.deepcopy(cfg)
        with self.assertRaises(ValueError):m.apply_scenario(cfg, 'batch02_toast_000')
        self.assertEqual(cfg, before)

    def test_explicit_manifest_and_malformed_data(self):
        original = m.load_manifest()
        mutations = [
            lambda d: d.update(schema_version=1),
            lambda d: d['scenarios'].append(copy.deepcopy(d['scenarios'][0])),
            lambda d: d['scenarios'][0]['poses']['plate']['position_m'].__setitem__(2, 9),
            lambda d: d['scenarios'][0]['poses']['plate']['scale'].__setitem__(0, 9),
            lambda d: d['scenarios'][0]['poses']['bread_0']['position_m'].__setitem__(0, 9),
            lambda d: d['scenarios'][0]['poses']['bread_0'].__setitem__('quaternion_wxyz', [1, 0, 0, 0]),
            lambda d: d['scenarios'][0]['poses']['plate']['position_m'].__setitem__(0, float('nan')),
        ]
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp)/'manifest.json'
            path.write_text(json.dumps(original))
            cfg = fixture(original);cfg.scenario_manifest = str(path)
            m.apply_scenario(cfg, original['scenarios'][0]['id'])
            for mutate in mutations:
                data = copy.deepcopy(original);mutate(data);path.write_text(json.dumps(data))
                cfg = fixture(original);cfg.scenario_manifest = str(path);before = copy.deepcopy(cfg)
                with self.assertRaises(ValueError):m.apply_scenario(cfg, original['scenarios'][0]['id'])
                self.assertEqual(cfg, before)
            path.unlink()
            with self.assertRaises(FileNotFoundError):m.get_scenario('batch02_toast_000',path)


class OperatorSelection(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        path = ROOT/'scripts/tools/view_make_toast_scenario.py'
        spec = importlib.util.spec_from_file_location('viewer', path)
        cls.viewer = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.viewer)

    def args(self, *tokens):
        return self.viewer.build_parser().parse_args(tokens)

    def test_all_operator_entries_in_csv_order(self):
        with (ROOT/'scenario_delivery/assignments.csv').open() as f:rows=list(csv.DictReader(f))
        for op in ('operator_A', 'operator_B'):
            expected=[r['scenario_id'] for r in rows if r['operator']==op]
            self.assertEqual(len(expected),50)
            for i,sid in enumerate(expected,1):
                args=self.args('--operator',op,'--scenario_index',str(i))
                self.assertEqual(self.viewer.resolve_selection(args),sid)
                self.assertEqual(m.get_scenario(sid)[1]['operator'],op)

    def test_invalid_selection_arguments(self):
        import contextlib,io
        invalid=[['--operator','operator_A'],['--scenario_index','1'],
                 ['--operator','operator_A','--scenario_index','1','--scenario_id','batch02_toast_000']]
        invalid += [['--operator','operator_A','--scenario_index',str(i)] for i in (-1,0,51)]
        for tokens in invalid:
            with self.assertRaises(ValueError):self.viewer.resolve_selection(self.args(*tokens))
        for tokens in [['--operator','operator_C','--scenario_index','1'],['--scenario_index','abc']]:
            with contextlib.redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):self.args(*tokens)
        self.assertIsNone(self.viewer.resolve_selection(self.args()))
        self.assertEqual(self.viewer.resolve_selection(self.args('--scenario_id','batch02_toast_099')),'batch02_toast_099')
        with self.assertRaises(ValueError):self.viewer.resolve_selection(self.args('--scenario_id','unknown'))

    def test_assignment_integrity_and_explicit_manifest(self):
        with (ROOT/'scenario_delivery/assignments.csv').open() as f:rows=list(csv.DictReader(f))
        with tempfile.TemporaryDirectory() as tmp:
            manifest=Path(tmp)/'explicit.json';manifest.write_text(json.dumps(m.load_manifest()))
            args=self.args('--operator','operator_B','--scenario_index','50','--scenario_manifest',str(manifest))
            self.assertEqual(self.viewer.resolve_selection(args),[r['scenario_id'] for r in rows if r['operator']=='operator_B'][-1])
            path=Path(tmp)/'assignments.csv'
            for kind in ('duplicate','missing','operator','count'):
                altered=copy.deepcopy(rows)
                if kind=='duplicate':altered[1]['scenario_id']=altered[0]['scenario_id']
                if kind=='missing':altered[1]['scenario_id']='missing'
                if kind=='operator':altered[1]['operator']='invalid'
                if kind=='count':altered.pop()
                with path.open('w',newline='') as f:
                    writer=csv.DictWriter(f,fieldnames=rows[0].keys());writer.writeheader();writer.writerows(altered)
                with self.assertRaises(ValueError):self.viewer.resolve_selection(args,path)
            data=m.load_manifest();data['scenarios'].pop();manifest.write_text(json.dumps(data))
            with self.assertRaises(ValueError):self.viewer.resolve_selection(args)
            data=m.load_manifest();data['scenarios'].append(copy.deepcopy(data['scenarios'][0]));manifest.write_text(json.dumps(data))
            with self.assertRaises(ValueError):self.viewer.resolve_selection(args)

    def test_portable_checkout_and_working_directory(self):
        import os,shutil
        before=Path.cwd()
        with tempfile.TemporaryDirectory() as tmp:
            checkout=Path(tmp)/'renamed checkout';other=Path(tmp)/'unrelated';other.mkdir()
            paths=['scripts/tools/view_make_toast_scenario.py',str(PATH.relative_to(ROOT)),
                   'scenario_delivery/manifest.json','scenario_delivery/assignments.csv']
            for rel in paths:
                target=checkout/rel;target.parent.mkdir(parents=True,exist_ok=True);shutil.copy2(ROOT/rel,target)
            spec=importlib.util.spec_from_file_location('relocated_viewer',checkout/paths[0])
            viewer=importlib.util.module_from_spec(spec);spec.loader.exec_module(viewer)
            try:
                for cwd in (checkout,other):
                    os.chdir(cwd)
                    for op in ('operator_A','operator_B'):
                        for i in ('1','50'):
                            tokens=['--operator',op,'--scenario_index',i]
                            expected=self.viewer.resolve_selection(self.args(*tokens))
                            self.assertEqual(viewer.resolve_selection(viewer.build_parser().parse_args(tokens)),expected)
                            manifest=checkout/'scenario_delivery/manifest.json'
                            for explicit in (str(manifest),os.path.relpath(manifest,cwd)):
                                args=viewer.build_parser().parse_args(tokens+['--scenario_manifest',explicit])
                                self.assertEqual(viewer.resolve_selection(args),expected)
            finally:os.chdir(before)


if __name__ == '__main__':
    unittest.main()
