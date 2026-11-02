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


if __name__ == '__main__':
    unittest.main()
