"""CPU/stdlib tests of config transformation; these do not validate physics."""

import copy
import importlib.util
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest

ROOT = Path(__file__).resolve().parents[2]
PATH = ROOT / "source/cyclo_lab/cyclo_lab/manager_based/manipulation/make_toast/config/ffw_bg2/scenarios.py"
spec = importlib.util.spec_from_file_location("toast_scenarios_test", PATH)
m = importlib.util.module_from_spec(spec)
spec.loader.exec_module(m)


def config_fixture():
    data = m.load_manifest()
    scene = NS()
    for name, pose in data["baseline"].items():
        setattr(scene, name, NS(init_state=NS(pos=tuple(pose["position_m"]),
                                             rot=tuple(pose["quaternion_wxyz"])),
                                spawn=NS(scale=tuple(data["scales"].get(name, (1, 1, 1))))))
    # Match the original v2 rack grooves. Use independent slice poses as inputs.
    for i, x in enumerate((.035, .010, -.015, -.039)):
        setattr(scene, f"bread_{i}", NS(init_state=NS(pos=(.443, -.25 - 2*x, 1.0932),
                                                       rot=(.5, .5, .5, -.5))))
    return NS(scene=scene, serve_to_plate=True, actions="unchanged", events="unchanged")


class ScenarioTests(unittest.TestCase):
    def test_known_world_yaw_and_pivot(self):
        pos, quat = m.transform_pose((2, 2, 3), (1, 0, 0, 0), (1, 2, 3),
                                     (.1, .2, 0), math.pi / 2)
        for actual, expected in zip(pos, (1.1, 3.2, 3)):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(quat, (math.sqrt(.5), 0, 0, math.sqrt(.5))):
            self.assertAlmostEqual(actual, expected)

    def test_baseline_exact_noop(self):
        cfg = config_fixture()
        before = copy.deepcopy(cfg.scene)
        m.apply_scenario(cfg, "baseline")
        self.assertEqual(cfg.scene, before)

    def test_all_four_breads_follow_rigid_shelf_motion(self):
        for scenario_id in ("small_offset", "swapped_candidate"):
            cfg = config_fixture()
            before = copy.deepcopy(cfg)
            _, entry = m.get_scenario(scenario_id)
            yaw = entry["changes"]["bread_shelf"]["yaw"]
            m.apply_scenario(cfg, scenario_id)
            for i in range(4):
                old = getattr(before.scene, f"bread_{i}").init_state
                new = getattr(cfg.scene, f"bread_{i}").init_state
                offset = [a-b for a,b in zip(old.pos, before.scene.bread_shelf.init_state.pos)]
                expected = m.rotate_z(offset, yaw)
                actual = [a-b for a,b in zip(new.pos, cfg.scene.bread_shelf.init_state.pos)]
                for a,b in zip(actual, expected):
                    self.assertAlmostEqual(a,b,places=12)
                self.assertEqual(new.rot, m.yaw_quaternion(old.rot,yaw))
            self.assertEqual(cfg.actions, before.actions)
            self.assertEqual(cfg.events, before.events)
            once = copy.deepcopy(cfg.scene)
            m.apply_scenario(cfg, scenario_id)
            self.assertEqual(cfg.scene, once)
            with self.assertRaises(ValueError):
                m.apply_scenario(cfg, "baseline")

    def test_unknown_or_changed_baseline_rejected_before_mutation(self):
        cfg = config_fixture()
        before = copy.deepcopy(cfg)
        with self.assertRaises(ValueError):
            m.apply_scenario(cfg, "missing")
        self.assertEqual(cfg,before)
        cfg.scene.plate.init_state.pos = (1,2,3)
        before = copy.deepcopy(cfg)
        with self.assertRaises(ValueError):
            m.apply_scenario(cfg, "small_offset")
        self.assertEqual(cfg,before)

    def test_candidates_fit_measured_envelope_and_remain_unverified(self):
        for name in ("small_offset", "swapped_candidate"):
            report = m.static_layout_report(name)
            self.assertGreater(min(report["table_envelope_margin_m"].values()),0)
            self.assertTrue(all(not pair["aabb_xy_overlap"] for pair in report["pairs"].values()))
            self.assertEqual(report["reachability"],"NOT_TESTED")
            self.assertEqual(report["tabletop_containment"],"NOT_VERIFIED")


if __name__ == "__main__":
    unittest.main()
