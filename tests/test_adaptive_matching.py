"""Regression tests for broad screening and complete final evidence checks."""
from dataclasses import replace
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from scipy.optimize import least_squares as solve

from digital_twin.sensor_identification.classifier import IdentificationConfig
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.matching import _distinct_starts, fit_model


class AdaptiveMatchingTests(unittest.TestCase):
    def setUp(self):
        self.model = CandidateModel('box', trimesh.creation.box(extents=[10., 14., 24.]), sample_count=1000)

    def test_screen_keeps_distinct_modes_in_cost_order(self):
        identity = np.eye(3)
        tilted = Rotation.from_euler('x', 40., degrees=True).as_matrix()
        ranked = [(0., identity, np.zeros(3), 'best'),
                  (.1, identity, np.ones(3), 'duplicate'),
                  (.2, tilted, np.zeros(3), 'rotation'),
                  (.3, identity, np.array([8., 0., 0.]), 'translation')]
        self.assertEqual([s[3] for s in _distinct_starts(ranked, 3)], ['best', 'rotation', 'translation'])
        self.assertEqual([s[3] for s in _distinct_starts(ranked, 1)], ['best'])

    def test_hidden_bad_contact_is_checked_even_when_spacing_removes_it(self):
        # The first representative is on the top face. Its nearby bad neighbor
        # is suppressed by search spacing but must still fail the final audit.
        contacts = np.vstack(([[0., 0., 12.], [0., 0., 13.8]], self.model.mesh.vertices))
        config = replace(IdentificationConfig(), quick_preview=True, tolerance_mm=.5,
                         point_spacing_mm=2., random_starts=1, refine_starts=1,
                         icp_iterations=1, max_nfev=1)
        def frozen(fun, x0, **kwargs):
            return SimpleNamespace(x=x0, success=True, nfev=1)
        with patch('digital_twin.sensor_identification.matching._rigid', return_value=(np.eye(3), np.zeros(3))), \
             patch('digital_twin.sensor_identification.matching.least_squares', side_effect=frozen):
            fit = fit_model(self.model, contacts, np.empty((0, 3)), np.empty(0), config)
        self.assertFalse(fit['compatible'])
        self.assertEqual(fit['scored_contact_points'], len(contacts))
        self.assertGreater(fit['max_error_mm'], config.tolerance_mm)
        self.assertEqual(fit['validation_method'], 'exact_full_mesh')

    def test_detailed_search_promotes_a_bounded_subset_and_recovers_pose(self):
        r = Rotation.from_euler('xyz', [17., -26., 38.], degrees=True).as_matrix()
        contacts = self.model.mesh.vertices@r.T+[7., -4., 19.]
        config = replace(IdentificationConfig(), tolerance_mm=.3, random_starts=2,
                         refine_starts=2, icp_iterations=12, max_nfev=40, point_spacing_mm=0.)
        fit = fit_model(self.model, contacts, np.empty((0, 3)), np.empty(0), config,
                        boundary_xyz=np.array([[0., 0., -100.]]),
                        boundary_normals=np.array([[0., 0., 1.]]))
        self.assertTrue(fit['compatible'], fit)
        self.assertLess(fit['rmse_mm'], .1)
        self.assertLessEqual(fit['promoted_starts'], 12)
        self.assertLess(fit['promoted_starts'], fit['starts_evaluated'])
        self.assertEqual(fit['search_method'], 'coarse_to_fine_multistart')

    def test_close_hand_patch_cannot_be_removed_from_final_audit(self):
        hand = np.array([[0., 0., -13.], [0., 0., -11.2]])
        normals = np.tile([0., 0., 1.], (2, 1))
        config = replace(IdentificationConfig(), tolerance_mm=1., hand_tolerance_mm=.5,
                         point_spacing_mm=2., quick_preview=True, random_starts=1,
                         refine_starts=1, icp_iterations=1, max_nfev=1)
        def frozen(fun, x0, **kwargs):
            return SimpleNamespace(x=x0, success=True, nfev=1)
        with patch('digital_twin.sensor_identification.matching._rigid', return_value=(np.eye(3), np.zeros(3))), \
             patch('digital_twin.sensor_identification.matching.least_squares', side_effect=frozen):
            fit = fit_model(self.model, self.model.mesh.vertices, np.empty((0, 3)), np.empty(0),
                            config, boundary_xyz=hand, boundary_normals=normals)
        self.assertFalse(fit['compatible'])
        self.assertEqual(fit['hand_constraints_checked'], 2)
        self.assertAlmostEqual(fit['hand_limit_max_mm'], .8)

    def test_quick_acceptance_uses_exact_triangles_without_sample_gap(self):
        contacts = self.model.mesh.vertices
        config = replace(IdentificationConfig(), quick_preview=True, tolerance_mm=.01,
                         random_starts=1, refine_starts=1, icp_iterations=1, max_nfev=1)
        def frozen(fun, x0, **kwargs):
            return SimpleNamespace(x=x0, success=True, nfev=1)
        with patch('digital_twin.sensor_identification.matching._rigid', return_value=(np.eye(3), np.zeros(3))), \
             patch('digital_twin.sensor_identification.matching.least_squares', side_effect=frozen), \
             patch.object(self.model, 'nearest_sampled', side_effect=AssertionError('approximate final query')):
            fit = fit_model(self.model, contacts, np.empty((0, 3)), np.empty(0), config)
        self.assertTrue(fit['compatible'], fit)
        self.assertLess(fit['max_error_mm'], 1e-9)

    def test_constraint_exchange_recovers_contact_removed_by_spacing(self):
        contacts = np.vstack(([[0., 0., 12.], [0., 0., 13.8]], self.model.mesh.vertices))
        config = replace(IdentificationConfig(), tolerance_mm=1., point_spacing_mm=2.,
                         random_starts=1, refine_starts=1, icp_iterations=1, max_nfev=40)
        calls = 0
        def initial_pose_then_solve(fun, x0, **kwargs):
            nonlocal calls
            calls += 1
            if calls == 1:
                return SimpleNamespace(x=x0, success=True, nfev=1)
            return solve(fun, x0, **kwargs)
        with patch('digital_twin.sensor_identification.matching._rigid', return_value=(np.eye(3), np.zeros(3))), \
             patch('digital_twin.sensor_identification.matching.least_squares', side_effect=initial_pose_then_solve):
            fit = fit_model(self.model, contacts, np.empty((0, 3)), np.empty(0), config)
        self.assertTrue(fit['compatible'], fit)
        self.assertLessEqual(fit['max_error_mm'], config.tolerance_mm)
        self.assertGreater(fit['contact_repair_evaluations'], 0)


if __name__ == '__main__':
    unittest.main()
