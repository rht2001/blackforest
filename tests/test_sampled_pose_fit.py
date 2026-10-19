from dataclasses import replace
import unittest
from unittest.mock import patch
from types import SimpleNamespace

import numpy as np
import trimesh

from digital_twin.sensor_identification.classifier import IdentificationConfig
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.matching import fit_model
from digital_twin.sensor_identification.limits import triangle_penetrations


class SampledPoseFitTests(unittest.TestCase):
    def setUp(self):
        self.model = CandidateModel('box', trimesh.creation.box(extents=[12, 20, 60]), sample_count=2000)
        self.config = replace(IdentificationConfig(), random_starts=1, refine_starts=2,
                              icp_iterations=8, max_nfev=30, tolerance_mm=.8)

    def test_local_projection_matches_box_surface_without_sample_gaps(self):
        points = np.random.default_rng(42).uniform([-20, -30, -40], [20, 30, 40], (530, 3))
        closest, distances = self.model.nearest_sampled(points)
        exact, expected = self.model.nearest(points)
        np.testing.assert_allclose(distances, expected, atol=1e-8)
        np.testing.assert_allclose(closest, exact, atol=1e-8)
        empty, d = self.model.nearest_sampled(np.empty((0, 3)))
        self.assertEqual(empty.shape, (0, 3))
        self.assertEqual(d.shape, (0,))

    def test_full_mesh_queries_are_only_final_contact_and_clearance_checks(self):
        palm = np.array([[x, 0., z] for x in np.linspace(-20, 20, 5)
                         for z in np.linspace(75, 155, 9)])
        normals = np.tile([0., 1., 0.], (len(palm), 1))
        contacts = self.model.mesh.vertices+[0., 11., 115.]
        exclusions = np.array([[0., 30., 115.]])
        with patch.object(self.model, 'nearest', wraps=self.model.nearest) as exact:
            fit = fit_model(self.model, contacts, exclusions, np.array([2.]), self.config,
                            boundary_xyz=palm, boundary_normals=normals,
                            palm_xyz=palm, palm_normals=normals)
        self.assertTrue(fit['compatible'], fit)
        self.assertEqual(exact.call_count, 2*fit['refinements'])
        self.assertEqual([len(call.args[0]) for call in exact.call_args_list], [8, 1]*fit['refinements'])
        self.assertEqual(fit['validation_method'], 'exact_full_mesh')

    def test_quick_preview_samples_contacts_but_audits_all_hand_patches(self):
        contacts = np.tile(self.model.mesh.vertices, (40, 1))
        contacts[-1] = [100., 100., 100.]
        boundary = np.tile([[0., -11., 0.]], (300, 1))
        normals = np.tile([[0., -1., 0.]], (300, 1))
        config = replace(self.config, quick_preview=True, refine_starts=1, icp_iterations=2, max_nfev=3)
        with patch.object(self.model, 'nearest', wraps=self.model.nearest) as exact, \
                patch('digital_twin.sensor_identification.matching.triangle_penetrations',
                      wraps=triangle_penetrations) as audit:
            fit = fit_model(self.model, contacts, np.empty((0, 3)), np.array([]), config,
                            boundary_xyz=boundary, boundary_normals=normals)
        self.assertEqual(fit['validation_method'], 'exact_full_mesh')
        self.assertEqual(audit.call_count, fit['refinements'])
        self.assertEqual(len(audit.call_args.args[3]), len(boundary))
        self.assertEqual(exact.call_count, fit['refinements'])
        self.assertEqual(fit['search_contact_points'], 9)
        self.assertEqual(fit['search_hand_points'], 1)
        self.assertEqual(fit['scored_contact_points'], len(contacts))
        self.assertEqual(fit['input_contact_points'], len(contacts))
        self.assertEqual(fit['input_hand_points'], len(boundary))
        self.assertGreater(fit['max_error_mm'], config.tolerance_mm)

    def test_preview_never_promotes_a_calibrated_pose_to_confirmed_identity(self):
        from digital_twin.sensor_identification.classifier import identify_sensor
        points = [dict(id=i, x=float(x), y=float(y), z=float(z), pressure=900.)
                  for i, (x, y, z) in enumerate(self.model.mesh.vertices)]
        fit = dict(compatible=True, score=.1, optimizer_success=True, consistent_refinements=2)
        with patch('digital_twin.sensor_identification.classifier.fit_model', return_value=fit):
            result = identify_sensor(points, dict(SENSOR_A=self.model, SENSOR_B=self.model),
                                     replace(self.config, quick_preview=True, calibrated=True))
        self.assertEqual(result['classification'], 'AMBIGUOUS')
        self.assertEqual(result['reason'], 'quick_placement')
        self.assertIn(result['best_candidate'], ('SENSOR_A', 'SENSOR_B'))

    def test_exact_audit_rejects_crossings_hidden_by_sampled_nearest_surface(self):
        contacts = self.model.mesh.vertices.copy()
        # Freeze a known crossing pose to isolate the exact audit from the
        # optimizer's ability to escape a finite hand patch.
        def freeze(fun, x0, **kwargs):
            return SimpleNamespace(x=x0, success=True, nfev=1)
        with patch('digital_twin.sensor_identification.matching.least_squares', side_effect=freeze), \
             patch('digital_twin.sensor_identification.matching._rigid', return_value=(np.eye(3),np.zeros(3))):
            fit = fit_model(self.model, contacts, np.empty((0, 3)), np.array([]),
                            replace(self.config, tolerance_mm=10.),
                            boundary_xyz=np.array([[0., 0., 29.]]),
                            boundary_normals=np.array([[0., 0., 1.]]))
        self.assertFalse(fit['compatible'])
        self.assertIn('hand_penetration', fit['rejection_reasons'])


if __name__ == '__main__':
    unittest.main()
