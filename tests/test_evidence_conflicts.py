from dataclasses import replace
from pathlib import Path
import re
import unittest
from unittest.mock import patch
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from digital_twin.sensor_identification.evidence import contact_hand_conflicts
from digital_twin.sensor_identification.classifier import IdentificationConfig, identify_sensor
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.matching import fit_model
from digital_twin.sensor_identification import web_service


class EvidenceConflictTests(unittest.TestCase):
    def test_full_contact_ball_inside_patch_certifies_incompatibility(self):
        result = contact_hand_conflicts(np.array([[0., 0., -12.], [10., 0., -12.]]),
                                       np.zeros((1, 3)), np.array([[0., 0., 1.]]), 3., 3.)
        self.assertEqual(result['contact_indices'], [0])
        self.assertEqual(result['boundary_indices'], [0])
        self.assertEqual(result['witnesses'][0]['minimum_required_penetration_mm'], 9.)

    def test_ball_that_can_escape_tangent_patch_has_no_certificate(self):
        contacts = np.array([[1.1, 0., -12.], [0., 1.1, -12.], [0., 0., -6.]])
        result = contact_hand_conflicts(contacts, np.zeros((1, 3)), np.array([[0., 0., 1.]]), 3., 3.)
        self.assertEqual(result['contact_count'], 0)

    def test_large_tolerance_and_empty_evidence_do_not_assert_conflict(self):
        for contacts, points, normals, tolerance in [
                (np.array([[0., 0., -12.]]), np.zeros((1, 3)), np.array([[0., 0., 1.]]), 4.),
                (np.empty((0, 3)), np.zeros((1, 3)), np.array([[0., 0., 1.]]), 3.),
                (np.array([[0., 0., -12.]]), None, None, 3.)]:
            self.assertEqual(contact_hand_conflicts(contacts, points, normals, tolerance, 3.)['contact_count'], 0)

    def test_certificate_survives_rigid_frame_change(self):
        r = Rotation.from_euler('xyz', [23., -31., 48.], degrees=True).as_matrix()
        t = np.array([57., -89., 21.])
        result = contact_hand_conflicts(np.array([[0., 0., -12.]])@r.T+t,
                                       np.zeros((1, 3))@r.T+t, np.array([[0., 0., 1.]])@r.T, 3., 3.)
        self.assertEqual(result['contact_count'], 1)
        self.assertAlmostEqual(result['witnesses'][0]['depth_mm'], 12.)

    def test_every_point_of_certified_ball_violates_patch(self):
        rng = np.random.default_rng(88)
        ball = rng.normal(size=(2000, 3));ball *= 3./np.linalg.norm(ball, axis=1)[:, None]
        ball += [0., 0., -12.]
        self.assertTrue(np.all(np.abs(ball[:, :2]) <= 4.))
        self.assertTrue(np.all(-ball[:, 2] > 3.))

    def test_contradictory_geometry_returns_rejected_preview_in_both_modes(self):
        model = CandidateModel('box', trimesh.creation.box(extents=[2., 2., 2.]), sample_count=500)
        points = [dict(id=str(i), x=float(x), y=float(y), z=float(z-12.), pressure=700., source_index='42')
                  for i, (x, y, z) in enumerate(model.mesh.vertices)]
        boundary = [dict(id='bad-limit', x=0., y=0., z=0., normal=[0., 0., 1.], source_index='43')]
        for quick in (False, True):
            with self.subTest(quick=quick):
                config = replace(IdentificationConfig(), calibrated=True, quick_preview=quick,
                                 min_spread_mm=.1, tolerance_mm=.5, point_spacing_mm=0.,
                                 random_starts=1, refine_starts=2, icp_iterations=3, max_nfev=8)
                result = identify_sensor(points, dict(SENSOR_A=model, SENSOR_B=model), config, boundary_points=boundary)
                self.assertEqual(result['classification'], 'AMBIGUOUS')
                self.assertEqual(result['reason'], 'contact_hand_geometry_conflict')
                self.assertEqual(len(result['fits']), 2)
                self.assertEqual(result['geometry_conflict']['witnesses'][0]['contact_source_index'], '42')
                for fit in result['fits'].values():
                    self.assertFalse(fit['compatible'])
                    self.assertTrue(fit['hand_limit_max_is_lower_bound'])
                    self.assertFalse(fit['hand_validation_complete'])
                    self.assertEqual(fit['hand_constraints_checked'], 1)
                    self.assertIn('input_geometry_conflict', fit['rejection_reasons'])
                    self.assertTrue(fit['contact_fit_compatible'])

    def test_frontend_and_backend_require_the_same_revision(self):
        source = (Path(web_service.__file__).parent/'simulation.js').read_text(encoding='utf-8')
        revisions = re.findall(r"estimator_revision!==\s*'([^']+)'", source)
        self.assertEqual(len(revisions), 2)
        self.assertEqual(set(revisions), {web_service.ESTIMATOR_REVISION})

    def test_soft_preview_axis_hint_cannot_replace_a_passing_contact_fit_with_a_failure(self):
        model = CandidateModel('long', trimesh.creation.box(extents=[2., 20., 2.]), sample_count=500)
        contacts = np.array([[x, y, -12.] for x in np.linspace(-20., 20., 21) for y in [-.2, .2]])
        original = dict(compatible=True, transform=np.eye(4).tolist(), max_error_mm=.2,
                        rmse_mm=.1, rejection_reasons=[])
        worse = {**original, 'compatible': False, 'max_error_mm': 4., 'rmse_mm': 2.}
        with patch('digital_twin.sensor_identification.matching._fit_model', side_effect=[original, worse]) as fit, \
             patch('digital_twin.sensor_identification.matching.triangle_penetrations', return_value=np.array([8.])):
            result = fit_model(model, contacts, np.empty((0, 3)), np.empty(0), IdentificationConfig(),
                               boundary_xyz=np.zeros((1, 3)), boundary_normals=np.array([[0., 0., 1.]]))
        self.assertEqual(fit.call_count, 2)
        self.assertTrue(result['contact_fit_compatible'])
        self.assertEqual(result['max_error_mm'], .2)
        self.assertNotIn('preview_axis_prior', result)
        self.assertFalse(result['compatible'])

    def test_already_aligned_preview_does_not_repeat_the_search(self):
        model = CandidateModel('long', trimesh.creation.box(extents=[2., 20., 2.]), sample_count=500)
        contacts = np.array([[x, y, -12.] for x in np.linspace(-20., 20., 21) for y in [-.2, .2]])
        transform = np.eye(4)
        transform[:3, :3] = Rotation.from_euler('z', -90., degrees=True).as_matrix()
        original = dict(compatible=True, transform=transform.tolist(), max_error_mm=.2,
                        rmse_mm=.1, rejection_reasons=[])
        with patch('digital_twin.sensor_identification.matching._fit_model', return_value=original) as fit, \
             patch('digital_twin.sensor_identification.matching.triangle_penetrations', return_value=np.array([8.])):
            result = fit_model(model, contacts, np.empty((0, 3)), np.empty(0), IdentificationConfig(),
                               boundary_xyz=np.zeros((1, 3)), boundary_normals=np.array([[0., 0., 1.]]))
        self.assertEqual(fit.call_count, 1)
        self.assertTrue(result['contact_fit_compatible'])
        self.assertFalse(result['compatible'])


if __name__ == '__main__':
    unittest.main()
