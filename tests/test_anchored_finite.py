import sys, unittest
from pathlib import Path
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.finite_hand import FiniteHand
from digital_twin.sensor_identification.ray_bvh import TriangleBVH
from digital_twin.sensor_identification.anchored_fit import fit_anchored, _contact_surface
from digital_twin.sensor_identification.visible_surface import first_surface
from digital_twin.sensor_identification.classifier import IdentificationConfig
from digital_twin.sensor_identification.adapter import normalize_frame


class FiniteHandTests(unittest.TestCase):
    def test_saved_snapshot_retains_contact_region_and_direction(self):
        frame = dict(frame='left_base_link', units='metres', points=[dict(id='thumb:1',
            position_m=[.01, .02, .03], value=800, normal=[0, 1, 0], region='thumb_tip',
            link='thumb_sensor', source_index='42')])
        point = normalize_frame(frame)['points'][0]
        self.assertEqual(point['region'], 'thumb_tip')
        self.assertEqual(point['link'], 'thumb_sensor')
        self.assertEqual(point['source_index'], '42')
        self.assertEqual(point['normal'], [0, 1, 0])
        self.assertEqual(point['x'], .01)

    def setUp(self):
        self.box = CandidateModel('hand', trimesh.creation.box(extents=[10, 20, 30]), sample_count=1000)

    def test_forbidden_region_stops_at_back_of_actual_hand(self):
        hand = FiniteHand([('hand', self.box, np.eye(4))])
        np.testing.assert_allclose(hand.depths([[0, 0, 0], [0, 0, -40], [0, 0, 40]]), [5, 0, 0])

    def test_anchored_correspondence_accepts_visible_near_edge_but_not_far_wall(self):
        points = np.array([[-5., 10.7, 0], [5., 0, 0]])
        normals = np.tile([1., 0, 0], (2, 1))
        self.assertTrue(np.isinf(first_surface(self.box, points, normals, np.eye(3), np.zeros(3))[0]))
        error = _contact_surface(self.box, points, normals, np.eye(3), np.zeros(3), exact=True)
        np.testing.assert_allclose(error, [.7, 10.], atol=1e-7)

    def test_queries_follow_recorded_rigid_transform(self):
        tr = np.eye(4); tr[:3, :3] = Rotation.from_euler('xyz', [.4, -.2, .7]).as_matrix(); tr[:3, 3] = [21, -7, 130]
        p = np.array([[0, 0, 0], [0, 0, -40]])@tr[:3, :3].T+tr[:3, 3]
        np.testing.assert_allclose(FiniteHand([('hand', self.box, tr)]).depths(p), [5, 0], atol=1e-8)

    def test_escape_targets_point_out_of_finite_body(self):
        hand = FiniteHand([('hand', self.box, np.eye(4))])
        ids, target, n = hand.correction_targets([[2, 0, 0], [100, 0, 0]], 1.)
        np.testing.assert_array_equal(ids, [0]); np.testing.assert_allclose(target, [[5, 0, 0]])
        np.testing.assert_allclose(n, [[1, 0, 0]])

    def test_missing_pose_is_never_certified(self):
        hand = FiniteHand.from_snapshot(None)
        audit = hand.audit(self.box, np.eye(3), np.zeros(3), 3.)
        self.assertFalse(audit['finite_hand_available']); self.assertFalse(audit['hand_validation_complete'])

    def test_bvh_matches_triangle_reference_with_seams(self):
        mesh = trimesh.creation.icosphere(subdivisions=2, radius=10)
        rng = np.random.default_rng(31); origins = rng.normal(0, 25, (200, 3)); directions = rng.normal(size=(200, 3))
        locations, ids, _ = mesh.ray.intersects_location(origins, directions, multiple_hits=True)
        reference = np.full(200, np.inf)
        parameter = np.sum((locations-origins[ids])*directions[ids], axis=1)/np.sum(directions[ids]**2, axis=1)
        np.minimum.at(reference, ids, parameter)
        actual, _ = TriangleBVH(mesh).first(origins, directions)
        np.testing.assert_array_equal(np.isfinite(actual), np.isfinite(reference))
        np.testing.assert_allclose(actual[np.isfinite(actual)], reference[np.isfinite(reference)], atol=1e-8)

    def test_anchored_registration_preserves_better_start_and_identity_gate(self):
        points = np.array([[-5, y, z] for y in [-6, 0, 6] for z in [-10, 0, 10]], dtype=float)
        normals = np.tile([1., 0, 0], (len(points), 1))
        far = np.eye(4); far[0, 3] = 100.
        prior = dict(center_mm=[0, 0, 0], rotation_xyz_deg=[0, 0, 0], position_window_mm=[20, 12, 15])
        fit = fit_anchored(self.box, points, normals, ['contact']*len(points), np.empty((0, 3)),
                           np.empty(0), IdentificationConfig(quick_preview=True, tolerance_mm=3.), prior,
                           hand=FiniteHand([('far_hand', self.box, far)], skipped=['open_skin']))
        self.assertAlmostEqual(fit['visible_surface_inlier_fraction'], 1.)
        self.assertLess(fit['prior_rotation_delta_deg'], 1e-7)
        self.assertEqual(fit['scored_contact_points'], len(points))
        self.assertTrue(fit['placement_usable']); self.assertFalse(fit['compatible'])
        self.assertFalse(fit['hand_validation_complete']); self.assertIn('open_skin', fit['finite_hand_open_links'])


if __name__ == '__main__':
    unittest.main()
