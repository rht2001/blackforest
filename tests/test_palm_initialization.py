from dataclasses import replace
import unittest

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from digital_twin.sensor_identification.classifier import IdentificationConfig, identify_sensor
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.matching import _palm_starts, fit_model


class PalmInitializationTests(unittest.TestCase):
    def setUp(self):
        self.model = CandidateModel('box', trimesh.creation.box(extents=[12, 20, 60]), sample_count=2000)
        self.palm = np.array([[x, 0., z] for x in np.linspace(-20, 20, 5)
                              for z in np.linspace(75, 155, 9)])
        self.normals = np.tile([0., 1., 0.], (len(self.palm), 1))
        self.contacts = self.model.mesh.vertices+[0., 11., 115.]
        self.config = replace(IdentificationConfig(), random_starts=1, refine_starts=2,
                              icp_iterations=12, max_nfev=50, tolerance_mm=.8)

    def test_batched_nearest_preserves_exact_distance(self):
        points = np.random.default_rng(42).uniform(-100, 100, (97, 3))
        expected, distance, _ = trimesh.proximity.closest_point(self.model.mesh, points)
        actual, actual_distance = self.model.nearest(points)
        np.testing.assert_allclose(actual, expected)
        np.testing.assert_allclose(actual_distance, distance)

    def test_starts_are_rigid_outside_palm_and_equivariant(self):
        starts, normal = _palm_starts(self.model, self.contacts, self.palm, self.normals)
        self.assertEqual(len(starts), 48)
        for r, t in starts:
            np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-10)
            self.assertGreaterEqual(((self.model.mesh.vertices@r.T+t)@normal).min(), -1e-8)
        rotation = Rotation.from_euler('xyz', [23, -11, 40], degrees=True).as_matrix()
        translation = np.array([40., -30., 12.])
        moved, n = _palm_starts(self.model, self.contacts@rotation.T+translation,
                                self.palm@rotation.T+translation, self.normals@rotation.T)
        np.testing.assert_allclose(n, normal@rotation.T, atol=1e-10)
        # Symmetric start set permits a change of longitudinal axis sign.
        for r, t in starts:
            self.assertTrue(any(np.allclose(mr, rotation@r, atol=1e-7)
                                and np.allclose(mt, rotation@t+translation, atol=1e-7)
                                for mr, mt in moved))

    def test_fits_contacts_without_crossing_rigid_palm(self):
        events = []
        fit = fit_model(self.model, self.contacts, np.empty((0, 3)), np.array([]), self.config,
                        boundary_xyz=self.palm, boundary_normals=self.normals,
                        palm_xyz=self.palm, palm_normals=self.normals, progress=events.append)
        self.assertTrue(fit['compatible'], fit)
        self.assertLessEqual(fit['hand_limit_max_mm'], self.config.hand_tolerance_mm)
        self.assertEqual(fit['palm_starts_evaluated'], 48)
        self.assertEqual(fit['palm_initial_dof'], 4)
        self.assertEqual(fit['refinement_dof'], 6)
        self.assertEqual(events[0]['stage'], 'palm_initialization')

    def test_finger_only_limits_do_not_invent_palm(self):
        from unittest.mock import patch
        points = [dict(id=i, x=float(x), y=float(y), z=float(z), pressure=900.)
                  for i, (x, y, z) in enumerate(self.contacts)]
        boundaries = [dict(id=f'b{i}', x=float(x), y=float(y), z=float(z),
                           normal=n.tolist(), link='left_thumb_link')
                      for i, ((x, y, z), n) in enumerate(zip(self.palm, self.normals))]
        fake = dict(compatible=False, score=10.)
        with patch('digital_twin.sensor_identification.classifier.fit_model', return_value=fake) as fitter:
            identify_sensor(points, dict(SENSOR_A=self.model, SENSOR_B=self.model),
                            self.config, boundary_points=boundaries)
            self.assertNotIn('palm_xyz', fitter.call_args.kwargs)
            for p in boundaries:
                p['link'] = 'left_base_link'
            identify_sensor(points, dict(SENSOR_A=self.model, SENSOR_B=self.model),
                            replace(self.config, coordinate_units='m'), boundary_points=boundaries)
            np.testing.assert_allclose(fitter.call_args.kwargs['palm_xyz'], self.palm*1000)


if __name__ == '__main__':
    unittest.main()
