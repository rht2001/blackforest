from dataclasses import replace
import unittest
from unittest.mock import patch

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from digital_twin.sensor_identification.contact_directions import pressure_directions, direction_residual
from digital_twin.sensor_identification.classifier import IdentificationConfig, identify_sensor
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.matching import fit_model


class PressureDirectionTests(unittest.TestCase):
    def setUp(self):
        x, y = np.meshgrid(np.arange(-8., 9., 2.), np.arange(-4., 5., 2.))
        self.points = np.column_stack((x.ravel(), y.ravel(), np.full(x.size, 5.)))
        self.pressures = 250.+800.*np.exp(-np.sum(self.points[:, :2]**2, axis=1)/30.)
        self.model = CandidateModel('box', trimesh.creation.box(extents=[24., 18., 10.]), sample_count=1500)

    def test_weighted_patch_plane_rotates_with_observations(self):
        rotation = Rotation.from_euler('xyz', [28., -33., 47.], degrees=True).as_matrix()
        centers, normals, strength = pressure_directions(self.points@rotation.T+[15., -9., 20.], self.pressures, 200.)
        self.assertGreater(len(centers), 10)
        np.testing.assert_allclose(np.abs(normals@rotation[:, 2]), 1., atol=1e-8)
        self.assertTrue(np.all((strength > 0.) & (strength <= 3.)))

    def test_no_direction_from_lines_duplicates_or_volumetric_neighborhood(self):
        line = np.column_stack((np.arange(20.), np.zeros((20, 2))))
        for points in (line, np.repeat(self.points[:3], 12, axis=0),
                       np.array(list(np.ndindex(3, 3, 3)), dtype=float)*2.):
            self.assertEqual(len(pressure_directions(points, np.full(len(points), 700.), 200.)[0]), 0)
        self.assertEqual(len(pressure_directions(self.points, None, 200.)[0]), 0)

    def test_surface_normal_penalty_detects_tilt_and_ignores_sign(self):
        directions = pressure_directions(self.points, self.pressures, 200.)
        aligned = direction_residual(self.model, np.eye(3), np.zeros(3), directions)
        tilted = direction_residual(self.model, Rotation.from_euler('x', 25., degrees=True).as_matrix(), np.zeros(3), directions)
        self.assertLess(np.linalg.norm(aligned), 1e-8)
        self.assertGreater(np.linalg.norm(tilted), 1.)
        flipped = (directions[0], -directions[1], directions[2])
        self.assertAlmostEqual(np.linalg.norm(tilted), np.linalg.norm(direction_residual(
            self.model, Rotation.from_euler('x', 25., degrees=True).as_matrix(), np.zeros(3), flipped)))

    def test_pressure_fit_uses_patch_directions_in_rotated_pose(self):
        rotation = Rotation.from_euler('xyz', [25., -32., 39.], degrees=True).as_matrix()
        points = self.points@rotation.T+[20., -10., 14.]
        config = replace(IdentificationConfig(), random_starts=1, refine_starts=2,
                         icp_iterations=8, max_nfev=30, point_spacing_mm=0.)
        fit = fit_model(self.model, points, np.empty((0, 3)), np.empty(0), config,
                        contact_pressures=self.pressures)
        self.assertGreater(fit['pressure_direction_patches'], 10)
        self.assertLess(fit['pressure_direction_rmse_mm'], .1)
        self.assertLess(fit['rmse_mm'], .1)
        self.assertTrue(fit['compatible'])

    def test_classifier_forwards_pressure_and_converts_metres(self):
        points = [dict(id=i, x=p[0]/1000, y=p[1]/1000, z=p[2]/1000, pressure=float(w))
                  for i, (p, w) in enumerate(zip(self.points, self.pressures))]
        fit = dict(compatible=True, score=0., optimizer_success=True, consistent_refinements=2)
        config = replace(IdentificationConfig(), coordinate_units='m', min_spread_mm=1.)
        with patch('digital_twin.sensor_identification.classifier.fit_model', return_value=fit) as mocked:
            identify_sensor(points, {'SENSOR_A': self.model, 'SENSOR_B': self.model}, config)
        for call in mocked.call_args_list:
            np.testing.assert_allclose(call.args[1], self.points)
            np.testing.assert_allclose(call.kwargs['contact_pressures'], self.pressures)


if __name__ == '__main__':
    unittest.main()
