"""Surface benchmarks use the supplied CAD, without constructing a controller."""
from dataclasses import replace
import json
from pathlib import Path
import tempfile
import unittest

import numpy as np
from scipy.spatial.transform import Rotation

from digital_twin.sensor_identification import IdentificationConfig, identify_sensor, load_candidates
from digital_twin.sensor_identification.adapter import identify_frame, read_points, normalize_frame
from digital_twin.sensor_identification.demo import synthetic_points, write_viewer
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.matching import fit_model


class IdentificationTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = load_candidates()
        cls.config = IdentificationConfig(calibrated=True)

    def test_missing_sparse_invalid_and_no_contact(self):
        for points in (None, [], [{'id': 1, 'pressure': None}],
                       [dict(id=i, x=i, y=0, z=0, pressure=0) for i in range(12)],
                       synthetic_points(self.models['SENSOR_A'], count=3)):
            result = identify_sensor(points, self.models, self.config)
            self.assertEqual(result['classification'], 'AMBIGUOUS')
            self.assertFalse(result['fits'])

    def test_collinear_contacts_are_insufficient(self):
        p = [dict(id=i, x=i, y=0., z=0., pressure=700.) for i in range(12)]
        self.assertEqual(identify_sensor(p, self.models, self.config)['reason'],
                         'insufficient_spatial_coverage')

    def test_threshold_invalid_duplicate_and_nonfinite(self):
        p = [dict(id=i, x=i, y=i, z=0., pressure=200.) for i in range(12)]
        p += [dict(id=0, x=0, y=0, z=0, pressure=300),
              dict(id='bad', x=float('nan'), y=0, z=0, pressure=300),
              dict(id='missing', x=0, y=0, z=0, pressure=None)]
        result = identify_sensor(p, self.models, self.config)
        self.assertEqual(result['contacts'], 0)
        self.assertEqual(result['invalid_points'], 3)

    def test_stale_frame_and_units(self):
        self.assertEqual(identify_frame({'valid': False}, self.models)['reason'], 'invalid_or_stale_frame')
        with self.assertRaises(ValueError):
            identify_frame({'valid': True, 'points': [], 'frame': 'unknown'}, self.models)
        for kwargs in ({'tolerance_mm': 0}, {'coordinate_units': 'inch'}, {'refine_starts': 0}):
            with self.assertRaises(ValueError):
                IdentificationConfig(**kwargs)

    def test_current_twin_saved_metres_format(self):
        cloud = dict(valid=True, frame='left_base_link', units='metres', received_at=1000,
                     points=[dict(id=i, position_m=[.01*i, .02, .03], value=700) for i in range(3)])
        normalized = normalize_frame(cloud)
        self.assertEqual(normalized['units'], 'm')
        self.assertEqual(normalized['points'][1]['x'], .01)
        self.assertEqual(normalized['received_at'], 1000)
        self.assertEqual(cloud['units'], 'metres')
        result = identify_frame(cloud, self.models)
        self.assertEqual(result['contacts'], 3)
        self.assertEqual(result['classification'], 'AMBIGUOUS')

    def test_cad_unit_loading_and_triangle_distance(self):
        self.assertAlmostEqual(self.models['SENSOR_A'].mesh.extents[1], 118.9895, places=3)
        for model in self.models.values():
            p = model.samples[:12]
            self.assertLess(model.nearest(p)[1].max(), 1e-5)
            self.assertFalse(model.mesh.is_watertight)

    def test_full_coverage_b_and_rigid_pose(self):
        points = synthetic_points(self.models['SENSOR_B'])
        result = identify_sensor(points, self.models, self.config)
        self.assertEqual(result['classification'], 'SENSOR_B', json.dumps(result))
        for fit in result['fits'].values():
            r = np.array(fit['transform'])[:3, :3]
            np.testing.assert_allclose(r.T@r, np.eye(3), atol=1e-8)
            self.assertAlmostEqual(np.linalg.det(r), 1., places=8)
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp)/'viewer.html'
            write_viewer(path, points, self.models, result)
            text = path.read_text(encoding='utf-8')
            self.assertIn('SENSOR_B', text)
            self.assertIn('Valid non-contact', text)

    def test_full_coverage_a_noisy_different_pose_and_metres(self):
        points = synthetic_points(self.models['SENSOR_A'], seed=18, noise_mm=.35)
        r = Rotation.from_euler('xyz', [-70, 43, 17], degrees=True).as_matrix()
        for p in points:
            xyz = r@np.array([p[k] for k in ('x', 'y', 'z')])+[15, -70, 42]
            p.update(zip(('x', 'y', 'z'), (xyz/1000).tolist()))
        result = identify_sensor(points, self.models, replace(self.config, coordinate_units='m'))
        # A's metal body also occurs on B: do not require forced recognition.
        self.assertTrue(result['fits']['SENSOR_A']['compatible'], json.dumps(result))
        self.assertIn(result['classification'], ('SENSOR_A', 'AMBIGUOUS'))

    def test_shared_shaft_is_ambiguous(self):
        points = synthetic_points(self.models['SENSOR_A'], region='shared', noise_mm=.1)
        result = identify_sensor(points, self.models, self.config)
        self.assertEqual(result['classification'], 'AMBIGUOUS', json.dumps(result))
        self.assertEqual(result['reason'], 'both_candidates_compatible', json.dumps(result))

    def test_impossible_cloud_neither_fits(self):
        rng = np.random.default_rng(15)
        points = [dict(id=i, x=float(p[0]), y=float(p[1]), z=float(p[2]), pressure=900.)
                  for i, p in enumerate(rng.uniform(-250, 250, (16, 3)))]
        result = identify_sensor(points, self.models, self.config)
        self.assertEqual(result['reason'], 'neither_candidate_compatible_within_search')
        self.assertEqual(result['classification'], 'AMBIGUOUS')

    def test_noncontacts_ignored_and_uncalibrated_blocked(self):
        # Mock only fitting to isolate decision policy, not the geometry tests.
        from unittest.mock import patch
        points = synthetic_points(self.models['SENSOR_B'], count=12)
        points += [dict(id='zero', x=0., y=0., z=0., pressure=0.)]
        fits = [dict(compatible=True, score=.1, optimizer_success=True, consistent_refinements=4),
                dict(compatible=False, score=4., optimizer_success=True, consistent_refinements=4)]
        with patch('digital_twin.sensor_identification.classifier.fit_model', side_effect=fits) as fitter:
            result = identify_sensor(points, self.models, replace(self.config, calibrated=False))
            self.assertEqual(result['reason'], 'calibration_unverified')
            self.assertEqual(result['certified_exclusions'], 0)
            self.assertEqual(fitter.call_args.args[2].shape, (0, 3))

    def test_certified_surface_clearance_contradicts_box_contacts(self):
        import trimesh
        model = CandidateModel('box', trimesh.creation.box(extents=[10., 10., 10.]), sample_count=1000)
        config = replace(self.config, tolerance_mm=.1, random_starts=1,
                         refine_starts=2, icp_iterations=10, max_nfev=40)
        contact = model.mesh.vertices.copy()
        fit = fit_model(model, contact, np.array([[5., 0., 0.]]), np.array([2.]), config)
        self.assertFalse(fit['compatible'])
        self.assertGreater(fit['score'], 1.)


if __name__ == '__main__':
    unittest.main()
