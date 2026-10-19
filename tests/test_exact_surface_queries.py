"""Exact acceleration must agree with all triangles, including open meshes."""
import unittest
import numpy as np
import trimesh

from digital_twin.sensor_identification.geometry import CandidateModel, load_candidates


class ExactSurfaceQueryTests(unittest.TestCase):
    def test_open_thin_triangle_near_far_edges_and_empty(self):
        mesh = trimesh.Trimesh(vertices=[[-80., 0., 0.], [80., 0., 0.], [0., .05, 0.]],
                               faces=[[0, 1, 2]], process=False)
        model = CandidateModel('open', mesh, sample_count=100)
        points = np.array([[0., .02, 0.], [0., .02, .1], [0., -.1, 2.],
                           [200., 100., -30.], [-80., 0., 0.]])
        closest, distances = model.nearest(points)
        expected, expected_distances, _ = trimesh.proximity.closest_point_naive(mesh, points)
        np.testing.assert_allclose(distances, expected_distances, atol=1e-8)
        np.testing.assert_allclose(closest, expected, atol=1e-8)
        empty_points, empty_distances = model.nearest(np.empty((0, 3)))
        self.assertEqual(empty_points.shape, (0, 3))
        self.assertEqual(empty_distances.shape, (0,))

    def test_actual_cads_match_reference_for_near_and_distant_queries(self):
        rng = np.random.default_rng(51)
        for name, model in load_candidates().items():
            with self.subTest(name=name):
                points = np.vstack((model.samples[:40]+rng.normal(0., .3, (40, 3)),
                                    rng.uniform(-150., 150., (40, 3)), model.samples[100:104]))
                closest, distance = model.nearest(points)
                _, expected, _ = trimesh.proximity.closest_point(model.mesh, points)
                np.testing.assert_allclose(distance, expected, rtol=1e-8, atol=1e-7)
                np.testing.assert_allclose(np.linalg.norm(closest-points, axis=1), distance, atol=1e-8)


if __name__ == '__main__':
    unittest.main()
