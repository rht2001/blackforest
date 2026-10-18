from dataclasses import replace
from unittest.mock import patch
import unittest
import numpy as np
import trimesh
from digital_twin.sensor_identification.matching import _spaced_indices, fit_model
from digital_twin.sensor_identification.classifier import IdentificationConfig
from digital_twin.sensor_identification.geometry import CandidateModel

class PointSpacingTests(unittest.TestCase):
    def test_radius_merges_across_grid_edges_and_keeps_isolated_points(self):
        points=np.array([[1.99,0,0],[2.01,0,0],[2.1,0,0],[8,0,0]],dtype=float)
        original=points.copy()
        np.testing.assert_array_equal(_spaced_indices(points,2),[0,3])
        np.testing.assert_array_equal(points,original)
    def test_each_removed_point_has_a_representative_within_spacing(self):
        points=np.random.default_rng(2).uniform(-10,10,(200,3))
        chosen=_spaced_indices(points,2)
        d=np.linalg.norm(points[:,None,:]-points[chosen][None,:,:],axis=2)
        self.assertTrue((d.min(axis=1)<=2+1e-10).all())
        distances=d[chosen]
        np.fill_diagonal(distances,np.inf)
        self.assertTrue((distances>2).all())
        np.testing.assert_array_equal(chosen,_spaced_indices(points,2))
    def test_opposing_and_sharp_hand_normals_stay_separate(self):
        points=np.zeros((4,3));normals=np.array([[0,0,1],[0,0,1],[0,0,-1],[1,0,0]])
        np.testing.assert_array_equal(_spaced_indices(points,2,normals),[0,2,3])
    def test_disabled_and_invalid_spacing(self):
        points=np.zeros((3,3))
        np.testing.assert_array_equal(_spaced_indices(points,0),[0,1,2])
        self.assertEqual(len(_spaced_indices(np.empty((0,3)),2)),0)
        for value in (-1,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError): IdentificationConfig(point_spacing_mm=value)
    def test_fitting_and_final_checks_share_representatives(self):
        model=CandidateModel('box',trimesh.creation.box(extents=[12,20,60]),sample_count=100)
        contacts=np.repeat(model.mesh.vertices,10,axis=0)
        hand=np.repeat([[0.,0.,-40.]],20,axis=0)
        normals=np.tile([0.,0.,1.],(20,1))
        original=contacts.copy()
        config=IdentificationConfig(quick_preview=True,random_starts=1,refine_starts=1,icp_iterations=1,max_nfev=1)
        with patch('digital_twin.sensor_identification.matching.triangle_penetrations',return_value=np.zeros(1)) as audit:
            fit=fit_model(model,contacts,np.empty((0,3)),np.array([]),config,boundary_xyz=hand,boundary_normals=normals)
        self.assertEqual(fit['input_contact_points'],80)
        self.assertEqual(fit['scored_contact_points'],8)
        self.assertEqual(fit['search_contact_points'],8)
        self.assertEqual(fit['input_hand_points'],20)
        self.assertEqual(fit['hand_constraints_checked'],1)
        self.assertEqual(len(audit.call_args.args[3]),1)
        np.testing.assert_array_equal(contacts,original)

if __name__=='__main__':unittest.main()
