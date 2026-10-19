import sys
from pathlib import Path
import unittest
from unittest.mock import patch
import numpy as np
import trimesh
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.limits import sampled_patch_penetrations, triangle_penetrations
from digital_twin.sensor_identification.matching import fit_model, _palm_starts
from digital_twin.sensor_identification.classifier import IdentificationConfig
from scipy.spatial.transform import Rotation

class CollisionOrientationTests(unittest.TestCase):
    def setUp(self):
        self.model=CandidateModel('box',trimesh.creation.box(extents=[20,8,6]),sample_count=1000)
    def test_search_sees_far_surface_even_when_nearest_is_outward(self):
        p=np.array([[0.,0.,2.]])
        n=np.array([[0.,0.,1.]])
        self.assertGreater(self.model.nearest(p)[0][0,2],p[0,2])
        depth=sampled_patch_penetrations(self.model.samples,p,n)
        self.assertAlmostEqual(depth[0],5.)
        r=Rotation.from_euler('xyz',[20,30,40],degrees=True).as_matrix()
        t=np.array([10.,20.,30.])
        np.testing.assert_allclose(sampled_patch_penetrations(self.model.samples@r.T+t,p@r.T+t,n@r.T),depth)
    def test_full_roll_coverage_for_palm_starts(self):
        palm=np.array([[x,y,-10.] for x in (-20.,0.,20.) for y in (-10.,0.,10.)])
        starts,normal=_palm_starts(self.model,self.model.mesh.vertices,palm,np.tile([0.,0.,1.],(9,1)))
        self.assertEqual(len(starts),48)
        rolls=[starts[i*3][0] for i in range(8)]
        self.assertEqual(len({tuple(np.round(r.flatten(),5)) for r in rolls}),8)
        self.assertAlmostEqual(Rotation.from_matrix(rolls[4]@rolls[0].T).magnitude(),np.pi)
    def test_quick_mode_audits_full_hand_and_rejects_enclosure(self):
        config=IdentificationConfig(quick_preview=True,tolerance_mm=.01,random_starts=1,refine_starts=2,icp_iterations=1,max_nfev=1)
        with patch('digital_twin.sensor_identification.matching.triangle_penetrations',wraps=triangle_penetrations) as audit:
            fit=fit_model(self.model,self.model.mesh.vertices,np.empty((0,3)),np.array([]),config,
                          boundary_xyz=np.array([[0.,0.,0.]]),boundary_normals=np.array([[0.,0.,1.]]))
        self.assertEqual(audit.call_count,2)
        self.assertFalse(fit['compatible'])
        self.assertEqual(fit['validation_method'],'exact_full_mesh')
        self.assertIn('hand_penetration',fit['rejection_reasons'])
if __name__=='__main__':unittest.main()
