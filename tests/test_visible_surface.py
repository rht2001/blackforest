import sys,unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parent))
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.visible_surface import first_surface,_balanced_indices,_weights,fit_conflicted_surface
from digital_twin.sensor_identification.classifier import IdentificationConfig

class VisibilityTests(unittest.TestCase):
 def setUp(self):
  self.model=CandidateModel('box',trimesh.creation.box(extents=[10,20,30]),sample_count=1000)
 def test_first_wall_rejects_false_far_wall_match(self):
  # Both are exactly on the mesh. Only the near wall is a valid contact
  # when approaching from -X through the outward taxel direction +X.
  p=np.array([[-5.,0,0],[5.,0,0]])
  self.assertTrue(np.allclose(self.model.nearest(p)[1],0))
  np.testing.assert_allclose(first_surface(self.model,p,np.tile([1.,0,0],(2,1)),np.eye(3),np.zeros(3)),[0,-10])
 def test_first_wall_ignores_internal_assembly_faces(self):
  inside=trimesh.creation.box(extents=[4,8,12])
  m=CandidateModel('assembly',trimesh.util.concatenate([self.model.mesh,inside]),sample_count=1000)
  np.testing.assert_allclose(first_surface(m,np.array([[-2.,0,0]]),np.array([[1.,0,0]]),np.eye(3),np.zeros(3)),[-3])
 def test_miss_is_reported_not_zero(self):
  d=first_surface(self.model,np.array([[0.,50,0]]),np.array([[1.,0,0]]),np.eye(3),np.zeros(3))
  self.assertTrue(np.isinf(d[0]))
 def test_winding_does_not_change_visibility(self):
  inverted=self.model.mesh.copy();inverted.invert();m=CandidateModel('inverted',inverted,sample_count=1000)
  p=np.array([[5.,0,0]]);n=np.array([[1.,0,0]])
  np.testing.assert_allclose(first_surface(m,p,n,np.eye(3),np.zeros(3)),[-10])
 def test_frame_rotation_and_translation(self):
  r=Rotation.from_euler('xyz',[.6,-.3,.2]).as_matrix();t=np.array([40.,-20,100])
  p=np.array([[-5.,0,0],[5.,0,0]])@r.T+t;n=np.tile([1.,0,0],(2,1))@r.T
  np.testing.assert_allclose(first_surface(self.model,p,n,r,t),[0,-10],atol=1e-8)
 def test_sparse_region_kept_in_balanced_search(self):
  g=np.array(['palm']*100+['thumb']*3+['index']*30)
  ids=_balanced_indices(g,30)
  self.assertEqual(sum(g[ids]=='thumb'),3);self.assertEqual(sum(g[ids]=='palm'),10)
 def test_each_region_has_equal_total_weight_after_reduction(self):
  g=np.array(['palm']*100+['thumb']*3+['index']*30)
  for ids in (np.arange(len(g)),_balanced_indices(g,30)):
   reduced=g[ids];w=_weights(reduced)
   totals=[w[reduced==label].sum() for label in ('palm','thumb','index')]
   np.testing.assert_allclose(totals,[totals[0]]*3)
 def test_consistent_input_retains_existing_complete_hand_solver(self):
  p=np.array([[-5.,0,0],[5.,0,0],[0,0,15.]])
  self.assertIsNone(fit_conflicted_surface(self.model,p,np.empty((0,3)),np.empty(0),IdentificationConfig(),
    contact_normals=np.eye(3),contact_groups=['thumb','palm','index']))

if __name__=='__main__':unittest.main()
