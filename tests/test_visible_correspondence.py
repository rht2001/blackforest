import unittest
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.visible_surface import first_surface
from digital_twin.sensor_identification.visible_correspondence import visible_correspondences


class VisibleCorrespondenceTests(unittest.TestCase):
 def setUp(self):
  self.model=CandidateModel('box',trimesh.creation.box(extents=[10,20,30]),sample_count=1000)
 def test_far_wall_is_still_rejected(self):
  p=np.array([[5.,0,0]]);n=np.array([[1.,0,0]])
  q,d,_,hit,occlusion=visible_correspondences(self.model,p,n,np.eye(3),np.zeros(3),exact=True)
  self.assertTrue(hit[0]);np.testing.assert_allclose(q,[[-5,0,0]]);np.testing.assert_allclose(d,[10]);np.testing.assert_allclose(occlusion,[10])
 def test_edge_within_positional_tolerance_is_not_a_ray_miss(self):
  p=np.array([[-6.,0,16.]]);n=np.array([[1.,0,0]])
  self.assertTrue(np.isinf(first_surface(self.model,p,n,np.eye(3),np.zeros(3))[0]))
  q,d,_,hit,occlusion=visible_correspondences(self.model,p,n,np.eye(3),np.zeros(3),exact=True)
  self.assertTrue(hit[0]);np.testing.assert_allclose(d,[np.sqrt(2)],atol=1e-8)
  np.testing.assert_allclose(q,[[-5,0,15]],atol=1e-8)
 def test_internal_assembly_wall_cannot_hide_outer_wall(self):
  m=CandidateModel('assembly',trimesh.util.concatenate([self.model.mesh,trimesh.creation.box(extents=[4,8,12])]),sample_count=1000)
  p=np.array([[2.,0,0]]);n=np.array([[1.,0,0]])
  q,d,_,hit,occlusion=visible_correspondences(m,p,n,np.eye(3),np.zeros(3),exact=True)
  np.testing.assert_allclose(d,[7]);np.testing.assert_allclose(q,[[-5,0,0]]);np.testing.assert_allclose(occlusion,[7])
 def test_rigid_frame_invariance(self):
  r=Rotation.from_euler('xyz',[.3,-.2,.4]).as_matrix();t=np.array([20.,-10,130])
  p=np.array([[-6.,0,16.],[5.,0,0]]);n=np.tile([1.,0,0],(2,1))
  q,d,_,hit,_=visible_correspondences(self.model,p@r.T+t,n@r.T,r,t,exact=True)
  np.testing.assert_allclose(d,[np.sqrt(2),10],atol=1e-8)
 def test_winding_independence(self):
  mesh=self.model.mesh.copy();mesh.invert();m=CandidateModel('inverted',mesh,sample_count=1000)
  p=np.array([[5.,0,0]]);n=np.array([[1.,0,0]])
  np.testing.assert_allclose(visible_correspondences(m,p,n,np.eye(3),np.zeros(3),exact=True)[1],[10])

if __name__=='__main__':unittest.main()
