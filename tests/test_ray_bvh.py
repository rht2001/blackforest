import unittest
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.ray_bvh import TriangleBVH
from digital_twin.sensor_identification.geometry import CandidateModel


class RayBVHTests(unittest.TestCase):
 def test_random_rays_match_complete_triangle_reference(self):
  mesh=trimesh.util.concatenate([trimesh.creation.box(extents=[10,30,12]),trimesh.creation.icosphere(subdivisions=2,radius=4).apply_translation([12,0,0])])
  rng=np.random.default_rng(123);origins=rng.normal(0,25,(500,3));directions=rng.normal(size=(500,3));directions/=np.linalg.norm(directions,axis=1)[:,None]
  locations,ids,_=mesh.ray.intersects_location(origins,directions,multiple_hits=True)
  reference=np.full(len(origins),np.inf);np.minimum.at(reference,ids,np.einsum('ij,ij->i',locations-origins[ids],directions[ids]))
  actual,faces=TriangleBVH(mesh).first(origins,directions)
  np.testing.assert_array_equal(np.isfinite(reference),faces>=0)
  good=np.isfinite(reference);np.testing.assert_allclose(actual[good],reference[good],atol=1e-8,rtol=0)
 def test_parallel_inside_outside_and_seam_rays(self):
  mesh=trimesh.creation.box(extents=[10,20,30]);bvh=TriangleBVH(mesh)
  origins=np.array([[-20,0,0],[-20,11,0],[0,0,0],[-20,10,15],[-20,10,16]])
  distance,face=bvh.first(origins,np.tile([1.,0,0],(len(origins),1)))
  np.testing.assert_allclose(distance[[0,2,3]],[15,5,15]);self.assertTrue(np.isinf(distance[[1,4]]).all())
 def test_empty_rays(self):
  d,f=TriangleBVH(trimesh.creation.box()).first(np.empty((0,3)),np.empty((0,3)))
  self.assertEqual(len(d),0);self.assertEqual(len(f),0)
 def test_nonunit_direction_preserves_ray_parameter(self):
  d,_=TriangleBVH(trimesh.creation.box(extents=[10,20,30])).first([[-20,0,0]],[[2.,0,0]])
  np.testing.assert_allclose(d,[7.5])
 def test_known_segment_end_rejects_later_faces(self):
  bvh=TriangleBVH(trimesh.creation.box(extents=[10,20,30]))
  d,f=bvh.first([[-20.,0,0],[-20.,0,0]],[[1.,0,0],[1.,0,0]],max_distance=[15.,14.])
  self.assertAlmostEqual(d[0],15.);self.assertTrue(np.isinf(d[1]));self.assertEqual(f[1],-1)
 def test_contains_matches_watertight_reference(self):
  mesh=trimesh.creation.icosphere(subdivisions=2,radius=10)
  points=np.random.default_rng(31).normal(0,8,(500,3))
  np.testing.assert_array_equal(TriangleBVH(mesh).contains(points),mesh.contains(points))
  mesh.invert()
  np.testing.assert_array_equal(TriangleBVH(mesh).contains(points),mesh.contains(points))


if __name__=='__main__':unittest.main()
