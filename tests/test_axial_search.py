import unittest
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.axial_search import axial_pose,roll_angles,screen_rolls
from digital_twin.sensor_identification.geometry import CandidateModel


class AxialSearchTests(unittest.TestCase):
    def test_full_period_has_no_duplicate_endpoint_or_missing_half_turn(self):
        for quick,step in [(False,15),(True,30)]:
            angles=np.rad2deg(roll_angles(quick))
            np.testing.assert_allclose(angles,np.arange(0,360,step))
            self.assertEqual(np.count_nonzero(np.isclose(angles,180)),1)

    def test_roll_preserves_center_and_world_shaft_for_every_local_axis(self):
        reference=Rotation.from_euler('xyz',[.4,-.7,.2]).as_matrix()
        center=np.array([10.,-30.,120.]);model_center=np.array([8.,17.,-12.])
        for axis in range(3):
            for angle in roll_angles():
                r,t=axial_pose(np.r_[np.zeros(6),angle],reference,center,model_center,axis)
                np.testing.assert_allclose(r@model_center+t,center,atol=1e-12)
                np.testing.assert_allclose(r[:,axis],reference[:,axis],atol=1e-12)
                self.assertAlmostEqual(np.linalg.det(r),1.)

    def test_screen_retains_quarter_turn_outside_old_twenty_degree_bounds(self):
        model=CandidateModel('box',trimesh.creation.box(extents=[10,20,50]),sample_count=2000)
        points=np.array([[-10.,y,z] for y in [-4.,0.,4.] for z in [-16.,0.,16.]])
        normals=np.tile([1.,0,0],(len(points),1));events=[]
        seeds,info=screen_rolls(model,points,normals,['finger']*len(points),np.eye(3),np.zeros(3),[4,4,4],1.,progress=events.append)
        self.assertTrue(any(abs(np.cos(s[6]))<1e-8 for s in seeds))
        self.assertEqual(info['axial_roll_placements_screened'],72)
        self.assertEqual(events[-1]['completed'],24)

if __name__=='__main__':unittest.main()
