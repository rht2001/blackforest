import unittest
from types import SimpleNamespace
import numpy as np
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.grasp_prior import empirical_pose

SMALL='bcf73f55eb4674d01f7de225ce0e07d3e3d15954aff24f2f3e1519a6b050c974'
LARGE='ec9ec5eebfb97d6af6623371953eb2bb4836858568952cb12f6c77e5b38e2366'

class GraspStartTests(unittest.TestCase):
    def test_small_start_is_half_turn_about_its_own_shaft(self):
        pose=empirical_pose(SimpleNamespace(source_sha256=SMALL),'left_base_link')
        old=Rotation.from_euler('xyz',[-3.1826635762,-2.7735447350,91.1918856940],degrees=True).as_matrix()
        new=Rotation.from_euler('xyz',pose['rotation_xyz_deg'],degrees=True).as_matrix()
        np.testing.assert_allclose(new[:,1],old[:,1],atol=1e-12)
        np.testing.assert_allclose(new[:,[0,2]],-old[:,[0,2]],atol=1e-12)
        np.testing.assert_allclose(pose['center_mm'],[20.0090457787,23.5735870142,129.2507983112],atol=1e-12)

    def test_large_sensor_start_is_unchanged(self):
        pose=empirical_pose(SimpleNamespace(source_sha256=LARGE),'left_base_link')
        self.assertEqual(pose['rotation_xyz_deg'],[-179.5800221948,-3.2957311234,90.4413873626])

    def test_start_remains_limited_to_known_geometry_and_base_frame(self):
        self.assertIsNone(empirical_pose(SimpleNamespace(source_sha256=SMALL),'world'))
        self.assertIsNone(empirical_pose(SimpleNamespace(source_sha256='unknown'),'left_base_link'))

if __name__=='__main__':unittest.main()
