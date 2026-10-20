import unittest
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.axial_search import axial_pose,roll_angles,screen_rolls
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.anchored_fit import fit_anchored
from digital_twin.sensor_identification.classifier import IdentificationConfig


class AnchoredRollTests(unittest.TestCase):
    def test_local_fit_can_recover_quarter_turn_without_translating_shaft(self):
        model=CandidateModel('box',trimesh.creation.box(extents=[10,20,50]),sample_count=2000)
        # Opposite contact faces prevent translation from hiding a wrong roll.
        points=np.array([[x,y,z] for x in [-10.,10.] for y in [-4.,0.,4.] for z in [-16.,0.,16.]])
        normals=np.zeros_like(points);normals[:,0]=-np.sign(points[:,0])
        prior=dict(center_mm=[0,0,0],rotation_xyz_deg=[0,0,0],position_window_mm=[1.,1.,1.])
        fit=fit_anchored(model,points,normals,['finger']*len(points),np.empty((0,3)),np.empty(0),
            IdentificationConfig(quick_preview=True,tolerance_mm=1.),prior)
        self.assertGreater(fit['visible_surface_inlier_fraction'],.99)
        self.assertLess(abs(np.cos(np.deg2rad(fit['axial_roll_deg']))),.03)
        self.assertEqual(fit['axial_roll_search_degrees'],360)
        self.assertFalse(fit['compatible'])
