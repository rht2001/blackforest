import sys,unittest
from pathlib import Path
from unittest.mock import patch
import numpy as np,trimesh
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.finite_hand import FiniteHand
from digital_twin.sensor_identification.classifier import compare_anchored_candidates
class CoverageTests(unittest.TestCase):
 def test_measured_thumb_pose_adds_collision_absent_in_frozen_pose(self):
  mesh=trimesh.creation.box(extents=[4,4,4]);mesh.apply_translation([20,0,0])
  model=CandidateModel('thumb',mesh,sample_count=100)
  joint=dict(name='thumb',parent='left_base_link',child='left_thumb_2',type='revolute')
  profile=dict(root_link='left_base_link',joints=[joint])
  snapshot=dict(frame='left_base_link',profile='rh56_tactile_toolkit_left',reference_sha256='ref',
   kinematics_calibration=dict(joints=dict(thumb=dict(origin_xyz_m=[0,0,0],origin_rpy_rad=[0,0,0],axis_xyz=[0,0,1]))),joint_angles_rad=dict(thumb=0))
  with patch('digital_twin.sensor_identification.finite_hand._assets',return_value=(profile,[('left_thumb_2',model)],[],'ref')):
   frozen=FiniteHand.from_snapshot(snapshot)
   self.assertEqual(frozen.depths([[-20,0,0]])[0],0)
   snapshot['motion_samples']=[dict(link_prefix='left_thumb_',source_index='42',joint_angles_rad=dict(thumb=np.pi))]
   swept=FiniteHand.from_snapshot(snapshot)
   self.assertAlmostEqual(swept.depths([[-20,0,0]])[0],2)
   self.assertEqual(swept.motion_pose_count,1)
   snapshot['motion_samples'][0]['link_prefix']='unrelated'
   self.assertFalse(FiniteHand.from_snapshot(snapshot).parts)
 def test_nearly_equal_fit_has_no_size_evidence(self):
  def fit(error=2.8,usable=True):return dict(placement_usable=usable,visible_surface_score_mm=error,physical_contact_region_support=dict(index=1,palm=.48))
  fits=dict(SENSOR_A=fit(2.77),SENSOR_B=fit())
  result=compare_anchored_candidates(fits,3)
  self.assertTrue(result['missing_thumb_contacts']);self.assertLess(result['balanced_error_gap_mm'],.04)
  fits['SENSOR_A']=fit(5)
  self.assertIsNone(compare_anchored_candidates(fits,3))
  fits['SENSOR_A']=fit(2.8,False)
  self.assertIsNone(compare_anchored_candidates(fits,3))
if __name__=='__main__':unittest.main()
