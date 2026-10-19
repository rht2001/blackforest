import sys,unittest
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
import numpy as np
from scipy.spatial.transform import Rotation
sys.path.insert(0,'C:/Users/karl/Desktop/personal_branch_hand/blackforest')
from digital_twin.sensor_identification.geometry import load_candidates, CandidateModel
from digital_twin.sensor_identification.grasp_prior import empirical_pose
from digital_twin.sensor_identification.matching import fit_model
from digital_twin.sensor_identification.classifier import IdentificationConfig
from digital_twin.sensor_identification.adapter import identify_frame

class EmpiricalGraspTests(unittest.TestCase):
 @classmethod
 def setUpClass(cls):cls.models=load_candidates()
 def test_provenance_frame_and_independent_pose_values(self):
  for name,m in self.models.items():
   p=empirical_pose(m,'left_base_link');self.assertEqual(p['source_recordings'],53)
   self.assertIsNone(empirical_pose(m,'world'))
   p['center_mm'][0]=999;self.assertNotEqual(empirical_pose(m,'left_base_link')['center_mm'][0],999)
   fake=CandidateModel(name,m.mesh,100);self.assertIsNone(empirical_pose(fake,'left_base_link'))
 def test_frame_routing_and_unit_conversion(self):
  frame=dict(valid=True,units='m',frame='left_base_link',points=[dict(id=i,x=i*.002,y=(i%3)*.01,z=(i%5)*.01,pressure=800) for i in range(20)])
  with patch('digital_twin.sensor_identification.classifier.fit_model',return_value=dict(compatible=False,score=1,rejection_reasons=[])) as fit:
   identify_frame(frame,self.models,quick_preview=True)
   self.assertEqual(fit.call_count,2);self.assertIn('initial_pose',fit.call_args.kwargs)
   np.testing.assert_allclose(fit.call_args.args[1][1],[2,10,10])
   frame['frame']='world';identify_frame(frame,self.models,quick_preview=True)
   self.assertNotIn('initial_pose',fit.call_args.kwargs)
 def test_real_cads_start_at_centers_and_recover_nearby_pose(self):
  for name,m in self.models.items():
   with self.subTest(name=name):
    p=empirical_pose(m,'left_base_link');r=Rotation.from_euler('xyz',p['rotation_xyz_deg'],degrees=True).as_matrix();center=np.array(p['center_mm'])
    t=center-r@m.mesh.bounds.mean(0);sample=m.samples[np.linspace(0,len(m.samples)-1,80,dtype=int)]
    contacts=sample@r.T+t+[3,-1.5,2];events=[]
    fit=fit_model(m,contacts,np.empty((0,3)),np.empty(0),IdentificationConfig(quick_preview=True,point_spacing_mm=0,max_nfev=25),initial_pose=p,progress=events.append)
    initial=np.array(fit['initial_transform']);np.testing.assert_allclose(initial[:3,:3]@m.mesh.bounds.mean(0)+initial[:3,3],center)
    self.assertTrue(fit['pose_prior_used']);self.assertLess(fit['rmse_mm'],1.0)
    for event in events:
     if event.get('preview'):
      tr=np.array(event['transform']);actual=tr[:3,:3]@m.mesh.bounds.mean(0)+tr[:3,3]
      self.assertTrue(np.all(abs(actual-center)<=np.array(p['position_window_mm'])+1e-7))
 def test_distant_contact_cannot_escape_window_or_bypass_collision(self):
  m=self.models['SENSOR_A'];p=empirical_pose(m,'left_base_link');r=Rotation.from_euler('xyz',p['rotation_xyz_deg'],degrees=True).as_matrix();center=np.array(p['center_mm']);t=center-r@m.mesh.bounds.mean(0)
  contacts=m.samples[:24]@r.T+t+[100,0,0]
  fit=fit_model(m,contacts,np.empty((0,3)),np.empty(0),IdentificationConfig(quick_preview=True,max_nfev=3),initial_pose=p)
  tr=np.array(fit['transform']);self.assertTrue(np.all(abs(tr[:3,:3]@m.mesh.bounds.mean(0)+tr[:3,3]-center)<=np.array(p['position_window_mm'])+1e-7))
  self.assertFalse(fit['compatible'])
  with patch('digital_twin.sensor_identification.matching.triangle_penetrations',return_value=np.array([4.])) as audit:
   fit=fit_model(m,m.samples[:24]@r.T+t,np.empty((0,3)),np.empty(0),IdentificationConfig(quick_preview=True,max_nfev=2,tolerance_mm=100),initial_pose=p,boundary_xyz=np.array([[1000.,1000,1000]]),boundary_normals=np.array([[0.,1,0]]))
   self.assertTrue(audit.called);self.assertFalse(fit['compatible']);self.assertIn('hand_penetration',fit['rejection_reasons'])
 def test_empty_frame_does_not_invent_a_pose(self):
  result=identify_frame(dict(valid=True,units='m',frame='left_base_link',points=[]),self.models,quick_preview=True)
  self.assertEqual(result['fits'],{});self.assertEqual(result['classification'],'AMBIGUOUS')

 def test_full_shaft_roll_survives_refinement_parameterization(self):
  from types import SimpleNamespace
  m=self.models['SENSOR_B'];p=empirical_pose(m,'left_base_link');base=Rotation.from_euler('xyz',p['rotation_xyz_deg'],degrees=True).as_matrix();calls=[]
  def freeze(fun,x0,**kwargs):
   x0=np.array(x0);calls.append(len(x0))
   if len(calls)==1:x0=np.array([0.,np.deg2rad(170.),0.])
   fun(x0);return SimpleNamespace(x=x0,success=True,nfev=1)
  with patch('digital_twin.sensor_identification.matching.least_squares',side_effect=freeze):
   f=fit_model(m,m.samples[:30],np.empty((0,3)),np.empty(0),IdentificationConfig(quick_preview=True),initial_pose=p)
  self.assertEqual(calls,[3,3,6]);np.testing.assert_allclose(np.array(f['transform'])[:3,:3],base@Rotation.from_euler('y',170,degrees=True).as_matrix(),atol=1e-8)
 def test_detailed_search_can_recover_beyond_quick_window(self):
  m=self.models['SENSOR_A'];p=empirical_pose(m,'left_base_link');r=Rotation.from_euler('xyz',p['rotation_xyz_deg'],degrees=True).as_matrix();t=np.array(p['center_mm'])-r@m.mesh.bounds.mean(0)+[70,0,0]
  x=m.samples[np.linspace(0,len(m.samples)-1,80,dtype=int)]@r.T+t
  f=fit_model(m,x,np.empty((0,3)),np.empty(0),IdentificationConfig(random_starts=1,icp_iterations=15,max_nfev=20,refine_starts=2),initial_pose=p)
  self.assertTrue(f['compatible']);self.assertFalse(f['pose_prior_used']);self.assertGreater(f['starts_evaluated'],1)
  self.assertLess(f['rmse_mm'],.8)

if __name__=='__main__':unittest.main()
