import sys
import unittest
from dataclasses import replace
from unittest.mock import patch
from pathlib import Path
import json
import numpy as np
import trimesh

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from digital_twin.sensor_identification.thumb_prior import thumb_closure_hint
from digital_twin.sensor_identification.classifier import identify_sensor, IdentificationConfig
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.matching import fit_model
from digital_twin.sensor_identification.adapter import identify_frame


class ThumbFirstTests(unittest.TestCase):
    def setUp(self):
        self.frame = dict(valid=True, units='mm', frame='left_base_link',
            hand_snapshot=dict(angles_act=[1000,1000,1000,1000,100,500],
                kinematics_calibration={'actuators': {'thumb_bend': dict(raw_at_open=1000,raw_at_closed=0)}},
                points=[dict(region='thumb_tip', value=700) for _ in range(3)]))
        self.small = CandidateModel('small', trimesh.creation.box(extents=[10,10,10]),sample_count=100)
        self.large = CandidateModel('large', trimesh.creation.box(extents=[20,20,20]),sample_count=100)
        self.models = dict(SENSOR_A=self.small,SENSOR_B=self.large)
        self.frame['points'] = [dict(id=i,x=float(x),y=float(y),z=float(z),pressure=700)
            for i,(x,y,z) in enumerate(self.large.mesh.vertices)]

    def test_contact_near_closed_and_boundary(self):
        self.assertAlmostEqual(thumb_closure_hint(self.frame,200)['closure_fraction'],.9)
        for raw, expected in [(150,True),(151,False),(1000,False)]:
            self.frame['hand_snapshot']['angles_act'][4]=raw
            self.assertEqual(bool(thumb_closure_hint(self.frame,200)),expected)

    def test_unknown_pressure_threshold_sparse_and_missing_calibration(self):
        points=self.frame['hand_snapshot']['points']
        for value in [None,0,200,float('nan'),True,5000]:
            for p in points:p['value']=value
            self.assertIsNone(thumb_closure_hint(self.frame,200))
        for p in points:p['value']=700
        points.pop()
        self.assertIsNone(thumb_closure_hint(self.frame,200))
        self.frame['hand_snapshot'].pop('kinematics_calibration')
        self.assertIsNone(thumb_closure_hint(self.frame,200))

    def test_reversed_and_invalid_angles(self):
        self.frame['hand_snapshot']['kinematics_calibration']['actuators']['thumb_bend']['reversed']=True
        self.assertIsNone(thumb_closure_hint(self.frame,200))
        self.frame['hand_snapshot']['angles_act'][4]=900
        self.assertIsNotNone(thumb_closure_hint(self.frame,200))
        for value in [1001,-1,None,float('nan'),True]:
            self.frame['hand_snapshot']['angles_act'][4]=value
            self.assertIsNone(thumb_closure_hint(self.frame,200))

    def test_run_uses_contact_pose_instead_of_last_display_pose(self):
        self.frame['measurement_source']='csv_run'
        self.assertIsNone(thumb_closure_hint(self.frame,200))
        self.frame['thumb_closure_sample']=dict(angles_act=[1000,1000,1000,1000,20,500],thumb_pressures=[600]*3)
        self.frame['hand_snapshot']['angles_act'][4]=1000
        self.assertAlmostEqual(thumb_closure_hint(self.frame,200)['closure_fraction'],.98)

    def fake_fit(self, compatible=True):
        return dict(compatible=compatible,score=1.,optimizer_success=True,consistent_refinements=2,rejection_reasons=[])

    def test_small_first_progress_and_early_candidate_stop(self):
        events=[]
        with patch('digital_twin.sensor_identification.classifier.fit_model',return_value=self.fake_fit()) as fit:
            result=identify_frame(self.frame,self.models,quick_preview=True,progress=events.append)
        self.assertEqual(fit.call_count,1)
        self.assertIs(fit.call_args.args[0],self.small)
        self.assertEqual(result['placement_candidate'],'SENSOR_A')
        self.assertEqual(result['classification'],'AMBIGUOUS')
        self.assertEqual(events[0]['first_guess']['candidate'],'SENSOR_A')

    def test_failed_thumb_guess_falls_back(self):
        with patch('digital_twin.sensor_identification.classifier.fit_model',side_effect=[self.fake_fit(False),self.fake_fit()]) as fit:
            result=identify_frame(self.frame,self.models,quick_preview=True)
        self.assertEqual([c.args[0].name for c in fit.call_args_list],['small','large'])
        self.assertEqual(result['placement_candidate'],'SENSOR_B')
        self.assertEqual(result['placement_reason'],'geometric_fallback_after_thumb_guess')

    def test_detailed_keeps_both_and_no_hint_keeps_large_first(self):
        with patch('digital_twin.sensor_identification.classifier.fit_model',return_value=self.fake_fit()) as fit:
            result=identify_frame(self.frame,self.models,quick_preview=False)
            self.assertEqual(fit.call_count,2)
            self.assertEqual(result['reason'],'both_candidates_compatible')
            self.frame['hand_snapshot']['angles_act'][4]=500
            result=identify_frame(self.frame,self.models,quick_preview=True)
            self.assertEqual(result['candidate_order'],['SENSOR_B','SENSOR_A'])
            self.assertNotIn('first_guess',result)

    def test_real_quick_fit_stops_only_after_full_hand_check(self):
        config=IdentificationConfig(quick_preview=True,random_starts=1,refine_starts=3,icp_iterations=1,
            tolerance_mm=100.,hand_tolerance_mm=3.,point_spacing_mm=0)
        contacts=self.small.mesh.vertices.copy()
        kwargs=dict(boundary_xyz=np.array([[0.,0.,20.]]),boundary_normals=np.array([[0.,0.,1.]]))
        with patch('digital_twin.sensor_identification.matching.triangle_penetrations',side_effect=[np.array([4.]),np.array([0.])]) as audit:
            fit=fit_model(self.small,contacts,np.empty((0,3)),np.array([]),config,**kwargs)
        self.assertEqual(audit.call_count,2)
        self.assertEqual(fit['refinements'],2)
        self.assertTrue(fit['compatible'])

    def test_short_pass_failure_retries_same_candidate_with_full_budget(self):
        config=IdentificationConfig(quick_preview=True)
        with patch('digital_twin.sensor_identification.matching._fit_model',side_effect=[self.fake_fit(False),self.fake_fit()]) as fit:
            result=fit_model(self.small,np.zeros((8,3)),np.empty((0,3)),np.array([]),config)
        self.assertTrue(fit.call_args_list[0].kwargs['fast_attempt'])
        self.assertNotIn('fast_attempt',fit.call_args_list[1].kwargs)
        self.assertEqual(result['search_pass'],'full_fallback')
        with patch('digital_twin.sensor_identification.matching._fit_model',return_value=self.fake_fit()) as fit:
            result=fit_model(self.small,np.zeros((8,3)),np.empty((0,3)),np.array([]),replace(config,quick_preview=False))
        self.assertEqual(fit.call_count,1)
        self.assertEqual(result['search_pass'],'detailed')

if __name__=='__main__':unittest.main(verbosity=2)
