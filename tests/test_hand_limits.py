from pathlib import Path
import sys
import unittest
from unittest.mock import patch
import numpy as np

sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from digital_twin.sensor_identification.limits import penetration_residual, valid_boundaries
from digital_twin.sensor_identification.classifier import identify_sensor, IdentificationConfig
from digital_twin.sensor_identification.adapter import identify_frame


class LimitsTests(unittest.TestCase):
    def test_oriented_local_penetration(self):
        p=np.zeros((4,3));n=np.tile([0,0,1.],(4,1))
        q=np.array([[0,0,5],[0,0,-.5],[0,0,-5],[40,0,-5.]])
        r=penetration_residual(q,p,n,1.)
        np.testing.assert_allclose(r[:3],[0,.5,5])
        self.assertLess(r[3],1e-10)
        # Rotation/translation of the whole observation does not change limits.
        rot=np.array([[0,0,1],[1,0,0],[0,1,0]])
        np.testing.assert_allclose(r,penetration_residual(q@rot+7,p@rot+7,n@rot,1.))

    def test_boundary_validation(self):
        for value in ([dict(id='b',x=0,y=0,z=0,normal=[0,0,0])],
                      [dict(id='b',x=float('nan'),y=0,z=0,normal=[0,0,1])], [{}]):
            with self.assertRaises(ValueError):valid_boundaries(value)

    def test_pressure_roles_and_frame_units(self):
        contacts=[dict(id=str(i),x=(i%4)*.01,y=(i//4)*.01,z=0,pressure=900) for i in range(12)]
        contacts += [dict(id='inactive',x=0,y=0,z=0,pressure=0,normal=[0,0,1]),
                     dict(id='unknown',x=0,y=0,z=0,pressure=None,normal=[0,0,1])]
        boundary=dict(id='hand:0',x=.02,y=.01,z=0,normal=[0,0,1])
        frame=dict(valid=True,units='m',frame='left_base_link',points=contacts,boundary_points=[boundary])
        fit=dict(compatible=True,optimizer_success=True,consistent_refinements=2,score=0)
        with patch('digital_twin.sensor_identification.classifier.fit_model',return_value=fit) as fn:
            r=identify_frame(frame,dict(SENSOR_A=object(),SENSOR_B=object()))
        self.assertEqual(r['contacts'],12)
        self.assertEqual(r['hand_boundary_points'],1)
        self.assertEqual(r['inactive_taxel_limits'],1)
        np.testing.assert_allclose(fn.call_args.kwargs['boundary_xyz'],[[20,10,0],[0,0,0],[0,0,0]])
        self.assertEqual(r['surface_taxel_limits'],2)
        self.assertEqual(len(fn.call_args.args[1]),12)
        # Boundaries never count as contacts, even in a manual snapshot.
        r=identify_sensor([],dict(SENSOR_A=object(),SENSOR_B=object()),boundary_points=[boundary])
        self.assertEqual(r['reason'],'insufficient_contacts');self.assertFalse(r['fits'])

if __name__=='__main__':unittest.main()
