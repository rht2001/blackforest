import sys
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from digital_twin.sensor_identification.classifier import IdentificationConfig
from digital_twin.sensor_identification.adapter import identify_frame
from digital_twin.sensor_identification.web_service import IdentificationService

class HandToleranceTests(unittest.TestCase):
    def test_default_and_invalid_allowances(self):
        self.assertEqual(IdentificationConfig().hand_tolerance_mm,3.)
        for value in (0., -1., float('nan'), float('inf')):
            with self.assertRaises(ValueError): IdentificationConfig(hand_tolerance_mm=value)
    def test_frame_forwards_independent_hand_allowance(self):
        with patch('digital_twin.sensor_identification.adapter.identify_sensor',return_value={}) as identify:
            identify_frame(dict(valid=True,frame='left_base_link',units='m',points=[]),{},tolerance_mm=1.,hand_tolerance_mm=4.5,point_spacing_mm=3.5)
        config=identify.call_args.args[2]
        self.assertEqual(config.tolerance_mm,1.)
        self.assertEqual(config.hand_tolerance_mm,4.5)
        self.assertEqual(config.point_spacing_mm,3.5)
    def test_service_request_reaches_frame_fit(self):
        request=dict(mode='current',frame=dict(valid=True,frame='left_base_link',units='mm',points=[]),hand_tolerance_mm=4.5,point_spacing_mm=3.5)
        with tempfile.TemporaryDirectory() as folder:
            service=IdentificationService(folder)
            service._models={}
            with patch('digital_twin.sensor_identification.web_service.threading.Thread') as thread:
                service.start(request)
            worker=thread.call_args.kwargs
            self.assertEqual(worker['args'][1].hand_tolerance_mm,4.5)
            with patch('digital_twin.sensor_identification.web_service.identify_frame',return_value={'fits':{}}) as identify, patch('digital_twin.sensor_identification.web_service.write_viewer',side_effect=lambda path,*args,**kwargs:path.write_text('<html></html>')):
                service._run(*worker['args'])
            self.assertEqual(identify.call_args.kwargs['hand_tolerance_mm'],4.5)
            self.assertEqual(identify.call_args.kwargs['point_spacing_mm'],3.5)
            self.assertEqual(service.snapshot()['status'],'complete')

if __name__=='__main__':unittest.main()
