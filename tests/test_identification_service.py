"""Local HTTP integration with controlled workers; CAD fitting has separate tests."""
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import ProxyHandler, Request, build_opener

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'digital_twin'))
import bridge_server as bridge
from sensor_identification.web_service import IdentificationService


class ServiceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        self.service = self.server.sensor_identification = IdentificationService(self.temp.name)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.base = f'http://127.0.0.1:{self.server.server_port}'
        self.opener = build_opener(ProxyHandler({}))

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        self.temp.cleanup()

    def request(self, path, payload=None, origin=None):
        headers = {'Content-Type': 'application/json'}
        if origin:
            headers['Origin'] = origin
        req = Request(self.base + path, None if payload is None else json.dumps(payload).encode(), headers)
        try:
            response = self.opener.open(req, timeout=3)
        except HTTPError as error:
            response = error
        with response:
            return response.status, response.read()

    def wait_for_job(self):
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            state = self.service.snapshot()
            if state['status'] != 'running':
                return state
            time.sleep(.01)
        self.fail('Background job did not finish')

    def test_routes_and_invalid_requests_preserve_existing_state(self):
        self.assertEqual(json.loads(self.request('/api/identification')[1])['status'], 'idle')
        self.assertEqual(self.request('/identification/report')[0], 404)
        self.assertIn(b'Estimate', self.request('/sensor_identification/simulation.js')[1])
        self.assertEqual(self.request('/sensor_identification/measurements.js')[0], 200)
        self.assertEqual(self.request('/sensor_identification/history.js')[0], 200)
        self.assertEqual(self.request('/sensor_identification/runs.js')[0], 200)
        self.assertEqual(self.request('/api/identification', {'mode': 'bogus'})[0], 400)
        self.assertEqual(self.request('/api/identification', {'mode': 'SENSOR_B', 'tolerance_mm': -1})[0], 400)
        self.assertEqual(self.request('/api/identification', {'mode': 'current'})[0], 400)
        self.assertEqual(self.request('/api/identification', {'mode': 'SENSOR_B'}, origin='https://example.com')[0], 403)
        self.assertEqual(self.request('/health')[0], 200)
        self.assertEqual(self.request('/api/state')[0], 200)

    def test_worker_is_nonblocking_single_job_and_serves_report(self):
        started, release = threading.Event(), threading.Event()
        result = {'classification': 'SENSOR_B', 'reason': 'test', 'fits': {}}

        def identify(*args, **kwargs):
            started.set()
            release.wait(3)
            return result

        def viewer(path, *args, **kwargs):
            Path(path).write_text('<html>fitted shapes</html>', encoding='utf-8')

        with patch('sensor_identification.web_service.load_candidates', return_value={'SENSOR_B': object()}), \
                patch('sensor_identification.web_service.synthetic_points', return_value=[]), \
                patch('sensor_identification.web_service.identify_sensor', side_effect=identify), \
                patch('sensor_identification.web_service.write_viewer', side_effect=viewer):
            self.assertEqual(self.request('/api/identification', {'mode': 'SENSOR_B'})[0], 202)
            self.assertTrue(started.wait(1))
            try:
                self.assertEqual(self.request('/api/identification', {'mode': 'SENSOR_B'})[0], 409)
                self.assertEqual(self.request('/api/state')[0], 200)
                self.assertEqual(self.request('/identification/report')[0], 404)
            finally:
                release.set()
            state = self.wait_for_job()
            self.assertEqual(state['status'], 'complete')
            self.assertEqual(state['result'], result)
            self.assertIn(b'fitted shapes', self.request('/identification/report')[1])
            self.assertIn(b'fitted shapes', self.request('/identification/report?job='+state['job_id'])[1])

    def test_accumulated_run_reaches_estimator_and_retains_provenance(self):
        self.service._models = {}
        frame = dict(valid=True, units='m', frame='left_base_link',
                     measurement_source='csv_run', saved_name='run_002',
                     geometry_status='APPROXIMATE_UNCALIBRATED',
                     accumulation={'frames_total': 1883, 'unique_points': 95},
                     points=[dict(id='run:voxel:1', x=.01, y=.02, z=.03, pressure=500)])
        result = dict(classification='AMBIGUOUS', reason='insufficient_contacts', fits={})
        def viewer(path, *args, **kwargs):
            Path(path).write_text('<html>run cloud</html>', encoding='utf-8')
        with patch('sensor_identification.web_service.identify_frame', return_value=result) as identify, \
                patch('sensor_identification.web_service.write_viewer', side_effect=viewer):
            self.assertEqual(self.request('/api/identification', {'mode': 'current', 'frame': frame})[0], 202)
            state = self.wait_for_job()
            self.assertEqual(state['status'], 'complete')
            self.assertEqual(identify.call_args.args[0]['points'], frame['points'])
            self.assertFalse(identify.call_args.kwargs['calibrated'])
            self.assertEqual(state['result']['accumulation'], frame['accumulation'])
            self.assertEqual(state['result']['captured_points'], 1)

    def test_constraint_only_run_keeps_limits_and_produces_report(self):
        self.service._models = {'SENSOR_A': object(), 'SENSOR_B': object()}
        boundary = dict(id='sweep:1', x=.01, y=.02, z=.03, normal=[0,0,1],
                        pressure=0, kind='inactive_taxel')
        frame = dict(valid=True, units='m', frame='left_base_link',
                     measurement_source='csv_run', points=[], boundary_points=[boundary])
        def viewer(path, points, models, result, **kwargs):
            self.assertEqual(points, [])
            self.assertEqual(kwargs['boundary_points'], [boundary])
            Path(path).write_text('<html>swept constraints</html>', encoding='utf-8')
        with patch('sensor_identification.web_service.write_viewer', side_effect=viewer):
            self.assertEqual(self.request('/api/identification', {'mode': 'current', 'frame': frame})[0], 202)
            state = self.wait_for_job()
        self.assertEqual(state['status'], 'complete')
        self.assertEqual(state['result']['reason'], 'space_constraints_only')
        self.assertTrue(state['result']['space_constraints_available'])
        self.assertFalse(state['result']['contact_anchor_available'])
        self.assertEqual(state['result']['accumulated_inactive_taxel_limits'], 1)
        self.assertEqual(state['result']['fits'], {})

    def test_current_frame_cannot_bypass_calibration_and_errors_are_visible(self):
        result = {'classification': 'AMBIGUOUS', 'reason': 'calibration_unverified', 'fits': {}}
        self.service._models = {}
        with patch('sensor_identification.web_service.identify_frame', return_value=result) as identify, \
                patch('sensor_identification.web_service.write_viewer', side_effect=RuntimeError('test failure')):
            frame = {'valid': True, 'units': 'mm', 'frame': 'viewer_cad',
                     'geometry_status': 'CALIBRATED', 'points': []}
            self.assertEqual(self.request('/api/identification', {'mode': 'current', 'frame': frame})[0], 202)
            self.assertEqual(self.wait_for_job()['status'], 'error')
            self.assertEqual(identify.call_args.kwargs['calibrated'], False)
            self.assertIn('test failure', self.service.snapshot()['error'])


if __name__ == '__main__':
    unittest.main()
