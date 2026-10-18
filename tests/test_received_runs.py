"""Passive HTTP capture and reconstruction contract; never talks to hardware."""
import csv
import io
import json
from pathlib import Path
import sys
import tempfile
import threading
import time
import unittest
from urllib.request import Request, ProxyHandler, build_opener

ROOT = Path(__file__).resolve().parents[1]
# Also executable from the maintenance staging folder.
if not (ROOT / 'digital_twin').exists():
    ROOT = Path('C:/Users/karl/Desktop/personal_branch_hand/blackforest')
sys.path.insert(0, str(ROOT / 'digital_twin'))
import bridge_server as bridge
from received_runs import ReceivedRuns


class ReceivedRunTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.capture = ReceivedRuns(self.temp.name)

    def tearDown(self):
        if self.capture.stream:
            self.capture._finish('interrupted', 'test_teardown', time.time())
        self.temp.cleanup()

    def packet(self, state='thumb_probing', index=1, **extra):
        return dict(state=state, source_id='sender1', sample_index=index,
                    angles_act=[900, 900, 900, 900, 400, 500],
                    tactile_regions={'thumb_tip': [[700, 600, 0], [0, 0, 0], [0, 0, 0]]}, **extra)

    def test_complete_capture_and_recovery(self):
        self.capture.add(self.packet(index=1))
        key = self.capture.active['id']
        self.capture.add(self.packet(index=3, slip_detected=True))
        self.capture.add(self.packet('cylinder_confirmed', 4))
        meta = self.capture.snapshot()['runs'][0]
        self.assertEqual((meta['status'], meta['samples'], meta['missing_source_samples']), ('complete', 2, 1))
        rows = list(csv.DictReader(io.StringIO(self.capture.read(key).decode()), delimiter=';'))
        self.assertEqual([r['index'] for r in rows], ['0', '1'])
        matrix = json.loads(rows[0]['tactile_matrix'])
        self.assertEqual(matrix[0][12], 700)
        self.assertIsNone(matrix[0][0], 'Absent regions remain unknown')
        self.assertEqual(rows[1]['slip_detected'], 'True')
        self.assertEqual(ReceivedRuns(self.temp.name).snapshot()['runs'][0]['id'], key)

    def test_no_cached_pose_and_no_duplicate(self):
        self.capture.add(self.packet())
        self.capture.add(self.packet())
        self.capture.add({'state': 'thumb_probing', 'source_id': 'sender1', 'sample_index': 2,
                          'tactile_regions': {'thumb_tip': [[0]*3]*3}})
        self.capture.add({'state': 'probing_complete', 'source_id': 'sender1', 'sample_index': 3})
        meta = self.capture.snapshot()['runs'][0]
        self.assertEqual((meta['samples'], meta['skipped_samples']), (1, 2))

    def test_abort_timeout_and_source_ownership(self):
        self.capture.add(self.packet())
        self.capture.add({**self.packet('resetting', 2), 'source_id': 'foreign'})
        self.assertIsNotNone(self.capture.active)
        self.capture.add(self.packet('resetting', 2))
        self.assertEqual(self.capture.snapshot()['runs'][0]['status'], 'interrupted')
        self.capture.add(self.packet(index=4))
        self.capture.expire(time.time()+16)
        self.assertTrue(any(r['reason'] == 'telemetry_timeout' for r in self.capture.snapshot()['runs']))

    def test_restart_marks_unfinished_run(self):
        self.capture.add(self.packet())
        self.capture.stream.close()
        recovered = ReceivedRuns(self.temp.name)
        self.assertEqual(recovered.snapshot()['runs'][0]['reason'], 'receiver_restarted')
        self.assertEqual(recovered.snapshot()['runs'][0]['samples'], 1)
        self.capture.stream = self.capture.active = None

    def test_manual_capture_without_remote_software_states(self):
        self.capture.start_manual()
        self.capture.expire(time.time()+30)
        self.assertIsNotNone(self.capture.active, 'Manual capture waits for first sensor sample')
        self.capture.add({k: v for k, v in self.packet().items() if k != 'state'})
        self.capture.add(self.packet('open', 2))
        self.assertIsNotNone(self.capture.active, 'Local open state must not stop another computer capture')
        self.capture.finish_manual()
        meta = self.capture.snapshot()['runs'][0]
        self.assertEqual((meta['status'], meta['samples'], meta['capture_mode']), ('complete', 2, 'manual'))
        rows = list(csv.DictReader(io.StringIO(self.capture.read(meta['id']).decode()), delimiter=';'))
        self.assertEqual([r['reported_state'] for r in rows], ['', 'open'])
        self.assertTrue(all(r['state'] == 'thumb_probing' for r in rows))

    def test_duplicate_completion_still_closes_run(self):
        self.capture.add(self.packet())
        self.capture.add(self.packet('probing_complete'))
        self.assertIsNone(self.capture.active)
        self.assertEqual(self.capture.snapshot()['runs'][0]['status'], 'complete')

    def test_finger_closure_pre_roll_and_reopening(self):
        def pose(index, primary):
            return {**self.packet(index=index), 'state': 'open', 'angles_act': primary+[450, 500]}
        self.capture.add(pose(1, [500]*4))
        self.assertIsNone(self.capture.active, 'Joining a closed hand must not invent a start')
        self.capture.add(pose(2, [950]*4))
        self.assertTrue(self.capture.armed)
        self.capture.add(pose(3, [750, 900, 900, 900]))
        self.assertIsNone(self.capture.active, 'One bent finger is insufficient')
        self.capture.add(pose(4, [750, 750, 900, 900]))
        self.assertEqual(self.capture.active['capture_mode'], 'closure')
        self.assertEqual(self.capture.active['samples'], 3, 'Pre-roll starts at the observed open hand, excluding an earlier grasp')
        self.capture.add(pose(5, [950]*4))
        self.capture.add(pose(6, [950]*4))
        self.assertIsNotNone(self.capture.active)
        self.capture.add(pose(7, [950]*4))
        meta = self.capture.snapshot()['runs'][0]
        self.assertEqual((meta['status'], meta['reason'], meta['samples']), ('complete', 'hand_reopened', 6))
        self.assertEqual(meta['missing_source_samples'], 0)
        self.capture.add(pose(8, [750]*4))
        self.assertEqual(self.capture.active['capture_mode'], 'closure', 'A second run can start')

    def test_motion_disabling_source_switch_and_stale_arm(self):
        opened = {**self.packet(), 'state': 'open', 'angles_act': [950]*6}
        closed = {**self.packet(index=2), 'state': 'open', 'angles_act': [500]*6}
        self.capture.add(opened)
        self.capture.add({**closed, 'source_id': 'other'})
        self.assertIsNone(self.capture.active)

        self.capture.add(opened)
        self.capture.add(closed, time.time()+16)
        self.assertIsNone(self.capture.active)
        self.capture.set_auto(False)
        self.capture.add(opened)
        self.capture.add(closed)
        self.assertIsNone(self.capture.active)

    def test_capture_order_uses_start_time_within_same_second(self):
        for index in range(5):
            stamp = 1000+index/10
            self.capture.add(self.packet(index=index*2), stamp)
            self.capture.add(self.packet('cylinder_confirmed', index*2+1), stamp+.01)
        self.assertEqual([r['started_at'] for r in self.capture.snapshot()['runs']], [1000.4, 1000.3, 1000.2, 1000.1, 1000])

    def test_path_validation_and_metadata_validation(self):
        for key in ('../../main_dataset', '/etc/passwd', 'x'*29):
            with self.assertRaises(ValueError):
                self.capture.read(key)
        for packet in ({'sample_index': -1}, {'state': []}, {'slip_detected': 'False'}):
            with self.assertRaises(ValueError):
                bridge.normalize(packet)

    def test_http_capture_every_received_sample(self):
        server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        server.capture_directory = self.temp.name
        worker = threading.Thread(target=server.serve_forever, daemon=True)
        worker.start()
        base = f'http://127.0.0.1:{server.server_port}'
        opener = build_opener(ProxyHandler({}))
        try:
            for index in range(25):
                data = json.dumps(self.packet(index=index)).encode()
                with opener.open(Request(base+'/api/state', data, {'Content-Type': 'application/json'})) as reply:
                    self.assertTrue(json.load(reply)['ok'])
            with opener.open(Request(base+'/api/state', json.dumps(self.packet('cylinder_confirmed', 25)).encode(),
                                     {'Content-Type': 'application/json'})) as reply:
                self.assertEqual(reply.status, 200)
            with opener.open(base+'/api/probing-runs') as reply:
                meta = json.load(reply)['runs'][0]
            self.assertEqual(meta['samples'], 25)
            with opener.open(base+'/api/probing-runs/'+meta['id']) as reply:
                self.assertEqual(len(list(csv.DictReader(io.StringIO(reply.read().decode()), delimiter=';'))), 25)
            for action in ('start', 'finish'):
                with opener.open(Request(base+'/api/probing-runs', json.dumps({'action': action}).encode(),
                                         {'Content-Type': 'application/json'})) as reply:
                    self.assertEqual(reply.status, 200)
        finally:
            server.shutdown()
            server.server_close()
            worker.join()


if __name__ == '__main__':
    unittest.main()
