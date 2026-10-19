"""Observed external closure and CSV capture; no motor connection is opened."""
import csv
from copy import deepcopy
from pathlib import Path
import sys
import tempfile
import time
import unittest
from unittest.mock import Mock, patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'src'))
sys.path.insert(0, str(ROOT / 'digital_twin'))
from logic.grasp_manager import GraspManager
from logic.grasp_capture import GraspCapture
import test_taxel_coordinates as relay_tests


class ClosureTests(unittest.TestCase):
    def setUp(self):
        self.saved = Mock()
        self.manager = GraspManager(on_grasp_closed=self.saved)
        # No actuation or hardware methods may be called during observation.
        self.robot = Mock(spec=[])
        self.angles = [1000] * 6

    def step(self, now, angles=None, contact=True):
        if angles is not None:
            self.angles = angles
        data = {'angles': np.array(self.angles), 'tactile_mean_matrix': np.zeros((1,17)),
                'tactile_matrix': np.full((112,17), 100 if contact else 0)}
        with patch('logic.grasp_manager.time.time', return_value=now):
            result = self.manager.step(self.robot, data)
        self.assertEqual(self.robot.mock_calls, [])
        return result

    def closed(self, start=10):
        self.step(start, [300]*4 + [450,750])
        self.step(start + .1)
        self.step(start + .9)

    def test_external_closure_captures_once_including_late_attachment(self):
        self.closed()
        self.assertEqual(self.manager.state, 'settling')
        self.saved.assert_called_once_with()
        self.step(11)
        self.saved.assert_called_once_with()

    def test_moving_thumb_prevents_capture(self):
        self.step(10, [300]*4 + [1000,1000])
        self.step(10.8, [300]*4 + [700,900])
        self.step(11.6, [300]*4 + [450,750])
        self.saved.assert_not_called()
        self.step(11.7)
        self.step(12.5)
        self.saved.assert_called_once_with()

    def test_open_hand_or_missing_contact_does_not_capture(self):
        for now in (10, 11, 12):
            self.step(now)
        self.saved.assert_not_called()
        for now in (13, 14, 15):
            self.step(now, [300]*6, contact=False)
        self.saved.assert_not_called()

    def test_invalid_telemetry_restarts_stability_window(self):
        self.step(10, [300]*6)
        self.step(10.1)
        self.step(11, [65535]*6)
        self.step(12, [300]*6)
        self.step(12.1)
        self.saved.assert_not_called()
        self.step(12.9)
        self.saved.assert_called_once_with()

    def test_timeout_waits_for_external_reopening_and_second_capture(self):
        self.closed()
        with patch('logic.grasp_manager.confirm_grip', return_value=(False, 'empty', {})):
            self.step(14)
        self.assertEqual(self.manager.state, 'resetting')
        self.step(30)
        self.assertEqual(self.manager.state, 'resetting')
        self.step(31, [1000]*6)
        self.assertEqual(self.manager.state, 'open')
        self.closed(33)
        self.assertEqual(self.saved.call_count, 2)

    def test_confirmation_needs_continuously_valid_stable_measurements(self):
        self.closed()
        with patch('logic.grasp_manager.confirm_grip', return_value=(True, 'ok', {})):
            self.step(11)
            self.step(11.5)
        with patch('logic.grasp_manager.confirm_grip', return_value=(False, 'lost', {})):
            self.step(11.6)
        with patch('logic.grasp_manager.confirm_grip', return_value=(True, 'ok', {})):
            self.step(11.7)
            self.assertEqual(self.manager.state, 'settling')
            self.step(12.8)
        self.assertEqual(self.manager.state, 'cylinder_confirmed')


class CaptureTests(relay_tests.TaxelRelayTests):
    def test_csv_requires_post_closure_update_and_preserves_values(self):
        from threading import Timer
        with tempfile.TemporaryDirectory() as directory:
            capture = GraspCapture(directory, self.url, wait_timeout=1)
            try:
                self.post(self.points)
                after = time.time()
                self.assertEqual(__import__('taxel_coordinates').get_taxel_positions(self.url, after=after), [])
                fresh = deepcopy(self.points)
                fresh[0]['pressure'] = None
                fresh[1]['pressure'] = 0
                fresh[2]['pressure'] = 1234.5
                fresh[0]['z'] += .015
                timer = Timer(.1, lambda: self.post(fresh))
                timer.start()
                path = capture.save_snapshot(after)
                timer.join()
                with path.open(newline='', encoding='utf-8') as stream:
                    reader = csv.DictReader(stream)
                    rows = list(reader)
                    self.assertEqual(reader.fieldnames, ['id','x','y','z','pressure'])
                self.assertEqual(len(rows), 1062)
                self.assertEqual(rows[0]['pressure'], '')
                self.assertEqual(rows[1]['pressure'], '0')
                self.assertEqual(rows[2]['pressure'], '1234.5')
                self.assertEqual(float(rows[0]['z']), fresh[0]['z'])
            finally:
                capture.close()

    def test_missing_viewer_creates_no_csv_and_worker_survives(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = GraspCapture(directory, self.url, wait_timeout=.15)
            try:
                capture.submit()
                capture._queue.join()
                self.assertEqual(list(Path(directory).iterdir()), [])
                self.assertTrue(capture._worker.is_alive())
            finally:
                capture.close()
            self.assertFalse(capture._worker.is_alive())

    def test_old_bridge_404_explains_restart(self):
        from urllib.error import HTTPError
        with tempfile.TemporaryDirectory() as directory:
            capture = GraspCapture(directory, self.url, wait_timeout=.15)
            try:
                with patch('logic.grasp_capture.get_taxel_positions', side_effect=HTTPError(self.url,404,'Not Found',None,None)):
                    with self.assertRaisesRegex(RuntimeError, 'restart the updated'):
                        capture.save_snapshot(time.time())
                self.assertEqual(list(Path(directory).iterdir()), [])
            finally:
                capture.close()

    def test_disk_error_is_reported_without_killing_worker(self):
        with tempfile.TemporaryDirectory() as directory:
            capture = GraspCapture(directory, self.url, wait_timeout=.15)
            try:
                with patch.object(capture, 'save_snapshot', side_effect=PermissionError('read only')):
                    capture.submit()
                    capture._queue.join()
                self.assertTrue(capture._worker.is_alive())
                self.assertEqual(list(Path(directory).iterdir()), [])
            finally:
                capture.close()


if __name__ == '__main__':
    unittest.main()
