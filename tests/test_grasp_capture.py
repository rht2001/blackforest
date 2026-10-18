"""Automatic closure ordering and CSV capture; no motor connection is opened."""
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
        self.robot = Mock()
        self.robot.set_angles.return_value = True
        self.angles = [1000] * 6

    def step(self, now, angles=None):
        if angles is not None:
            self.angles = angles
        data = {'angles': np.array(self.angles), 'tactile_mean_matrix': np.zeros((1,17)),
                'tactile_matrix': np.zeros((112,17))}
        with patch('logic.grasp_manager.time.time', return_value=now):
            return self.manager.step(self.robot, data)

    def fingers_closed(self):
        with patch.object(self.manager, 'check_contact_trigger', return_value=True):
            self.step(10)
        self.robot.set_angles.assert_called_once_with([0,0,0,0,1000,1000])
        self.step(10.8, [300]*4 + [1000,1000])
        self.step(10.9)
        self.assertEqual(self.manager.state, 'closing')
        self.step(11.3)

    def test_fingers_then_thumb_then_single_capture_even_when_thumb_contacts_early(self):
        self.fingers_closed()
        self.robot.set_angles.assert_called_with([0,0,0,0,234,670])
        self.assertEqual(self.manager.state, 'thumb_closing')
        self.saved.assert_not_called()
        self.step(12.1, [300]*4 + [450,750])
        self.step(12.2)
        self.step(12.6)
        self.assertEqual(self.manager.state, 'settling')
        self.saved.assert_called_once_with()
        self.step(12.7)
        self.saved.assert_called_once_with()

    def test_unmoved_thumb_times_out_without_capture(self):
        self.fingers_closed()
        self.step(14.4)
        self.assertEqual(self.manager.state, 'resetting')
        self.robot.open_hand.assert_called_once()
        self.saved.assert_not_called()

    def test_thumb_command_failure_resets(self):
        self.robot.set_angles.side_effect = [True, False]
        self.fingers_closed()
        self.assertEqual(self.manager.state, 'resetting')
        self.robot.open_hand.assert_called_once()
        self.saved.assert_not_called()

    def test_failed_initial_command_does_not_close_thumb(self):
        self.robot.set_angles.return_value = False
        with patch.object(self.manager, 'check_contact_trigger', return_value=True):
            self.step(10)
        self.assertEqual(self.manager.state, 'resetting')
        self.robot.set_angles.assert_called_once_with([0,0,0,0,1000,1000])
        self.saved.assert_not_called()

    def test_invalid_telemetry_cannot_complete_thumb_closure(self):
        self.fingers_closed()
        self.step(12.1, [65535]*6)
        self.step(12.6)
        self.assertEqual(self.manager.state, 'thumb_closing')
        self.saved.assert_not_called()

    def test_moving_finger_does_not_trigger_thumb(self):
        self.manager.state = 'closing'
        self.manager.closing_start_time = 10
        for now, value in ((11, 500), (11.2, 480), (11.6, 460)):
            self.step(now, [value]*4 + [1000,1000])
        self.robot.set_angles.assert_not_called()
        self.saved.assert_not_called()

    def test_second_grasp_can_capture_again(self):
        for _ in range(2):
            self.manager.state = 'open'
            self.manager.last_angles = np.array([1000]*6)
            self.angles = [1000]*6
            self.fingers_closed_without_assertions()
        self.assertEqual(self.saved.call_count, 2)

    def fingers_closed_without_assertions(self):
        with patch.object(self.manager, 'check_contact_trigger', return_value=True):
            self.step(10)
        self.step(10.8, [300]*4 + [1000,1000])
        self.step(10.9)
        self.step(11.3)
        self.step(12.1, [300]*4 + [234,670])
        self.step(12.2)
        self.step(12.6)


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
