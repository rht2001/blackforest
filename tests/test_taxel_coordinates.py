"""Viewer position relay: no second FK implementation and no hardware access."""
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import unittest
from unittest.mock import patch
from urllib.error import HTTPError
from urllib.request import Request, ProxyHandler, build_opener

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'src'))
sys.path.insert(0, str(ROOT/'digital_twin'))
from taxel_coordinates import get_taxel_positions
import bridge_server as bridge


class TaxelRelayTests(unittest.TestCase):
    def setUp(self):
        self.saved = deepcopy(bridge.TAXELS), bridge.TAXELS_RECEIVED_AT
        bridge.TAXELS, bridge.TAXELS_RECEIVED_AT = [], 0
        self.server = bridge.ThreadingHTTPServer(('127.0.0.1', 0), bridge.Handler)
        threading.Thread(target=self.server.serve_forever, daemon=True).start()
        self.url = f'http://127.0.0.1:{self.server.server_port}'
        self.opener = build_opener(ProxyHandler({}))
        self.points = [{'id': identity, 'x': i*.00001, 'y': -.03, 'z': .1, 'pressure': i}
                       for i, identity in enumerate(sorted(bridge.TAXEL_IDS))]

    def tearDown(self):
        self.server.shutdown()
        self.server.server_close()
        bridge.TAXELS, bridge.TAXELS_RECEIVED_AT = self.saved

    def post(self, points):
        request = Request(self.url+'/api/taxels', data=json.dumps(points).encode(),
                          headers={'Content-Type': 'application/json'})
        with self.opener.open(request, timeout=2) as response:
            return json.load(response)

    def test_roundtrip_preserves_exact_viewer_points_without_altering_telemetry(self):
        state = deepcopy(bridge.STATE)
        self.assertEqual(self.post(self.points), {'ok': True})
        self.assertEqual(get_taxel_positions(self.url), self.points)
        self.assertEqual(bridge.STATE, state)
        moved = deepcopy(self.points)
        moved[0]['z'] += .01
        self.post(moved)
        self.assertEqual(get_taxel_positions(self.url), moved)

    def test_pressure_changes_without_motion_and_preserves_zero_and_missing(self):
        self.post(self.points)
        points = deepcopy(self.points)
        points[0]['pressure'] = 3200
        points[1]['pressure'] = 0
        points[2]['pressure'] = None
        self.post(points)
        self.assertEqual(get_taxel_positions(self.url), points)

    def test_absent_or_expired_viewer_returns_empty_list(self):
        self.assertEqual(get_taxel_positions(self.url), [])
        with patch.object(bridge.time, 'time', return_value=100):
            self.post(self.points)
            self.assertEqual(get_taxel_positions(self.url), self.points)
        with patch.object(bridge.time, 'time', return_value=105):
            self.assertEqual(get_taxel_positions(self.url), [])

    def test_invalid_data_cannot_replace_last_valid_cache(self):
        self.post(self.points)
        duplicate = deepcopy(self.points)
        duplicate[0]['id'] = duplicate[1]['id']
        bad_value = deepcopy(self.points)
        bad_value[0]['x'] = True
        nan = deepcopy(self.points)
        nan[0]['x'] = float('nan')
        extra = deepcopy(self.points)
        extra[0]['unknown'] = 123
        for points in ([], self.points[:-1], duplicate, bad_value, nan, extra, {}):
            with self.assertRaises(HTTPError) as error:
                self.post(points)
            self.assertEqual(error.exception.code, 400)
            error.exception.close()
        self.assertEqual(get_taxel_positions(self.url), self.points)

    def test_invalid_pressure_rejected(self):
        self.post(self.points)
        for value in (True, float('nan'), float('inf'), -1, 4096, '100'):
            points = deepcopy(self.points)
            points[0]['pressure'] = value
            with self.assertRaises(HTTPError) as error:
                self.post(points)
            self.assertEqual(error.exception.code, 400)
            error.exception.close()
        self.assertEqual(get_taxel_positions(self.url), self.points)
