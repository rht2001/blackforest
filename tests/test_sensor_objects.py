"""Verify both embedded grasp objects against the actual repository STLs."""
import hashlib
import json
from pathlib import Path
import re
import struct
import sys
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'digital_twin'))
from build_sensor_objects import build


class SensorObjectSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.models = build()

    def test_both_meshes_preserve_source_triangles_dimensions_and_units(self):
        self.assertEqual([m['id'] for m in self.models], ['vegabar_29', 'vegabar_38'])
        for model in self.models:
            source = (ROOT/model['source']).read_bytes()
            self.assertEqual(model['source_sha256'], hashlib.sha256(source).hexdigest())
            faces = struct.unpack_from('<I', source, 80)[0]
            self.assertEqual(len(model['indices']), faces*3)
            self.assertEqual(model['units'], 'metres')
            self.assertEqual(model['source_unit_scale_to_metres'], .001)
            vertices, indices = model['vertices'], model['indices']
            self.assertTrue(all(0 <= i < len(vertices)//3 for i in indices))
            for face in (0, faces//2, faces-1):
                points = list(struct.iter_unpack('<3f', source[84+face*50+12:84+face*50+48]))
                for corner, point in enumerate(points):
                    index = indices[face*3+corner]
                    for k in range(3):
                        self.assertAlmostEqual(vertices[index*3+k], (point[k]-model['source_center_mm'][k])*.001, places=9)
            for k in range(3):
                values = vertices[k::3]
                self.assertAlmostEqual(min(values)+max(values), 0, places=9)
                self.assertAlmostEqual(max(values)-min(values), (model['source_bounds_mm'][1][k]-model['source_bounds_mm'][0][k])*.001, places=9)

    def test_viewer_contains_matching_sensor_geometry(self):
        html = (ROOT/'digital_twin/hand_twin_6dof.html').read_text(encoding='utf-8')
        embedded = json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>', html, re.S).group(1))
        self.assertEqual(embedded['sensor_objects'], self.models)
