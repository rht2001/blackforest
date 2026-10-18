import sys
from pathlib import Path
from dataclasses import replace
from unittest.mock import patch
import unittest
import numpy as np
import trimesh
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from digital_twin.sensor_identification.classifier import IdentificationConfig, identify_sensor
from digital_twin.sensor_identification.geometry import CandidateModel

class LargerFirstTests(unittest.TestCase):
    def setUp(self):
        self.small = CandidateModel('small', trimesh.creation.box(extents=[10, 10, 10]), sample_count=100)
        self.large = CandidateModel('large', trimesh.creation.box(extents=[20, 20, 20]), sample_count=100)
        self.models = dict(SENSOR_A=self.small, SENSOR_B=self.large)
        self.points = [dict(id=i, x=float(x), y=float(y), z=float(z), pressure=900.)
                       for i, (x,y,z) in enumerate(self.large.mesh.vertices)]
        self.config = IdentificationConfig(quick_preview=True)
    def fit(self, compatible=True, score=1.):
        return dict(compatible=compatible, score=score, optimizer_success=True,
                    consistent_refinements=2, rejection_reasons=[] if compatible else ['hand_penetration'])
    def test_large_first_and_stop_when_acceptable(self):
        events = []
        with patch('digital_twin.sensor_identification.classifier.fit_model', return_value=self.fit()) as mock:
            result = identify_sensor(self.points, self.models, self.config, progress=events.append)
        self.assertIs(mock.call_args.args[0], self.large)
        self.assertEqual(mock.call_count, 1)
        self.assertEqual(result['placement_candidate'], 'SENSOR_B')
        self.assertEqual(result['best_candidate'], 'SENSOR_B')
        self.assertEqual(result['classification'], 'AMBIGUOUS')
        self.assertEqual(events[0]['candidate'], 'SENSOR_B')
    def test_collision_falls_back_to_small(self):
        with patch('digital_twin.sensor_identification.classifier.fit_model', side_effect=[self.fit(False), self.fit()]) as mock:
            result = identify_sensor(self.points, self.models, self.config)
        self.assertEqual([c.args[0].name for c in mock.call_args_list], ['large', 'small'])
        self.assertEqual(result['placement_candidate'], 'SENSOR_A')
        self.assertEqual(result['placement_reason'], 'smaller_candidate_after_larger_rejected')
        self.assertEqual(result['larger_rejection_reasons'], ['hand_penetration'])
    def test_detailed_prefers_larger_despite_lower_small_score_and_retains_ambiguity(self):
        with patch('digital_twin.sensor_identification.classifier.fit_model', side_effect=[self.fit(score=2.), self.fit(score=.1)]) as mock:
            result = identify_sensor(self.points, self.models, replace(self.config, quick_preview=False))
        self.assertEqual(mock.call_count, 2)
        self.assertEqual(result['placement_candidate'], 'SENSOR_B')
        self.assertEqual(result['reason'], 'both_candidates_compatible')
        self.assertEqual(result['classification'], 'AMBIGUOUS')
    def test_neither_fit_is_not_accepted(self):
        with patch('digital_twin.sensor_identification.classifier.fit_model', side_effect=[self.fit(False), self.fit(False)]):
            result = identify_sensor(self.points, self.models, self.config)
        self.assertNotIn('placement_candidate', result)
        self.assertEqual(result['placement_reason'], 'neither_candidate_fits')
    def test_size_comes_from_geometry_not_label(self):
        with patch('digital_twin.sensor_identification.classifier.fit_model', return_value=self.fit()) as mock:
            result = identify_sensor(self.points, dict(SENSOR_A=self.large, SENSOR_B=self.small), self.config)
        self.assertEqual(result['placement_candidate'], 'SENSOR_A')
        self.assertIs(mock.call_args.args[0], self.large)

if __name__ == '__main__': unittest.main()
