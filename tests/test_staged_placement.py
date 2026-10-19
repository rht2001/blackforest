import sys
import unittest
from unittest.mock import patch
from pathlib import Path
from types import SimpleNamespace
import numpy as np
import trimesh

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from digital_twin.sensor_identification.matching import fit_model
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.classifier import identify_sensor, IdentificationConfig

class StagedPlacementTests(unittest.TestCase):
    def test_one_centered_pose_turn_slide_and_full_audit(self):
        model = CandidateModel('box', trimesh.creation.box(extents=[12,20,60]), sample_count=1000)
        palm = np.array([[x,0.,z] for x in [-20.,0.,20.] for z in [75.,115.,155.]])
        contacts = model.mesh.vertices + [0.,11.,115.]
        events, calls = [], []
        def freeze(fun, x0, **kwargs):
            calls.append(len(x0))
            fun(x0)
            return SimpleNamespace(x=x0, success=True, nfev=1)
        config = IdentificationConfig(quick_preview=True, tolerance_mm=100., point_spacing_mm=0.)
        with patch('digital_twin.sensor_identification.matching.least_squares', side_effect=freeze), \
             patch('digital_twin.sensor_identification.matching.triangle_penetrations', return_value=np.array([4.])) as audit:
            fit = fit_model(model, contacts, np.empty((0,3)), np.empty(0), config,
                boundary_xyz=palm, boundary_normals=np.tile([0.,1.,0.], (len(palm),1)),
                palm_xyz=palm, palm_normals=np.tile([0.,1.,0.], (len(palm),1)),
                thumb_xyz=contacts, progress=events.append)
        self.assertEqual(calls, [3,3,6])
        self.assertEqual(fit['starts_evaluated'], 1)
        self.assertFalse(fit['compatible'])
        self.assertIn('hand_penetration', fit['rejection_reasons'])
        self.assertEqual(audit.call_count, 1)
        self.assertEqual(list(dict.fromkeys(e['stage'] for e in events))[:3], ['standard_placement','adjusting_angle','moving_to_thumb'])
        previews = [e for e in events if e.get('preview')]
        self.assertTrue(previews)
        self.assertTrue(all(np.array(e['transform']).shape == (4,4) for e in previews))
        self.assertIn('checking_limits', [e['stage'] for e in previews])
        transform = np.array(fit['initial_transform'])
        placed_center = transform[:3,:3] @ model.mesh.bounds.mean(axis=0) + transform[:3,3]
        np.testing.assert_allclose(placed_center[[0,2]], [0.,115.], atol=1e-6)

    def test_thumb_subset_forwarded_in_mm(self):
        model = CandidateModel('box', trimesh.creation.box(extents=[12,20,60]), sample_count=1000)
        points = [dict(id=i,x=float(x)/1000,y=float(y)/1000,z=float(z)/1000,
                       region='thumb_pad' if i<4 else 'index_tip',pressure=700)
                  for i,(x,y,z) in enumerate(model.mesh.vertices)]
        with patch('digital_twin.sensor_identification.classifier.fit_model', return_value=dict(compatible=True,score=0.)) as fit:
            identify_sensor(points, dict(SENSOR_A=model,SENSOR_B=model),
                            IdentificationConfig(quick_preview=True,coordinate_units='m'))
        np.testing.assert_allclose(fit.call_args.kwargs['thumb_xyz'], model.mesh.vertices[:4])

if __name__ == '__main__': unittest.main(verbosity=2)
