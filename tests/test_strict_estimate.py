from pathlib import Path
import sys
import unittest
import json
import re
from dataclasses import replace

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import numpy as np
import trimesh
from scipy.spatial.transform import Rotation
from digital_twin.sensor_identification.geometry import CandidateModel
from digital_twin.sensor_identification.limits import triangle_patch_depth, triangle_penetrations
from digital_twin.sensor_identification.matching import fit_model
from digital_twin.sensor_identification.classifier import IdentificationConfig


class StrictEstimateTests(unittest.TestCase):
    def test_display_mesh_origin_matches_source_stl_after_compensation(self):
        root=Path(__file__).resolve().parents[1]
        html=(root/'digital_twin/hand_twin_6dof.html').read_text(encoding='utf-8')
        data=json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>',html).group(1))
        for model in data['sensor_objects']:
            source=trimesh.load_mesh(root/model['source'],process=True)
            local=np.array(model['vertices']).reshape(-1,3)*1000+model['source_center_mm']
            np.testing.assert_allclose([local.min(axis=0),local.max(axis=0)],source.bounds,atol=1e-4)

    def test_crossing_triangle_without_any_vertex_in_patch(self):
        triangle=np.array([[[-20.,-20.,-3.],[20.,-20.,-3.],[0.,20.,-3.]]])
        self.assertAlmostEqual(triangle_patch_depth(triangle),3.)
        self.assertEqual(triangle_patch_depth(triangle+[0,0,4]),0.)
        self.assertEqual(triangle_patch_depth(triangle+[50,0,0]),0.)

    def test_vertical_triangle_crosses_patch_edges(self):
        t=np.array([[[0.,-20.,-5.],[0.,20.,-5.],[0.,0.,2.]]])
        self.assertAlmostEqual(triangle_patch_depth(t),5.)

    def test_enclosing_shape_cannot_hide_behind_nearest_outward_surface(self):
        model=CandidateModel('box',trimesh.creation.box(extents=[10,10,10]),sample_count=1000)
        p=np.array([[0.,0.,4.]])
        self.assertGreater(model.nearest(p)[0][0,2],p[0,2])
        depth=triangle_penetrations(model,np.eye(3),np.zeros(3),p,np.array([[0,0,1.]]))
        np.testing.assert_allclose(depth,[9.])
        r=Rotation.from_euler('xyz',[23,17,41],degrees=True).as_matrix();t=np.array([12.,31.,-8.])
        moved=triangle_penetrations(model,r,t,p@r.T+t,np.array([[0,0,1.]])@r.T)
        np.testing.assert_allclose(moved,depth,atol=1e-7)

    def test_single_bad_red_contact_rejects_low_average_error(self):
        model=CandidateModel('box',trimesh.creation.box(extents=[10,10,10]),sample_count=1000)
        # Corners constrain translation/rotation. One extra red point used to
        # pass the RMS + 90% inlier gate despite exceeding tolerance.
        contacts=np.vstack([np.repeat(model.mesh.vertices,8,axis=0),[[0.,0.,7.8]]])
        config=replace(IdentificationConfig(),tolerance_mm=1.,random_starts=1,
                       refine_starts=3,icp_iterations=12,max_nfev=50)
        fit=fit_model(model,contacts,np.empty((0,3)),np.array([]),config)
        self.assertFalse(fit['compatible'])
        self.assertIn('contact_outside_tolerance',fit['rejection_reasons'])

    def test_contact_tolerance_does_not_relax_hand_penetration(self):
        model=CandidateModel('box',trimesh.creation.box(extents=[10,10,10]),sample_count=1000)
        config=replace(IdentificationConfig(),tolerance_mm=10.,random_starts=1,
                       refine_starts=2,icp_iterations=8,max_nfev=20)
        fit=fit_model(model,model.mesh.vertices,np.empty((0,3)),np.array([]),config,
                      boundary_xyz=np.array([[0.,0.,4.]]),boundary_normals=np.array([[0.,0.,1.]]))
        self.assertLess(fit['max_error_mm'],10.)
        self.assertFalse(fit['compatible'])
        self.assertIn('hand_penetration',fit['rejection_reasons'])

if __name__=='__main__': unittest.main()
