import hashlib
import importlib.util
import json
from pathlib import Path
import re
import unittest
import numpy as np

ROOT=Path(__file__).resolve().parents[1]/'digital_twin'

class TactileGeometryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.geometry=json.loads((ROOT/'tactile_geometry.json').read_text(encoding='utf-8'))
        html=(ROOT/'hand_twin_6dof.html').read_text(encoding='utf-8')
        cls.data=json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>',html,re.S).group(1))
        spec=importlib.util.spec_from_file_location('geometry_builder',ROOT/'build_tactile_geometry.py')
        cls.builder=importlib.util.module_from_spec(spec)
        spec.loader.exec_module(cls.builder)

    def test_rebuild_is_deterministic_and_matches_embedded_points(self):
        generated=self.builder.build(self.data)
        self.assertEqual(generated,self.geometry)
        self.assertEqual(self.data['tactile_geometry'],self.geometry)
        self.assertEqual(generated['total_cells'],1062)

    def test_centres_intersect_own_sensor_mesh_in_own_local_frame(self):
        assets={v['link']:v for v in self.data['assets']['rh56_tactile_toolkit_left']}
        for region in self.geometry['regions']:
            a=assets[region['link']]
            vertices=np.asarray(a['vertices']).reshape(-1,3)*a['scale']
            triangles=vertices[np.asarray(a['indices']).reshape(-1,3)]
            tf=self.builder.origin(a['origin_xyz_m'],a['origin_rpy_rad'])
            inv=np.linalg.inv(tf)
            low,high=vertices.min(axis=0),vertices.max(axis=0)
            for cell in region['cells']:
                local=(inv@np.r_[cell['point_m'],1])[:3]
                self.assertTrue(np.all(local>=low-1e-8)and np.all(local<=high+1e-8))
                # A point is on a triangle when its barycentric projection has
                # nonnegative weights and a near-zero distance to that plane.
                a0,b,c=triangles[:,0],triangles[:,1],triangles[:,2]
                u,v=b-a0,c-a0
                n=np.cross(u,v)
                length=np.linalg.norm(n,axis=1)
                mask=length>1e-12
                distance=np.full(len(triangles),np.inf)
                distance[mask]=np.abs(np.sum((local-a0[mask])*n[mask],axis=1))/length[mask]
                near=np.where(distance<2e-9)[0]
                self.assertTrue(len(near)>0,region['key'])
                found=False
                for i in near:
                    uv=np.column_stack([u[i],v[i]])
                    weights=np.linalg.lstsq(uv,local-a0[i],rcond=None)[0]
                    if np.min(weights)>=-1e-7 and weights.sum()<=1+1e-7:
                        found=True
                        break
                self.assertTrue(found,(region['key'],cell['wire_index']))

    def test_geometry_digest_covers_points_and_source_metadata(self):
        value={k:v for k,v in self.geometry.items()if k!='geometry_sha256'}
        actual=hashlib.sha256(json.dumps(value,sort_keys=True,separators=(',',':')).encode()).hexdigest()
        self.assertEqual(actual,self.geometry['geometry_sha256'])

if __name__=='__main__':unittest.main()
