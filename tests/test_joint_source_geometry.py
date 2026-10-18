"""Cross-check browser FK against independently parsed pinned URDFs using NumPy."""
from pathlib import Path
import hashlib
import json
import math
import re
import subprocess
import unittest
import xml.etree.ElementTree as ET
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
DT=ROOT/'digital_twin'
REFERENCE=json.loads((DT/'joint_geometry_profiles_for_codex.json').read_text(encoding='utf-8'))
PROFILES={'rh56_tactile_toolkit_left':REFERENCE['primary_profiles']['rh56_tactile_toolkit_left']}

def origin(element):
    xyz=np.array([float(x) for x in element.get('xyz','0 0 0').split()])
    r,p,y=[float(x) for x in element.get('rpy','0 0 0').split()]
    cr,sr,cp,sp,cy,sy=math.cos(r),math.sin(r),math.cos(p),math.sin(p),math.cos(y),math.sin(y)
    out=np.eye(4)
    out[:3,:3]=[[cy*cp,cy*sp*sr-sy*cr,cy*sp*cr+sy*sr],[sy*cp,sy*sp*sr+cy*cr,sy*sp*cr-cy*sr],[-sp,cp*sr,cp*cr]]
    out[:3,3]=xyz
    return out

def rotate(axis,angle):
    axis=axis/np.linalg.norm(axis)
    x,y,z=axis
    skew=np.array([[0,-z,y],[z,0,-x],[-y,x,0]])
    out=np.eye(4)
    out[:3,:3]=np.eye(3)+math.sin(angle)*skew+(1-math.cos(angle))*(skew@skew)
    return out

class JointSourceTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.report=json.loads(subprocess.check_output(['node',str(ROOT/'tests/export_profile_poses.js')],text=True))

    def source(self,pid):
        return ET.parse(next((DT/'profile_sources'/pid).glob('*.urdf'))).getroot()

    def test_pinned_urdf_and_asset_manifest_hashes(self):
        manifest=json.loads((DT/'joint_asset_manifest.json').read_text(encoding='utf-8'))
        for pid,p in PROFILES.items():
            source=next((DT/'profile_sources'/pid).glob('*.urdf')).read_bytes()
            sha=hashlib.sha1(b'blob '+str(len(source)).encode()+b'\0'+source).hexdigest()
            self.assertEqual(sha,REFERENCE['sources'][p['source_ref']]['blob_sha'])
            assets=[a for a in manifest['assets'] if a['profile']==pid]
            self.assertEqual(len(assets),14 if pid.startswith('dex') else 31)
            self.assertTrue(all(a['commit'] and a['blob_sha'] and a['sha256'] for a in assets))

    def test_all_fk_frames_against_independent_source_xml(self):
        for pid,p in PROFILES.items():
            joints=list(self.source(pid).findall('joint'))
            for pose,snapshot in self.report['profiles'][pid].items():
                with self.subTest(profile=pid,pose=pose):
                    q={}
                    for m in p['control_mapping'].values():
                        q[m['driver_joint']]=snapshot['q'][m['driver_joint']]
                    remaining=[j for j in joints if j.find('mimic') is not None]
                    while remaining:
                        ready=next(j for j in remaining if j.find('mimic').get('joint') in q)
                        mimic=ready.find('mimic')
                        q[ready.get('name')]=q[mimic.get('joint')]*float(mimic.get('multiplier','1'))+float(mimic.get('offset','0'))
                        remaining.remove(ready)
                    for name,value in q.items():
                        self.assertAlmostEqual(value,snapshot['q'][name],places=12)
                    frames={p['root_link']:np.eye(4)}
                    remaining=joints.copy()
                    while remaining:
                        joint=next(j for j in remaining if j.find('parent').get('link') in frames)
                        transform=origin(joint.find('origin').attrib if joint.find('origin') is not None else {})
                        if joint.get('type')!='fixed':
                            axis=np.array([float(v) for v in joint.find('axis').get('xyz').split()])
                            transform=transform@rotate(axis,q[joint.get('name')])
                        child=joint.find('child').get('link')
                        frames[child]=frames[joint.find('parent').get('link')]@transform
                        actual=np.array(snapshot['frames'][child]).reshape(4,4)
                        np.testing.assert_allclose(actual,frames[child],atol=1e-12)
                        remaining.remove(joint)

    def test_embedded_reference_matches_canonical_source(self):
        html=(DT/'hand_twin_6dof.html').read_text(encoding='utf-8')
        embedded=json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>',html,re.S).group(1))
        self.assertEqual(embedded['reference'],REFERENCE)
        self.assertEqual(embedded['reference_sha256'],hashlib.sha256((DT/'joint_geometry_profiles_for_codex.json').read_bytes()).hexdigest())

if __name__=='__main__':unittest.main()
