from pathlib import Path
import json
import hashlib
import sys
import tempfile
import unittest
from dataclasses import replace

sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'digital_twin'))
import numpy as np
import trimesh
from sensor_identification.classifier import IdentificationConfig,identify_sensor
from sensor_identification.geometry import CandidateModel
from sensor_identification.web_service import IdentificationService,ESTIMATOR_REVISION
import sensor_identification.web_service as service_module


class FeedbackProgressTests(unittest.TestCase):
    def test_real_matching_reports_both_candidates_and_stages(self):
        model=CandidateModel('box',trimesh.creation.box(extents=[10,10,10]),sample_count=1000)
        points=[dict(id=str(i),x=float(v[0]),y=float(v[1]),z=float(v[2]),pressure=900) for i,v in enumerate(model.mesh.vertices)]
        events=[]
        config=replace(IdentificationConfig(),random_starts=1,refine_starts=1,icp_iterations=8,max_nfev=20)
        result=identify_sensor(points,dict(SENSOR_A=model,SENSOR_B=model),config,progress=events.append)
        self.assertEqual({e['candidate'] for e in events},{'SENSOR_A','SENSOR_B'})
        self.assertTrue({'alignment','refining','checking_limits'}<={e['stage'] for e in events})
        self.assertTrue(all(f['compatible'] for f in result['fits'].values()))
        self.assertTrue(all(0<=e['completed']<=e['total'] for e in events if 'total' in e))

    def test_completed_result_is_restored_after_restart(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'viewer.html').write_bytes(b'<html>report</html>')
            state=dict(status='complete',estimator_revision=ESTIMATOR_REVISION,workspace=str(Path(service_module.__file__).resolve().parents[2]),result={'classification':'AMBIGUOUS','fits':{}},job_id='saved',report_sha256=hashlib.sha256(b'<html>report</html>').hexdigest())
            (root/'job_state.json').write_text(json.dumps(state))
            service=IdentificationService(root)
            self.assertEqual(service.snapshot()['job_id'],'saved')
            self.assertEqual(service.report(),b'<html>report</html>')
            (root/'viewer.html').write_text('A different job report')
            self.assertEqual(IdentificationService(root).snapshot()['status'],'idle')

    def test_running_or_malformed_saved_jobs_are_not_revived(self):
        with tempfile.TemporaryDirectory() as folder:
            root=Path(folder);(root/'viewer.html').write_text('report')
            for state in ([],{'status':'running'}, {'status':'complete','workspace':'C:/other-project'}):
                (root/'job_state.json').write_text(json.dumps(state))
                self.assertEqual(IdentificationService(root).snapshot()['status'],'idle')

if __name__=='__main__':unittest.main()
