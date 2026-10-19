import sys
import tempfile
import time
import unittest
from pathlib import Path
sys.path.insert(0,str(Path(__file__).resolve().parents[1]))
from digital_twin.sensor_identification.web_service import IdentificationService

class LivePoseTests(unittest.TestCase):
    def test_bounded_history_snapshot_isolation_and_nonpose_progress(self):
        with tempfile.TemporaryDirectory() as folder:
            service=IdentificationService(folder)
            service._state=dict(status='running',started_at=time.time())
            matrix=[[1,0,0,12],[0,1,0,34],[0,0,1,56],[0,0,0,1]]
            for i in range(270):
                service._progress(dict(candidate='SENSOR_A',stage='refining',preview=True,transform=matrix))
            service._progress(dict(stage='checking_limits'))
            state=service.snapshot()
            self.assertEqual(len(state['pose_history']),256)
            self.assertEqual(state['preview_pose']['sequence'],270)
            self.assertEqual(state['progress']['stage'],'checking_limits')
            self.assertEqual(state['pose_history'][0]['sequence'],15)
            state['pose_history'][-1]['transform'][0][3]=999
            self.assertEqual(service.snapshot()['preview_pose']['transform'][0][3],12)
            service._state['status']='complete'
            service._progress(dict(candidate='SENSOR_B',stage='refining',preview=True,transform=matrix))
            self.assertEqual(service.snapshot()['pose_sequence'],270)

if __name__=='__main__':unittest.main()
