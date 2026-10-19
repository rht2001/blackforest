import hashlib
import json
from pathlib import Path
import tempfile
import unittest

from digital_twin.sensor_identification import web_service


class EstimatorRevisionTests(unittest.TestCase):
    def test_cached_result_requires_current_algorithm_revision(self):
        for revision, expected in [('thumb_first_v6', 'idle'),
                                   (web_service.ESTIMATOR_REVISION, 'complete')]:
            with self.subTest(revision=revision), tempfile.TemporaryDirectory() as folder:
                output = Path(folder)
                report = b'<html>cached report</html>'
                (output/'viewer.html').write_bytes(report)
                state = dict(status='complete', result={'fits': {}},
                             workspace=str(Path(web_service.__file__).resolve().parents[2]),
                             estimator_revision=revision,
                             report_sha256=hashlib.sha256(report).hexdigest())
                (output/'job_state.json').write_text(json.dumps(state))
                service = web_service.IdentificationService(output)
                self.assertEqual(service.snapshot()['status'], expected)


if __name__ == '__main__':
    unittest.main()
