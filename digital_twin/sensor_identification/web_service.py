"""One background identification job for the existing local simulation bridge."""
import copy
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

from .adapter import identify_frame, normalize_frame
from .classifier import IdentificationConfig, identify_sensor
from .demo import synthetic_points, write_viewer
from .geometry import load_candidates

ESTIMATOR_REVISION = 'empirical_grasp_pose_v11'


class IdentificationService:
    def __init__(self, output=None):
        self.output = Path(output) if output else Path(__file__).resolve().parents[2] / 'sensor_identification_demo' / 'simulation'
        self._lock = threading.Lock()
        self._state = {'status': 'idle', 'estimator_revision': ESTIMATOR_REVISION}
        self._report = None
        self._models = None
        self._restore_completed_job()

    def _restore_completed_job(self):
        try:
            state = json.loads((self.output / 'job_state.json').read_text(encoding='utf-8'))
            if not isinstance(state, dict):
                return
            expected = str(Path(__file__).resolve().parents[2])
            if (state.get('status') == 'complete' and isinstance(state.get('result'), dict)
                    and state.get('workspace') == expected
                    and state.get('estimator_revision') == ESTIMATOR_REVISION):
                report = (self.output / 'viewer.html').read_bytes()
                if state.get('report_sha256') != hashlib.sha256(report).hexdigest():
                    return
                state.pop('workspace', None)
                self._state = state
                self._report = report
        except (OSError, ValueError, TypeError):
            pass

    def _progress(self, event):
        with self._lock:
            if self._state.get('status') == 'running':
                self._state['progress'] = {**event, 'updated_at': time.time()}
                if event.get('preview') is True and event.get('candidate') in ('SENSOR_A', 'SENSOR_B'):
                    sequence = self._state.get('pose_sequence', 0)+1
                    self._state['pose_sequence'] = sequence
                    pose = dict(candidate=event['candidate'], stage=event['stage'],
                                transform=copy.deepcopy(event['transform']), sequence=sequence,
                                elapsed_seconds=time.time()-self._state['started_at'])
                    self._state['preview_pose'] = pose
                    history = self._state.setdefault('pose_history', [])
                    history.append(pose)
                    del history[:-256]
                if 'candidate_decisions' in event:
                    self._state['candidate_decisions'] = copy.deepcopy(event['candidate_decisions'])
                if event.get('first_guess'):
                    self._state['first_guess'] = copy.deepcopy(event['first_guess'])

    def snapshot(self):
        with self._lock:
            return {**copy.deepcopy(self._state), 'workspace': str(Path(__file__).resolve().parents[2])}

    def report(self):
        with self._lock:
            return self._report

    def start(self, request):
        if not isinstance(request, dict):
            raise ValueError('Expected a JSON object')
        mode = request.get('mode')
        if mode not in ('SENSOR_A', 'SENSOR_B', 'shared', 'current'):
            raise ValueError('Choose SENSOR_A, SENSOR_B, shared, or current')
        config = IdentificationConfig(pressure_threshold=request.get('threshold', 200.),
                                      tolerance_mm=request.get('tolerance_mm', 1.5),
                                      hand_tolerance_mm=request.get('hand_tolerance_mm', 3.),
                                      point_spacing_mm=request.get('point_spacing_mm', 2.),
                                      quick_preview=request.get('quick_preview', False))
        if mode == 'current' and not isinstance(request.get('frame'), dict):
            raise ValueError('A current measurement frame is required')
        with self._lock:
            if self._state['status'] == 'running':
                return None
            self._state = dict(status='running', job_id=uuid.uuid4().hex, mode=mode,
                               estimator_revision=ESTIMATOR_REVISION, quick_preview=config.quick_preview,
                               progress=dict(stage='queued', updated_at=time.time()),
                               started_at=time.time(), input_label=str(request.get('input_label', mode))[:256])
            self._report = None
            state = copy.deepcopy(self._state)
        worker = threading.Thread(target=self._run, args=(copy.deepcopy(request), config),
                                  name='SensorIdentification', daemon=True)
        worker.start()
        return state

    def _run(self, request, config):
        try:
            self._progress(dict(stage='loading_models'))
            if self._models is None:
                self._models = load_candidates()
            self._progress(dict(stage='preparing'))
            mode = request['mode']
            units = 'mm'
            boundary_points = []
            if mode == 'current':
                frame = normalize_frame(request['frame'])
                # This UI has no verified physical calibration. Never accept a
                # caller's CALIBRATED label as permission to bypass that gate.
                result = identify_frame(frame, self._models,
                                        tolerance_mm=config.tolerance_mm,
                                        hand_tolerance_mm=config.hand_tolerance_mm, point_spacing_mm=config.point_spacing_mm,
                                        pressure_threshold=config.pressure_threshold,
                                        calibrated=False, quick_preview=config.quick_preview, progress=self._progress)
                points = frame.get('points', []) if frame.get('valid') is True else []
                units = frame.get('units', 'mm')
                boundary_points = frame.get('boundary_points', []) if frame.get('valid') is True else []
                result['measurement_source'] = frame.get('measurement_source', 'snapshot')
                result['captured_points'] = len(points)
                if frame.get('measurement_source') == 'csv_run':
                    result['accumulation'] = frame.get('accumulation', {})
                    result['saved_name'] = frame.get('saved_name')
            else:
                model = self._models['SENSOR_A' if mode == 'shared' else mode]
                points = synthetic_points(model, region='shared' if mode == 'shared' else 'full')
                result = identify_sensor(points, self._models, IdentificationConfig(
                    calibrated=True, tolerance_mm=config.tolerance_mm, hand_tolerance_mm=config.hand_tolerance_mm, point_spacing_mm=config.point_spacing_mm, quick_preview=config.quick_preview,
                    pressure_threshold=config.pressure_threshold), progress=self._progress)
            self._progress(dict(stage='writing_report'))
            self.output.mkdir(parents=True, exist_ok=True)
            viewer = self.output / 'viewer.html'
            write_viewer(viewer, points, self._models, result,
                         threshold=config.pressure_threshold, coordinate_units=units, boundary_points=boundary_points)
            (self.output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
            report = viewer.read_bytes()
            with self._lock:
                self._report = report
                self._state.update(status='complete', result=result, finished_at=time.time(),
                                   report_url='/identification/report',
                                   report_sha256=hashlib.sha256(report).hexdigest())
            try:
                temporary = self.output / 'job_state.json.tmp'
                temporary.write_text(json.dumps(self.snapshot(), allow_nan=False), encoding='utf-8')
                temporary.replace(self.output / 'job_state.json')
            except OSError:
                pass
        except Exception as error:
            with self._lock:
                self._state.update(status='error', error=str(error), finished_at=time.time())
