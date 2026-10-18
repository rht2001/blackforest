"""One background identification job for the existing local simulation bridge."""
import copy
import json
from pathlib import Path
import threading
import time
import uuid

from .adapter import identify_frame, normalize_frame
from .classifier import IdentificationConfig, identify_sensor
from .demo import synthetic_points, write_viewer
from .geometry import load_candidates


class IdentificationService:
    def __init__(self, output=None):
        self.output = Path(output) if output else Path(__file__).resolve().parents[2] / 'sensor_identification_demo' / 'simulation'
        self._lock = threading.Lock()
        self._state = {'status': 'idle'}
        self._report = None
        self._models = None

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
                                      tolerance_mm=request.get('tolerance_mm', 1.5))
        if mode == 'current' and not isinstance(request.get('frame'), dict):
            raise ValueError('A current measurement frame is required')
        with self._lock:
            if self._state['status'] == 'running':
                return None
            self._state = dict(status='running', job_id=uuid.uuid4().hex, mode=mode,
                               started_at=time.time(), input_label=str(request.get('input_label', mode))[:256])
            self._report = None
            state = copy.deepcopy(self._state)
        worker = threading.Thread(target=self._run, args=(copy.deepcopy(request), config),
                                  name='SensorIdentification', daemon=True)
        worker.start()
        return state

    def _run(self, request, config):
        try:
            if self._models is None:
                self._models = load_candidates()
            mode = request['mode']
            units = 'mm'
            boundary_points = []
            if mode == 'current':
                frame = normalize_frame(request['frame'])
                # This UI has no verified physical calibration. Never accept a
                # caller's CALIBRATED label as permission to bypass that gate.
                result = identify_frame(frame, self._models,
                                        tolerance_mm=config.tolerance_mm,
                                        pressure_threshold=config.pressure_threshold,
                                        calibrated=False)
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
                    calibrated=True, tolerance_mm=config.tolerance_mm,
                    pressure_threshold=config.pressure_threshold))
            self.output.mkdir(parents=True, exist_ok=True)
            viewer = self.output / 'viewer.html'
            write_viewer(viewer, points, self._models, result,
                         threshold=config.pressure_threshold, coordinate_units=units, boundary_points=boundary_points)
            (self.output / 'result.json').write_text(json.dumps(result, indent=2, allow_nan=False), encoding='utf-8')
            report = viewer.read_bytes()
            with self._lock:
                self._report = report
                self._state.update(status='complete', result=result, finished_at=time.time(),
                                   report_url='/identification/report')
        except Exception as error:
            with self._lock:
                self._state.update(status='error', error=str(error), finished_at=time.time())
