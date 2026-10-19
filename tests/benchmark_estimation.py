"""Offline baseline comparison; no server, telemetry reads, or motor commands.

python tests/benchmark_estimation.py --baseline-ref COMMIT --frame cloud.json --output comparison.json
With no --frame, use deterministic synthetic CAD observations.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys
import time
import types

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))
from digital_twin.sensor_identification import classifier, geometry, matching
from digital_twin.sensor_identification.adapter import normalize_frame
from digital_twin.sensor_identification.classifier import IdentificationConfig, valid_measurements
from digital_twin.sensor_identification.demo import synthetic_points
import numpy as np


def baseline_module(ref, filename):
    name = 'digital_twin.sensor_identification._benchmark_'+filename
    source = subprocess.check_output(['git', 'show', f'{ref}:digital_twin/sensor_identification/{filename}.py'], cwd=ROOT)
    module = types.ModuleType(name)
    module.__package__ = 'digital_twin.sensor_identification'
    module.__file__ = str(ROOT/'digital_twin/sensor_identification'/f'{filename}.py')
    sys.modules[name] = module
    exec(compile(source, module.__file__, 'exec'), module.__dict__)
    return module


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--baseline-ref', required=True)
    parser.add_argument('--frame', action='append', type=Path, default=[])
    parser.add_argument('--output', type=Path)
    parser.add_argument('--cad-root', type=Path, default=ROOT)
    args = parser.parse_args()
    revision = subprocess.check_output(['git', 'rev-parse', args.baseline_ref], cwd=ROOT, text=True).strip()
    old_matching = baseline_module(revision, 'matching')
    old_geometry = baseline_module(revision, 'geometry')
    models = geometry.load_candidates(args.cad_root)
    old_models = old_geometry.load_candidates(args.cad_root)
    # Compare warmed model caches, excluding one-time CAD loading/index build.
    for model in [*models.values(), *old_models.values()]:
        model.mesh.triangles_tree
    cases = [(path.name, normalize_frame(json.loads(path.read_text(encoding='utf-8-sig')))) for path in args.frame]
    if not cases:
        cases = [(name, dict(units='mm', points=synthetic_points(model))) for name, model in models.items()]
    report = dict(baseline_revision=revision, results=[],
                  limitations='Single-run timings; fit residuals are not ground-truth pose accuracy. Rejected conflict previews check only witness hand patches: their hand maximum is a lower bound, not a complete hand audit.')
    original_fit = classifier.fit_model
    try:
        for name, frame in cases:
            config = IdentificationConfig(coordinate_units=frame['units'], tolerance_mm=3.)
            clean, _ = valid_measurements(frame['points'])
            xyz = np.array([[p[k] for k in ('x', 'y', 'z')] for p in clean if p['pressure']>config.pressure_threshold], dtype=float).reshape(-1, 3)
            xyz *= 1000. if config.coordinate_units == 'm' else 1.
            for label, fit, candidates in [('baseline', old_matching.fit_model, old_models),
                                            ('adaptive', matching.fit_model, models)]:
                classifier.fit_model = fit
                start = time.perf_counter()
                result = classifier.identify_sensor(frame['points'], candidates, config,
                                                    boundary_points=frame.get('boundary_points', []))
                elapsed = time.perf_counter()-start
                fits = {}
                for candidate, fitted in result['fits'].items():
                    transform = np.array(fitted['transform'])
                    distances = models[candidate].nearest((xyz-transform[:3, 3])@transform[:3, :3])[1]
                    fits[candidate] = dict(compatible=fitted['compatible'],
                                          all_input_contact_max_mm=float(distances.max()),
                                          all_input_contact_rms_mm=float(np.sqrt(np.mean(distances**2))),
                                          hand_limit_max_mm=fitted['hand_limit_max_mm'],
                                          hand_validation_complete=fitted.get('hand_validation_complete', True),
                                          hand_limit_max_is_lower_bound=fitted.get('hand_limit_max_is_lower_bound', False),
                                          contact_fit_compatible=fitted.get('contact_fit_compatible', fitted['compatible']),
                                          pose_evidence=fitted.get('pose_evidence', 'full_constraints'),
                                          starts_evaluated=fitted['starts_evaluated'],
                                          promoted_starts=fitted.get('promoted_starts'))
                row = dict(case=name, algorithm=label, seconds=elapsed, fits=fits,
                           classification=result['classification'], reason=result['reason'])
                report['results'].append(row)
                print(json.dumps(row), flush=True)
                if args.output:
                    args.output.write_text(json.dumps(report, indent=2), encoding='utf-8')
    finally:
        classifier.fit_model = original_fit


if __name__ == '__main__':
    main()
