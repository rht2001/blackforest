"""Consume an existing viewer export; no controller imports or hardware IO."""
import csv
import json
from pathlib import Path

from .classifier import IdentificationConfig, identify_sensor
from .thumb_prior import thumb_closure_hint


def normalize_frame(frame):
    """Accept the current twin's native saved-contact format without rescaling."""
    if isinstance(frame, dict) and frame.get('units') == 'metres' and frame.get('frame') == 'left_base_link':
        points = []
        for point in frame.get('points', []):
            xyz = point.get('position_m') if isinstance(point, dict) else None
            if not isinstance(xyz, (list, tuple)) or len(xyz) != 3:
                points.append({'valid': False})
                continue
            points.append(dict(id=point.get('id'), x=xyz[0], y=xyz[1], z=xyz[2],
                               pressure=point.get('value'),
                               **{k: point[k] for k in ('region', 'link', 'taxel_id', 'source_index', 'normal') if k in point}))
        return {**frame, 'units': 'm', 'points': points}
    return frame


def identify_frame(frame, models, *, tolerance_mm=3., hand_tolerance_mm=3., point_spacing_mm=2., calibrated=False, pressure_threshold=200., quick_preview=False, progress=None):
    """Frame validity must describe pose AND pressure, not just a fresh receipt."""
    if not isinstance(frame, dict) or frame.get('valid') is not True:
        return dict(classification='AMBIGUOUS', reason='invalid_or_stale_frame', fits={})
    frame = normalize_frame(frame)
    if frame.get('units') not in ('mm', 'm') or not isinstance(frame.get('frame'), str):
        raise ValueError('Explicit coordinate units and frame name required')
    return identify_sensor(frame.get('points'), models, IdentificationConfig(
        coordinate_units=frame['units'], tolerance_mm=tolerance_mm, hand_tolerance_mm=hand_tolerance_mm, point_spacing_mm=point_spacing_mm,
        pressure_threshold=pressure_threshold, quick_preview=quick_preview,
        calibrated=calibrated and frame.get('geometry_status') == 'CALIBRATED'),
        boundary_points=frame.get('boundary_points', []), progress=progress, coordinate_frame=frame['frame'],
        hand_snapshot=frame.get('hand_snapshot'),
        thumb_hint=thumb_closure_hint(frame, pressure_threshold))


def read_points(path):
    """CSV has no frame metadata: callers must explicitly supply units."""
    path = Path(path)
    if path.suffix.lower() == '.csv':
        with path.open(newline='', encoding='utf-8-sig') as stream:
            points = []
            for row in csv.DictReader(stream):
                point = {'id': row.get('id')}
                for key in ('x', 'y', 'z', 'pressure'):
                    try:
                        point[key] = float(row[key])
                    except (KeyError, TypeError, ValueError):
                        point[key] = None
                points.append(point)
            return points
    return normalize_frame(json.loads(path.read_text(encoding='utf-8-sig')))
