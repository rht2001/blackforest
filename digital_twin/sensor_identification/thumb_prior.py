"""A thumb closure observation orders candidates; it never confirms identity."""
import math


def thumb_closure_hint(frame, threshold):
    snapshot = frame.get('hand_snapshot') or frame
    if not isinstance(snapshot, dict):
        return None
    calibration = snapshot.get('kinematics_calibration')
    if not isinstance(calibration, dict) or not isinstance(calibration.get('actuators'), dict):
        return None
    calibration = calibration['actuators'].get('thumb_bend')
    if not isinstance(calibration, dict):
        return None
    # RH56 telemetry uses [little, ring, middle, index, thumb bend, thumb rotation].
    sample = frame.get('thumb_closure_sample') if frame.get('measurement_source') == 'csv_run' else None
    if sample is None:
        if frame.get('measurement_source') == 'csv_run':
            return None  # Old accumulated clouds cannot pair a pose with its thumb contacts.
        sample = snapshot
    if not isinstance(sample, dict):
        return None
    try:
        angles = sample['angles_act']
        if len(angles) != 6 or any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or not 0 <= v <= 1000 for v in angles):
            return None
        opened, closed = calibration['raw_at_open'], calibration['raw_at_closed']
        if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in (opened, closed)) or opened == closed:
            return None
        fraction = (opened - angles[4]) / (opened - closed)
        if calibration.get('reversed') is True:
            fraction = 1 - fraction
        if not 0 <= fraction <= 1 or fraction < .85:
            return None
        points = sample.get('points', [])
        pressures = sample.get('thumb_pressures', [p.get('value', p.get('pressure')) for p in points if isinstance(p, dict) and str(p.get('region', '')).startswith('thumb_')])
        count = sum(not isinstance(p, bool) and isinstance(p, (int, float)) and math.isfinite(p) and threshold < p <= 4095 for p in pressures)
        if count < 3:
            return None
        return dict(reason='near_closed_thumb_with_contact', closure_fraction=fraction,
                    threshold_fraction=.85, thumb_contacts=count, provisional=True)
    except (KeyError, TypeError, ValueError, ZeroDivisionError):
        return None
