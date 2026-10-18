"""Compatibility decisions with explicit ambiguity and calibration gates."""
from dataclasses import dataclass
from collections.abc import Mapping

import numpy as np

from .matching import fit_model
from .limits import valid_boundaries


@dataclass(frozen=True)
class IdentificationConfig:
    pressure_threshold: float = 200.
    tolerance_mm: float = 1.5
    coordinate_units: str = 'mm'
    calibrated: bool = False
    min_contacts: int = 8
    min_spread_mm: float = 3.
    min_inlier_fraction: float = .9
    score_margin: float = 1.
    stability_margin: float = .35
    random_starts: int = 24
    refine_starts: int = 4
    icp_iterations: int = 50
    max_nfev: int = 80
    seed: int = 731

    def __post_init__(self):
        if self.coordinate_units not in ('mm', 'm'):
            raise ValueError('coordinate_units must be mm or m')
        for key in ('tolerance_mm', 'min_spread_mm', 'score_margin', 'stability_margin'):
            if not np.isfinite(getattr(self, key)) or getattr(self, key) <= 0:
                raise ValueError(key+' must be finite and positive')
        if not np.isfinite(self.pressure_threshold) or not 0 <= self.pressure_threshold <= 4095:
            raise ValueError('Invalid pressure threshold')
        if not 0 < self.min_inlier_fraction <= 1 or not isinstance(self.calibrated, bool):
            raise ValueError('Invalid validity configuration')
        for key in ('min_contacts', 'random_starts', 'refine_starts', 'icp_iterations', 'max_nfev'):
            v = getattr(self, key)
            if isinstance(v, bool) or not isinstance(v, int) or v < (4 if key == 'min_contacts' else 1):
                raise ValueError(key+' must be a positive integer (min_contacts >= 4)')


def valid_measurements(points):
    """Keep zero pressure; reject missing readings and duplicate/invalid IDs."""
    clean, invalid, seen = [], 0, set()
    if not isinstance(points, (list, tuple)):
        return [], 1
    for point in points:
        try:
            if not isinstance(point, Mapping) or point.get('valid', True) is not True:
                raise ValueError()
            ident = point['id']
            if not isinstance(ident, (str, int)) or isinstance(ident, bool) or ident in seen:
                raise ValueError()
            seen.add(ident)
            values = [point[k] for k in ('x', 'y', 'z', 'pressure')]
            if any(isinstance(v, bool) or not isinstance(v, (int, float)) or not np.isfinite(v) for v in values):
                raise ValueError()
            if not 0 <= point['pressure'] <= 4095:
                raise ValueError()
            clean.append(dict(point))
        except (KeyError, TypeError, ValueError):
            invalid += 1
    return clean, invalid


def identify_sensor(points, models, config=None, *, boundary_points=()):
    config = config or IdentificationConfig()
    if set(models) != {'SENSOR_A', 'SENSOR_B'}:
        raise ValueError('Exactly SENSOR_A and SENSOR_B models are required')
    boundaries = valid_boundaries(boundary_points)
    clean, invalid = valid_measurements(points)
    active = [p for p in clean if p['pressure'] > config.pressure_threshold]
    result = dict(classification='AMBIGUOUS', reason='insufficient_contacts', fits={},
                  valid_points=len(clean), invalid_points=invalid, contacts=len(active),
                  units='mm', calibrated=config.calibrated,
                  search='finite multistart local optimization; not a global infeasibility certificate')
    inactive = [p for p in clean if p['pressure'] <= config.pressure_threshold and p.get('normal') is not None]
    inactive = valid_boundaries(inactive)
    limits = boundaries + inactive
    result.update(hand_boundary_points=len(boundaries), inactive_taxel_limits=len(inactive),
                  limit_method='approximate local oriented surface; no solid collision certificate')
    if len(active) < config.min_contacts:
        return result
    scale = 1000. if config.coordinate_units == 'm' else 1.
    xyz = np.array([[p[k] for k in ('x', 'y', 'z')] for p in active])*scale
    singular = np.linalg.svd(xyz-xyz.mean(axis=0), compute_uv=False)/np.sqrt(len(xyz))
    if singular[1] < config.min_spread_mm:
        result['reason'] = 'insufficient_spatial_coverage'
        return result
    # A non-contact cell is NOT an empty-space constraint. Only a separately
    # certified surface-free ball can be used, in physical millimetres.
    negative = [p for p in clean if p['pressure'] <= config.pressure_threshold
                and p.get('surface_clearance_certified') is True
                and isinstance(p.get('surface_clearance_mm'), (int, float))
                and not isinstance(p.get('surface_clearance_mm'), bool)
                and np.isfinite(p['surface_clearance_mm']) and p['surface_clearance_mm'] > 0]
    exclusions = np.array([[p[k] for k in ('x', 'y', 'z')] for p in negative], dtype=float).reshape(-1, 3)*scale
    radii = np.array([p['surface_clearance_mm'] for p in negative])
    result['certified_exclusions'] = len(negative)
    kwargs = {}
    if limits:
        kwargs = dict(boundary_xyz=np.array([[p[k] for k in ('x','y','z')] for p in limits])*scale,
                      boundary_normals=np.array([p['normal'] for p in limits]))
    result['fits'] = {name: fit_model(model, xyz, exclusions, radii, config, **kwargs)
                      for name, model in sorted(models.items())}
    fits = result['fits']
    compatible = [name for name, fit in fits.items() if fit['compatible']]
    if len(compatible) == 2:
        result['reason'] = 'both_candidates_compatible'
    elif not compatible:
        result['reason'] = 'neither_candidate_compatible_within_search'
    else:
        best = compatible[0]
        other = next(n for n in fits if n != best)
        result['best_candidate'] = best
        if not config.calibrated:
            result['reason'] = 'calibration_unverified'
        elif not all(f['optimizer_success'] and f['consistent_refinements'] >= 2 for f in fits.values()):
            result['reason'] = 'alignment_search_unstable'
        elif fits[other]['score'] - fits[best]['score'] < config.score_margin:
            result['reason'] = 'score_margin_too_small'
        else:
            result.update(classification=best, reason='one_candidate_clearly_more_consistent')
    return result
