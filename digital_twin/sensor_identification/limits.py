"""Local oriented hand-surface limits; these do not certify solid collision."""
import numpy as np


def valid_boundaries(points):
    if not isinstance(points, (list, tuple)) or len(points) > 4096:
        raise ValueError('Expected at most 4096 hand boundary points')
    clean, seen = [], set()
    for p in points:
        if not isinstance(p, dict):
            raise ValueError('Invalid hand boundary point')
        ident, normal = p.get('id'), p.get('normal')
        if isinstance(ident, bool) or not isinstance(ident, (str, int)) or ident in seen:
            raise ValueError('Invalid or duplicate hand boundary ID')
        values = [p.get(k) for k in ('x', 'y', 'z')]
        if not isinstance(normal, (list, tuple)) or len(normal) != 3:
            raise ValueError('Boundary needs an outward normal')
        if any(isinstance(v, bool) or not isinstance(v, (float, int)) or not np.isfinite(v)
               for v in values+list(normal)):
            raise ValueError('Nonfinite boundary geometry')
        length = np.linalg.norm(normal)
        if not .9 < length < 1.1:
            raise ValueError('Boundary normal must have unit length')
        seen.add(ident)
        clean.append({**p, 'normal': (np.array(normal)/length).tolist()})
    return clean


def penetration_residual(closest, points, normals, tolerance, radius=4.):
    """Penalize inward nearest surfaces within a local tangent footprint (mm).

    Tangent weighting avoids extending a finger plane across the whole grasp.
    Open CAD assemblies have no reliable signed interior. This oriented local
    surface test is deliberately reported as approximate, not solid exclusion.
    """
    delta = closest-points
    signed = np.einsum('ij,ij->i', delta, normals)
    lateral2 = np.maximum(0., np.einsum('ij,ij->i', delta, delta)-signed*signed)
    weight = np.exp(-lateral2/(2*radius*radius))
    return np.maximum(0., -signed-tolerance)*weight
