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


def penetration_residual(closest, points, normals, tolerance=0., radius=4.):
    """Penalize inward nearest surfaces within a local tangent footprint (mm).

    The finite footprint avoids extending a finger plane across the grasp.
    Open CAD assemblies have no reliable signed interior. This oriented local
    surface test is deliberately reported as approximate, not solid exclusion.
    """
    delta = closest-points
    signed = np.einsum('ij,ij->i', delta, normals)
    lateral2 = np.maximum(0., np.einsum('ij,ij->i', delta, delta)-signed*signed)
    # Never discount penetration by distance or subtract the allowed error
    # before checking it. Tolerance is applied once by the acceptance gate.
    return np.where(lateral2 <= radius*radius, np.maximum(0., -signed), 0.)


def triangle_patch_depth(triangles, radius=4.):
    """Exact deepest triangle point in the square local hand patch.

    Input is in a patch frame: X/Y tangent, Z outward. The minimum of Z
    over a clipped triangle occurs at a triangle vertex, an edge/patch edge
    intersection, or a patch corner projected into the triangle.
    """
    if not len(triangles):
        return 0.
    t = np.asarray(triangles)
    best = 0.
    inside = (np.abs(t[..., :2]) <= radius+1e-9).all(axis=2)
    if inside.any():
        best = min(best, float(t[..., 2][inside].min()))
    a, b = t, np.roll(t, -1, axis=1)
    edge = b-a
    for axis in (0, 1):
        for bound in (-radius, radius):
            den = edge[..., axis]
            fraction = np.divide(bound-a[..., axis], den,
                                 out=np.full_like(den, np.inf), where=np.abs(den)>1e-12)
            valid = (fraction >= 0.) & (fraction <= 1.)
            f = np.where(valid, fraction, 0.)
            hit = a+f[..., None]*edge
            valid &= np.abs(hit[..., 1-axis]) <= radius+1e-9
            if valid.any():
                best = min(best, float(hit[..., 2][valid].min()))
    a, e, f = t[:, 0], t[:, 1]-t[:, 0], t[:, 2]-t[:, 0]
    den = e[:, 0]*f[:, 1]-e[:, 1]*f[:, 0]
    nonzero = np.abs(den)>1e-12
    for x in (-radius, radius):
        for y in (-radius, radius):
            dx, dy = x-a[:, 0], y-a[:, 1]
            u = np.divide(dx*f[:, 1]-dy*f[:, 0], den, out=np.zeros_like(den), where=nonzero)
            v = np.divide(e[:, 0]*dy-e[:, 1]*dx, den, out=np.zeros_like(den), where=nonzero)
            valid = nonzero & (u >= -1e-9) & (v >= -1e-9) & (u+v <= 1.+1e-9)
            if valid.any():
                z = a[:, 2]+u*e[:, 2]+v*f[:, 2]
                best = min(best, float(z[valid].min()))
    return -best


def triangle_penetrations(model, rotation, translation, points, normals, radius=4.):
    """Audit ALL triangles against finite tangent hand patches, no point sampling.

    Patches extend inward. This is a conservative oriented surface model,
    not a watertight reconstruction of the hand interior.
    """
    local_points = (points-translation)@rotation
    local_normals = normals@rotation
    vertices = model.mesh.vertices
    tree = model.mesh.triangles_tree
    bounds = model.mesh.bounds
    corners = np.array([[x,y,z] for x in bounds[:,0] for y in bounds[:,1] for z in bounds[:,2]])
    errors = np.zeros(len(points))
    for i, (point, normal) in enumerate(zip(local_points, local_normals)):
        helper = np.eye(3)[np.argmin(np.abs(normal))]
        u = np.cross(normal, helper); u /= np.linalg.norm(u)
        v = np.cross(normal, u)
        basis = np.array([u, v, normal]).T
        # Bound the inward prism by the candidate's actual bounds for the
        # spatial query, without capping allowed penetration depth.
        low = min(0., float(((corners-point)@normal).min()))-1e-7
        prism = np.array([[x,y,z] for x in (-radius,radius) for y in (-radius,radius) for z in (low,0.)])@basis.T+point
        box = np.r_[prism.min(axis=0)-1e-7, prism.max(axis=0)+1e-7]
        indices = list(tree.intersection(box))
        if indices:
            triangles = (vertices[model.mesh.faces[indices]]-point)@basis
            errors[i] = triangle_patch_depth(triangles, radius)
    return errors


def sampled_patch_penetrations(samples, points, normals, radius=4.):
    """Deepest sampled surface in each inward patch, for search guidance only.

    Use the same square patch frame as the exact triangle audit. Final checks
    must still catch triangle interiors and small features between samples.
    """
    errors = np.zeros(len(points))
    for start in range(0, len(points), 64):
        p, n = points[start:start+64], normals[start:start+64]
        helper = np.eye(3)[np.argmin(np.abs(n), axis=1)]
        u = np.cross(n, helper)
        u /= np.linalg.norm(u, axis=1)[:, None]
        v = np.cross(n, u)
        delta = samples[None, :, :]-p[:, None, :]
        x = np.einsum('ijk,ik->ij', delta, u)
        y = np.einsum('ijk,ik->ij', delta, v)
        z = np.einsum('ijk,ik->ij', delta, n)
        inside = (np.abs(x) <= radius) & (np.abs(y) <= radius)
        errors[start:start+len(p)] = np.maximum(0., np.where(inside, -z, 0.).max(axis=1))
    return errors
