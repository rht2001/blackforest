"""Deterministic multistart rigid ICP, followed by exact triangle refinement."""
import itertools

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
from .limits import penetration_residual, triangle_penetrations


def _axes(points):
    _, _, vt = np.linalg.svd(points - points.mean(axis=0), full_matrices=False)
    basis = vt.T
    if np.linalg.det(basis) < 0:
        basis[:, -1] *= -1
    return basis


def _rigid(source, target):
    a, b = source.mean(axis=0), target.mean(axis=0)
    u, _, vt = np.linalg.svd((source-a).T @ (target-b))
    r = vt.T @ np.diag([1., 1., np.linalg.det(vt.T @ u.T)]) @ u.T
    return r, b-r@a


def fit_model(model, contacts, exclusions, radii, config, *, boundary_xyz=None, boundary_normals=None):
    """All candidates receive the same starts, iterations and refinement budget.

    Transform convention: p_observation_mm = R @ p_stl_mm + t_mm.
    Local optimization does not certify global infeasibility.
    """
    has_limits = boundary_xyz is not None and len(boundary_xyz) > 0
    def limits(r, t, exact=False):
        if not has_limits:
            return np.array([])
        local = (boundary_xyz-t)@r
        if exact:
            closest = model.nearest(local)[0]
        else:
            closest = model.samples[model.tree.query(local)[1]]
        return penetration_residual(closest@r.T+t, boundary_xyz, boundary_normals)
    center = contacts.mean(axis=0)
    ca = _axes(contacts)
    ma = _axes(model.samples)
    rng = np.random.default_rng(config.seed)
    rotations = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([-1, 1], repeat=3):
            r = ca @ (np.eye(3)[:, perm] * signs) @ ma.T
            if np.linalg.det(r) > 0:
                rotations.append(r)
    rotations.extend(Rotation.random(config.random_starts, random_state=rng).as_matrix())
    starts = []
    for r in rotations:
        # Partial grasps can be far from the object's center. Vary the axial
        # location and include off-axis anchors; do not assume centered contact.
        for fraction in (-.35, 0., .35):
            anchor = model.samples.mean(axis=0) + fraction * np.ptp(model.samples @ ma[:, 0]) * ma[:, 0]
            starts.append((r.copy(), center - r@anchor))
    for _ in range(config.random_starts):
        r = Rotation.random(random_state=rng).as_matrix()
        starts.append((r, center-r@model.samples[rng.integers(len(model.samples))]))

    coarse = []
    for r, t in starts:
        for _ in range(config.icp_iterations):
            local = (contacts-t)@r
            _, index = model.tree.query(local)
            nr, nt = _rigid(model.samples[index], contacts)
            delta = np.linalg.norm(nt-t) + np.linalg.norm(nr-r)
            r, t = nr, nt
            if delta < 1e-5:
                break
        d, _ = model.tree.query((contacts-t)@r)
        penalty = 0.
        if len(exclusions):
            ed, _ = model.tree.query((exclusions-t)@r)
            penalty = np.mean(np.maximum(0., radii-ed)**2)
        boundary_error = limits(r,t)
        if len(boundary_error):
            penalty += np.mean(boundary_error**2)
        coarse.append((float(np.mean(d*d)+penalty), r, t))
    coarse.sort(key=lambda item: item[0])

    def residual(parameters):
        r = Rotation.from_rotvec(parameters[:3]).as_matrix()
        t = parameters[3:]
        _, d = model.nearest((contacts-t)@r)
        # Focus refinement on the worst contacts once they exceed tolerance;
        # a dense well-fitting patch must not drown out a single bad contact.
        d = np.r_[d, np.maximum(0., d-config.tolerance_mm)*np.sqrt(len(contacts))]
        if len(exclusions):
            _, ed = model.nearest((exclusions-t)@r)
            # Balance evidence types independently of their point counts.
            d = np.r_[d, np.maximum(0., radii-ed)*np.sqrt(len(contacts)/len(exclusions))]
        boundary_error = limits(r,t,exact=True)
        if len(boundary_error):
            d = np.r_[d, boundary_error*np.sqrt(len(contacts)/len(boundary_error))
                       *config.tolerance_mm/config.hand_tolerance_mm]
        return d / config.tolerance_mm

    fits = []
    for _, r, t in coarse[:config.refine_starts]:
        opt = least_squares(residual, np.r_[Rotation.from_matrix(r).as_rotvec(), t],
                            max_nfev=config.max_nfev, diff_step=1e-4,
                            ftol=1e-5, xtol=1e-5, gtol=1e-5)
        r = Rotation.from_rotvec(opt.x[:3]).as_matrix()
        t = opt.x[3:]
        _, d = model.nearest((contacts-t)@r)
        ed = model.nearest((exclusions-t)@r)[1] if len(exclusions) else np.array([])
        contradiction = np.maximum(0., radii-ed)
        rmse = float(np.sqrt(np.mean(d*d)))
        penalty = float(np.sqrt(np.mean(contradiction**2))) if len(ed) else 0.
        # A nearest surface can hide a deeper crossing or an enclosing shape.
        # Audit every triangle in each hand patch before accepting a pose.
        boundary_error = (triangle_penetrations(model, r, t, boundary_xyz, boundary_normals)
                          if has_limits else np.array([]))
        boundary_rmse = float(np.sqrt(np.mean(boundary_error**2))) if len(boundary_error) else 0.
        boundary_max = float(boundary_error.max()) if len(boundary_error) else 0.
        score = float(np.sqrt(rmse**2+penalty**2+boundary_rmse**2)/config.tolerance_mm)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = r, t
        fits.append(dict(score=score, rmse_mm=rmse, max_error_mm=float(d.max()),
                         inlier_fraction=float(np.mean(d <= config.tolerance_mm)),
                         exclusion_rmse_mm=penalty,
                         exclusion_max_mm=float(contradiction.max()) if len(contradiction) else 0.,
                         hand_limit_rmse_mm=boundary_rmse, hand_limit_max_mm=boundary_max,
                         transform=transform.tolist(), optimizer_success=bool(opt.success),
                         evaluations=int(opt.nfev)))
    for fit in fits:
        fit['compatible'] = (fit['max_error_mm'] <= config.tolerance_mm
                             and fit['exclusion_max_mm'] <= config.tolerance_mm
                             and fit['hand_limit_max_mm'] <= config.hand_tolerance_mm)
        fit['contact_tolerance_mm'] = config.tolerance_mm
        fit['hand_tolerance_mm'] = config.hand_tolerance_mm
        fit['rejection_reasons'] = (["contact_outside_tolerance"] if fit['max_error_mm'] > config.tolerance_mm else [])
        if fit['exclusion_max_mm'] > config.tolerance_mm:
            fit['rejection_reasons'].append('certified_clearance_violated')
        if fit['hand_limit_max_mm'] > config.hand_tolerance_mm:
            fit['rejection_reasons'].append('hand_penetration')
    # Feasibility takes precedence over average fit error.
    best = min(fits, key=lambda f: (not f['compatible'], f['score']))
    best['starts_evaluated'] = len(starts)
    best['refinements'] = len(fits)
    best['consistent_refinements'] = sum(abs(f['score']-best['score']) <= config.stability_margin for f in fits)
    return best
