import itertools

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .contact_directions import pressure_directions, direction_residual
from .limits import penetration_residual, sampled_patch_penetrations, triangle_penetrations
from .evidence import contact_hand_conflicts


def _spaced_indices(points, spacing_mm, normals=None):
    """Keep one original observation per radius neighborhood, deterministically.

    Hand patches with normals differing by more than 20 degrees remain distinct.
    Test true Euclidean distances so points across a voxel edge also merge.
    """
    if spacing_mm <= 0 or len(points) < 2:
        return np.arange(len(points))
    tree = cKDTree(points)
    suppressed = np.zeros(len(points), dtype=bool)
    chosen = []
    for index in range(len(points)):
        if suppressed[index]:
            continue
        chosen.append(index)
        neighbors = np.asarray(tree.query_ball_point(points[index], spacing_mm), dtype=int)
        if normals is not None:
            neighbors = neighbors[normals[neighbors] @ normals[index] >= np.cos(np.deg2rad(20.))]
        suppressed[neighbors] = True
    return np.asarray(chosen, dtype=int)


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


def _spread_indices(points, budget, normals=None):
    """Keep spatial coverage for a bounded preview search, not final scoring."""
    if len(points) <= budget:
        return np.arange(len(points))
    features = np.column_stack((points, normals*8.)) if normals is not None else points
    chosen = np.empty(budget, dtype=int)
    distances = np.full(len(points), np.inf)
    next_index = int(np.argmin(features[:, 0]))
    for i in range(budget):
        chosen[i] = next_index
        distances = np.minimum(distances, np.sum((features-features[next_index])**2, axis=1))
        distances[chosen[:i+1]] = -1.
        next_index = int(np.argmax(distances))
    return np.sort(chosen)


def _distinct_starts(ranked, budget):
    """Retain separate pose basins before spending the expensive search budget.

    Compare object centers, not STL origins: equivalent rotations must not
    appear far apart merely because a CAD export has an offset origin.
    The caller supplies translations of the model bounding-box center.
    """
    selected = []
    for item in ranked:
        if all(Rotation.from_matrix(item[1]@other[1].T).magnitude() > np.pi/8
               or np.linalg.norm(item[2]-other[2]) > 5. for other in selected):
            selected.append(item)
        if len(selected) >= budget:
            break
    return selected


def _palm_starts(model, contacts, points, normals):
    """A geometric grasp prior, not a learned pose or a hard palm-contact claim."""
    if points is None or len(points) < 8:
        return [], None
    # Use the dominant normal cluster; side/wrist samples must not tilt the
    # palmar plane. Deterministic subsampling also handles accumulated runs.
    probes = normals[np.linspace(0, len(normals)-1, min(64, len(normals)), dtype=int)]
    support = normals @ probes.T > .97
    mask = support[:, np.argmax(support.sum(axis=0))]
    if mask.sum() < 8:
        return [], None
    normal = normals[mask].mean(axis=0)
    normal /= np.linalg.norm(normal)
    centered = points[mask]-points[mask].mean(axis=0)
    tangent = centered-np.outer(centered@normal, normal)
    _, singular, vt = np.linalg.svd(tangent, full_matrices=False)
    if singular[1]/np.sqrt(mask.sum()) < 3.:
        return [], None
    axis = vt[0]
    # Canonical sign makes start ordering reproducible across LAPACK builds.
    if axis[np.argmax(np.abs(axis))] < 0:
        axis = -axis
    basis = np.column_stack((axis, np.cross(normal, axis), normal))
    model_axes = _axes(model.samples)
    model_center = model.mesh.bounds.mean(axis=0)
    starts = []
    for flip in (0., np.pi):
        for roll in np.arange(8)*np.pi/4:
            r = basis @ Rotation.from_euler('zx', [flip, roll]).as_matrix() @ model_axes.T
            for fraction in (0., -.35, .35):
                anchor = model_center+fraction*np.ptp(model.samples@model_axes[:, 0])*model_axes[:, 0]
                t = contacts.mean(axis=0)-r@anchor
                # Start outside the rigid palm. This support plane is only an
                # initialization; actual constraints remain finite hand patches.
                low = ((model.mesh.vertices@r.T+t)@normal).min()
                t += normal*max(0., np.median(points[mask]@normal)-low)
                starts.append((r, t))
    return starts, normal


def fit_model(model, contacts, exclusions, radii, config, *, boundary_xyz=None, boundary_normals=None,
              palm_xyz=None, palm_normals=None, contact_pressures=None, progress=None, axis_hint=None):
    """Try a short quick pass, retaining the full search as fallback."""
    conflict = contact_hand_conflicts(contacts, boundary_xyz, boundary_normals,
                                     config.tolerance_mm, config.hand_tolerance_mm)
    if conflict['boundary_count']:
        if progress:
            progress(dict(stage='input_geometry_conflict', conflict=conflict))
        # The original full constraint system is provably infeasible. A
        # contact fit is useful for inspection, but may never become accepted.
        # Do not spend minutes driving the object away from measured contacts
        # to reduce contradictory approximate hand residuals.
        _, spread, axes = np.linalg.svd(contacts-contacts.mean(axis=0), full_matrices=False)
        extents = np.sort(model.mesh.extents)
        # Only the explicitly rejected inspection preview uses this soft
        # geometric prior. Partial contacts cannot establish a sensor axis or
        # identity, but a long cloud otherwise admits implausibly tilted fits.
        hint = (axes[0] if len(spread) >= 2 and spread[1] > 0
                and spread[0]/spread[1] >= 1.7 and extents[-1]/extents[-2] >= 2. else None)
        preview = fit_model(model, contacts, exclusions, radii, config,
                            contact_pressures=contact_pressures, progress=progress)
        model_axis = np.eye(3)[:, np.argmax(model.mesh.extents)]
        direction = np.asarray(preview['transform'])[:3, :3]@model_axis
        if hint is not None and abs(direction@hint) < np.cos(np.deg2rad(25.)):
            regularized = fit_model(model, contacts, exclusions, radii, config,
                                    contact_pressures=contact_pressures, progress=progress, axis_hint=hint)
            if (regularized['compatible'] or (not preview['compatible']
                    and regularized['max_error_mm'] < preview['max_error_mm'])):
                preview = regularized
            else:
                hint = None
                preview['preview_prior_fallback'] = 'contact_feasibility_preferred_over_axis_hint'
        else:
            hint = None
        transform = np.asarray(preview['transform'])
        witness_indices = np.asarray(conflict['boundary_indices'], dtype=int)
        violations = triangle_penetrations(model, transform[:3, :3], transform[:3, 3],
                                          np.asarray(boundary_xyz)[witness_indices],
                                          np.asarray(boundary_normals)[witness_indices])
        preview.update(contact_fit_compatible=preview['compatible'], compatible=False,
                       geometry_conflict=conflict, pose_evidence='contact_only_conflict_preview',
                       search_pass='inconsistent_hand_constraints',
                       score_basis='contact_and_clearance_only',
                       search_method='conflict_aware_contact_preview',
                       hand_limit_rmse_mm=float(np.sqrt(np.mean(violations**2))),
                       hand_limit_max_mm=float(violations.max()),
                       hand_limit_max_is_lower_bound=True,
                       hand_collision_count=int(np.count_nonzero(violations > config.hand_tolerance_mm)),
                       hand_constraints_checked=len(witness_indices), input_hand_points=len(boundary_xyz),
                       hand_validation_complete=False,
                       validation_method='exact_contacts_and_conflict_witnesses')
        if hint is not None:
            preview['preview_axis_prior'] = dict(axis=hint.tolist(), lever_arm_mm=4.,
                                                method='elongated_contact_cloud_soft_prior_not_identity_evidence')
        preview['rejection_reasons'] = list(dict.fromkeys(preview['rejection_reasons']+['input_geometry_conflict']))
        return preview
    kwargs = dict(boundary_xyz=boundary_xyz, boundary_normals=boundary_normals,
                  palm_xyz=palm_xyz, palm_normals=palm_normals,
                  contact_pressures=contact_pressures, progress=progress)
    if axis_hint is not None:
        kwargs['axis_hint'] = axis_hint
    if config.quick_preview:
        attempt = _fit_model(model, contacts, exclusions, radii, config, fast_attempt=True, **kwargs)
        if attempt['compatible']:
            attempt['search_pass'] = 'short_first_pass'
            return attempt
    result = _fit_model(model, contacts, exclusions, radii, config, **kwargs)
    result['search_pass'] = 'full_fallback' if config.quick_preview else 'detailed'
    return result


def _fit_model(model, contacts, exclusions, radii, config, *, boundary_xyz=None, boundary_normals=None,
               palm_xyz=None, palm_normals=None, contact_pressures=None, progress=None, fast_attempt=False, axis_hint=None):
    """Both candidates receive the same budget within each search pass.

    Transform convention: p_observation_mm = R @ p_stl_mm + t_mm.
    Local optimization does not certify global infeasibility.
    """
    input_contact_points = len(contacts)
    input_hand_points = len(boundary_xyz) if boundary_xyz is not None else 0
    input_contacts = contacts
    input_boundary_xyz, input_boundary_normals = boundary_xyz, boundary_normals
    directions = pressure_directions(contacts, contact_pressures, config.pressure_threshold)
    has_directions = bool(len(directions[0]))
    # Contact-only ICP is already cheap. Screening pays off when it avoids
    # repeated finite-difference evaluations of hand/direction constraints.
    hierarchical_search = not fast_attempt and (input_hand_points > 0 or has_directions)
    contacts = contacts[_spaced_indices(contacts, config.point_spacing_mm)]
    if boundary_xyz is not None and len(boundary_xyz):
        selected = _spaced_indices(boundary_xyz, config.point_spacing_mm, boundary_normals)
        boundary_xyz, boundary_normals = boundary_xyz[selected], boundary_normals[selected]
    if palm_xyz is not None and len(palm_xyz):
        selected = _spaced_indices(palm_xyz, config.point_spacing_mm, palm_normals)
        palm_xyz, palm_normals = palm_xyz[selected], palm_normals[selected]
    # Reduction changes search cost only. No measured contact or supplied hand
    # patch may disappear from the final feasibility decision.
    full_contacts = input_contacts
    full_boundary_xyz, full_boundary_normals = input_boundary_xyz, input_boundary_normals
    fit_contacts = contacts
    fit_boundary_xyz, fit_boundary_normals = boundary_xyz, boundary_normals
    iterations = min(config.icp_iterations, 8) if config.quick_preview else config.icp_iterations
    if fast_attempt:
        iterations = min(iterations, 3)
    if config.quick_preview:
        contacts = contacts[_spread_indices(contacts, 256)]
        if boundary_xyz is not None and len(boundary_xyz):
            selected = _spread_indices(boundary_xyz, 256, boundary_normals)
            boundary_xyz, boundary_normals = boundary_xyz[selected], boundary_normals[selected]
    elif hierarchical_search:
        contacts = contacts[_spread_indices(contacts, 128)]
        if boundary_xyz is not None and len(boundary_xyz):
            selected = _spread_indices(boundary_xyz, 96, boundary_normals)
            boundary_xyz, boundary_normals = boundary_xyz[selected], boundary_normals[selected]
    has_limits = boundary_xyz is not None and len(boundary_xyz) > 0
    collision_samples = model.samples[_spread_indices(model.samples, 384 if hierarchical_search else 1536)] if has_limits else np.empty((0, 3))
    def limits(r, t, surface=False):
        if not has_limits:
            return np.array([])
        local = (boundary_xyz-t)@r
        if surface:
            closest = model.nearest_sampled(local)[0]
        else:
            closest = model.samples[model.tree.query(local)[1]]
        nearest = penetration_residual(closest@r.T+t, boundary_xyz, boundary_normals)
        # The nearest surface may be outside while the far side crosses the
        # hand. Include deeper surface samples in the optimization itself.
        depth = sampled_patch_penetrations(collision_samples@r.T+t, boundary_xyz, boundary_normals)
        return np.maximum(nearest, depth)
    center = contacts.mean(axis=0)
    ca = _axes(contacts if len(contacts) >= 3 else input_contacts)
    ma = _axes(model.samples)
    rng = np.random.default_rng(config.seed)
    rotations = []
    for perm in itertools.permutations(range(3)):
        for signs in itertools.product([-1, 1], repeat=3):
            r = ca @ (np.eye(3)[:, perm] * signs) @ ma.T
            if np.linalg.det(r) > 0:
                rotations.append(r)
    rotations.extend(Rotation.random(config.random_starts, random_state=rng).as_matrix())
    if fast_attempt:
        rotations = [rotations[i] for i in np.linspace(0, len(rotations)-1, 8, dtype=int)]
    starts = []
    for r in rotations:
        # Partial grasps can be far from the object's center. Vary the axial
        # location and include off-axis anchors; do not assume centered contact.
        for fraction in ((0.,) if fast_attempt else (-.35, 0., .35)):
            anchor = model.samples.mean(axis=0) + fraction * np.ptp(model.samples @ ma[:, 0]) * ma[:, 0]
            starts.append((r.copy(), center - r@anchor))
    for _ in range(0 if fast_attempt else config.random_starts):
        r = Rotation.random(random_state=rng).as_matrix()
        starts.append((r, center-r@model.samples[rng.integers(len(model.samples))]))

    preferred, palm_normal = _palm_starts(model, contacts, palm_xyz, palm_normals)
    if fast_attempt and preferred:
        preferred = [preferred[i] for i in np.linspace(0, len(preferred)-1, min(8, len(preferred)), dtype=int)]

    def search_residual(r, t, surface=False):
        d = model.nearest_sampled((contacts-t)@r)[1] if surface else model.tree.query((contacts-t)@r)[0]
        d = np.r_[d, np.maximum(0., d-config.tolerance_mm)*np.sqrt(len(contacts))]
        if len(exclusions):
            ed = model.nearest_sampled((exclusions-t)@r)[1] if surface else model.tree.query((exclusions-t)@r)[0]
            d = np.r_[d, np.maximum(0., radii-ed)*np.sqrt(len(contacts)/len(exclusions))]
        boundary_error = limits(r, t, surface=surface)
        if len(boundary_error):
            # Worst violations matter even with thousands of accumulated limits.
            weight = config.tolerance_mm/config.hand_tolerance_mm
            d = np.r_[d, boundary_error*np.sqrt(len(contacts)/len(boundary_error))*weight,
                      boundary_error.max()*np.sqrt(len(contacts))*weight]
        if has_directions:
            angular = direction_residual(model, r, t, directions)
            d = np.r_[d, angular*np.sqrt(len(contacts)/len(directions[0]))]
        if axis_hint is not None:
            model_axis = np.eye(3)[:, np.argmax(model.mesh.extents)]
            d = np.r_[d, np.cross(r@model_axis, axis_hint)*4.*np.sqrt(len(contacts))]
        return d/config.tolerance_mm

    coarse = []
    # First slide/turn a longitudinal grasp along the palm (four DOF). Tilt
    # becomes available in sampled-surface six-DOF refinement.
    for index, (r0, t0) in enumerate(preferred):
        if progress:
            progress(dict(stage='palm_initialization', completed=index, total=len(preferred)))
        def decode(p):
            return Rotation.from_rotvec(p[0]*palm_normal).as_matrix()@r0, t0+p[1:]
        opt = least_squares(lambda p: search_residual(*decode(p)), np.zeros(4),
                            x_scale=[1., 20., 20., 20.],
                            max_nfev=min(iterations, 3) if hierarchical_search else iterations)
        r, t = decode(opt.x)
        error = search_residual(r, t)
        coarse.append((float(np.mean(error**2)), r, t, 'palm_aligned'))
    for start_index, (r, t) in enumerate(starts):
        if progress and start_index % 8 == 0:
            progress(dict(stage='alignment', completed=start_index, total=len(starts)))
        for _ in range(min(iterations, 6) if hierarchical_search else iterations):
            local = (contacts-t)@r
            _, index = model.tree.query(local)
            nr, nt = _rigid(model.samples[index], contacts)
            delta = np.linalg.norm(nt-t) + np.linalg.norm(nr-r)
            r, t = nr, nt
            if delta < 1e-5:
                break
        if not hierarchical_search and (has_limits or has_directions):
            # ICP alone ignores the hand. Correct each fallback pose with
            # contact AND hand residuals before deciding which starts survive.
            r0, t0 = r.copy(), t.copy()
            def decode_free(p):
                return Rotation.from_rotvec(p[:3]).as_matrix()@r0, t0+p[3:]
            opt = least_squares(lambda p: search_residual(*decode_free(p)), np.zeros(6),
                                x_scale=[1., 1., 1., 20., 20., 20.], max_nfev=iterations)
            r, t = decode_free(opt.x)
        if has_limits or has_directions:
            error = search_residual(r, t)
            cost = float(np.mean(error**2))
        else:
            # Preserve the established contact-only multistart ranking.
            d = model.tree.query((contacts-t)@r)[0]
            cost = float(np.mean(d*d))
            if len(exclusions):
                ed = model.tree.query((exclusions-t)@r)[0]
                cost += float(np.mean(np.maximum(0., radii-ed)**2))
        coarse.append((cost, r, t, 'multistart'))
    coarse.sort(key=lambda item: item[0])

    screened_starts = len(coarse)
    if hierarchical_search:
        # Successive allocation: broad cheap screening, then constrained
        # alignment only for distinct promising basins. A palm hypothesis is
        # reserved so the reduced screening cloud cannot discard that family.
        model_center = model.mesh.bounds.mean(axis=0)
        centered = [(cost, r, r@model_center+t, label) for cost, r, t, label in coarse]
        promoted = _distinct_starts(centered, max(12, config.refine_starts*4))
        palm_best = next((item for item in centered if item[3] == 'palm_aligned'), None)
        if palm_best is not None and not any(item is palm_best for item in promoted):
            promoted = promoted[:-1]+[palm_best]
        contacts = fit_contacts
        boundary_xyz, boundary_normals = fit_boundary_xyz, fit_boundary_normals
        collision_samples = model.samples[_spread_indices(model.samples, 1536)] if has_limits else np.empty((0, 3))
        coarse = []
        for index, (_, r, center_t, label) in enumerate(promoted):
            if progress:
                progress(dict(stage='alignment', completed=index, total=len(promoted)))
            t = center_t-r@model_center
            for _ in range(iterations):
                nearest = model.tree.query((contacts-t)@r)[1]
                nr, nt = _rigid(model.samples[nearest], contacts)
                delta = np.linalg.norm(nt-t)+np.linalg.norm(nr-r)
                r, t = nr, nt
                if delta < 1e-5:
                    break
            if has_limits or has_directions:
                r0, t0 = r.copy(), t.copy()
                def decode_promoted(p):
                    return Rotation.from_rotvec(p[:3]).as_matrix()@r0, t0+p[3:]
                opt = least_squares(lambda p: search_residual(*decode_promoted(p)), np.zeros(6),
                                    x_scale=[1., 1., 1., 20., 20., 20.], max_nfev=iterations)
                r, t = decode_promoted(opt.x)
            coarse.append((float(np.mean(search_residual(r, t)**2)), r, t, label))
        coarse.sort(key=lambda item: item[0])

    def residual(parameters):
        r = Rotation.from_rotvec(parameters[:3]).as_matrix()
        t = parameters[3:]
        return search_residual(r, t, surface=True)
    fits = []
    # Retain different orientations instead of spending all refinements on
    # starts that converged to the same roll and translation.
    selected = []
    refine_count = min(config.refine_starts, 2) if fast_attempt else config.refine_starts
    for item in coarse:
        if all(Rotation.from_matrix(item[1]@other[1].T).magnitude() > np.pi/8
               or np.linalg.norm(item[2]-other[2]) > 5. for other in selected):
            selected.append(item)
        if len(selected) == refine_count:
            break
    if len(selected) < refine_count:
        selected.extend(item for item in coarse if not any(item is old for old in selected)
                        )
        selected = selected[:refine_count]
    refinement_total = len(selected)
    for refinement_index, (_, r, t, initialization) in enumerate(selected):
        if progress:
            progress(dict(stage='refining', completed=refinement_index, total=refinement_total))
        opt = least_squares(residual, np.r_[Rotation.from_matrix(r).as_rotvec(), t],
                            max_nfev=min(config.max_nfev, 15) if config.quick_preview else config.max_nfev, diff_step=1e-4,
                            ftol=1e-5, xtol=1e-5, gtol=1e-5)
        r = Rotation.from_rotvec(opt.x[:3]).as_matrix()
        t = opt.x[3:]
        final_nearest = model.nearest
        _, d = final_nearest((full_contacts-t)@r)
        repair_evaluations = 0
        optimizer_success = bool(opt.success)
        if not config.quick_preview:
            # Constraint exchange: a discarded contact can expose a violation
            # after the reduced-cloud fit. Add the worst original observations
            # and optimize again rather than merely rejecting an otherwise
            # recoverable pose. Keep the usual search representatives as well.
            refinement_contacts = contacts
            for _ in range(2):
                violating = np.flatnonzero(d > config.tolerance_mm)
                if (not len(violating) or refinement_index >= 2
                        or np.mean(d <= config.tolerance_mm) < .9):
                    break
                # Repair almost-feasible contact fits, not poses that already
                # fail the hand constraints. Do not trade collision for contact.
                if has_limits and limits(r, t, surface=True).max() > config.hand_tolerance_mm:
                    break
                worst = violating[np.argsort(d[violating])[-32:]]
                contacts = np.unique(np.vstack((contacts, full_contacts[worst])), axis=0)
                def repair_residual(parameters):
                    rotation = Rotation.from_rotvec(parameters[:3]).as_matrix()
                    translation = parameters[3:]
                    distance = model.nearest((full_contacts[worst]-translation)@rotation)[1]
                    # A small interior target avoids asymptotic convergence
                    # just outside the unchanged acceptance tolerance.
                    violation = np.maximum(0., distance-.99*config.tolerance_mm)
                    return np.r_[residual(parameters), violation/config.tolerance_mm*10.*np.sqrt(len(contacts))]
                repair = least_squares(repair_residual, np.r_[Rotation.from_matrix(r).as_rotvec(), t],
                                       x_scale=[1., 1., 1., 20., 20., 20.],
                                       max_nfev=config.max_nfev, diff_step=1e-4,
                                       ftol=1e-5, xtol=1e-5, gtol=1e-5)
                repair_evaluations += int(repair.nfev)
                repaired_r = Rotation.from_rotvec(repair.x[:3]).as_matrix()
                repaired_t = repair.x[3:]
                _, repaired_d = final_nearest((full_contacts-repaired_t)@repaired_r)
                if (has_limits and limits(repaired_r, repaired_t, surface=True).max()
                        > config.hand_tolerance_mm):
                    break
                def violation_cost(distances):
                    return (float(np.sum(np.maximum(0., distances-config.tolerance_mm)**2)),
                            float(np.mean(distances**2)))
                if violation_cost(repaired_d) >= violation_cost(d):
                    break
                r, t, d = repaired_r, repaired_t, repaired_d
                optimizer_success = optimizer_success and bool(repair.success)
            contacts = refinement_contacts
        ed = final_nearest((exclusions-t)@r)[1] if len(exclusions) else np.array([])
        contradiction = np.maximum(0., radii-ed)
        rmse = float(np.sqrt(np.mean(d*d)))
        penalty = float(np.sqrt(np.mean(contradiction**2))) if len(ed) else 0.
        # A nearest surface can hide a deeper crossing or an enclosing shape.
        # Audit every triangle in each hand patch before accepting a pose.
        if progress:
            progress(dict(stage='checking_limits', completed=refinement_index, total=refinement_total))
        # Search may approximate distances, but acceptance always audits the
        # complete CAD against every supplied contact and hand patch.
        boundary_error = (triangle_penetrations(model, r, t, full_boundary_xyz, full_boundary_normals)
                          if has_limits else np.array([]))
        boundary_rmse = float(np.sqrt(np.mean(boundary_error**2))) if len(boundary_error) else 0.
        boundary_max = float(boundary_error.max()) if len(boundary_error) else 0.
        angular = direction_residual(model, r, t, directions)
        direction_rmse = float(np.sqrt(np.sum(angular**2)/len(directions[0]))) if has_directions else 0.
        axis_error = (4.*np.linalg.norm(np.cross(r@np.eye(3)[:, np.argmax(model.mesh.extents)], axis_hint))
                      if axis_hint is not None else 0.)
        score = float(np.sqrt(rmse**2+penalty**2+boundary_rmse**2+direction_rmse**2+axis_error**2)/config.tolerance_mm)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = r, t
        fits.append(dict(score=score, rmse_mm=rmse, max_error_mm=float(d.max()),
                         inlier_fraction=float(np.mean(d <= config.tolerance_mm)),
                         exclusion_rmse_mm=penalty,
                         exclusion_max_mm=float(contradiction.max()) if len(contradiction) else 0.,
                         hand_limit_rmse_mm=boundary_rmse, hand_limit_max_mm=boundary_max,
                         hand_collision_count=int(np.count_nonzero(boundary_error > config.hand_tolerance_mm)),
                         hand_constraints_checked=len(boundary_error),
                         transform=transform.tolist(), optimizer_success=optimizer_success,
                         initialization=initialization,
                         pressure_direction_patches=len(directions[0]),
                         pressure_direction_rmse_mm=direction_rmse,
                         pressure_direction_method='pressure_weighted_local_planes_unsigned_soft',
                         refinement_method='sample_guided_local_triangles',
                         validation_method='exact_full_mesh',
                         quick_preview=config.quick_preview,
                         contact_repair_evaluations=repair_evaluations,
                         evaluations=int(opt.nfev)))
        # Quick placement needs one feasible pose, after all existing final checks.
        if (config.quick_preview and fits[-1]['max_error_mm'] <= config.tolerance_mm
                and fits[-1]['exclusion_max_mm'] <= config.tolerance_mm
                and fits[-1]['hand_limit_max_mm'] <= config.hand_tolerance_mm):
            break
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
    best['starts_evaluated'] = screened_starts
    best['promoted_starts'] = len(coarse)
    best['search_method'] = ('coarse_to_fine_multistart' if hierarchical_search else
                             'bounded_preview' if config.quick_preview else 'multistart')
    best['palm_starts_evaluated'] = len(preferred)
    best['palm_initial_dof'] = 4 if preferred else None
    best['refinement_dof'] = 6
    best['search_contact_points'] = len(contacts)
    best['search_hand_points'] = len(boundary_xyz) if has_limits else 0
    best['scored_contact_points'] = len(full_contacts)
    best['input_contact_points'] = input_contact_points
    best['input_hand_points'] = input_hand_points
    best['point_spacing_mm'] = config.point_spacing_mm
    best['refinements'] = len(fits)
    best['consistent_refinements'] = sum(abs(f['score']-best['score']) <= config.stability_margin for f in fits)
    return best
