import itertools
import time

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation
from .contact_directions import pressure_directions, direction_residual
from .limits import penetration_residual, sampled_patch_penetrations, triangle_penetrations


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
              palm_xyz=None, palm_normals=None, contact_pressures=None, thumb_xyz=None, progress=None, initial_pose=None):
    """Quick placement uses a bounded search; detailed mode uses all starts."""
    kwargs = dict(boundary_xyz=boundary_xyz, boundary_normals=boundary_normals,
                  palm_xyz=palm_xyz, palm_normals=palm_normals,
                  contact_pressures=contact_pressures, thumb_xyz=thumb_xyz, progress=progress, initial_pose=initial_pose)
    if config.quick_preview:
        attempt = _fit_model(model, contacts, exclusions, radii, config, fast_attempt=True, **kwargs)
        attempt['search_pass'] = 'quick_short_search'
        return attempt
    result = _fit_model(model, contacts, exclusions, radii, config, **kwargs)
    result['search_pass'] = 'detailed'
    return result


def _fit_model(model, contacts, exclusions, radii, config, *, boundary_xyz=None, boundary_normals=None,
               palm_xyz=None, palm_normals=None, contact_pressures=None, thumb_xyz=None, progress=None, fast_attempt=False, initial_pose=None):
    """Both candidates receive the same budget within each search pass.

    Transform convention: p_observation_mm = R @ p_stl_mm + t_mm.
    Local optimization does not certify global infeasibility.
    """
    input_contact_points = len(contacts)
    input_hand_points = len(boundary_xyz) if boundary_xyz is not None else 0
    input_contacts = contacts
    directions = pressure_directions(contacts, contact_pressures, config.pressure_threshold)
    has_directions = bool(len(directions[0]))
    contacts = contacts[_spaced_indices(contacts, config.point_spacing_mm)]
    if boundary_xyz is not None and len(boundary_xyz):
        selected = _spaced_indices(boundary_xyz, config.point_spacing_mm, boundary_normals)
        boundary_xyz, boundary_normals = boundary_xyz[selected], boundary_normals[selected]
    if palm_xyz is not None and len(palm_xyz):
        selected = _spaced_indices(palm_xyz, config.point_spacing_mm, palm_normals)
        palm_xyz, palm_normals = palm_xyz[selected], palm_normals[selected]
    full_contacts = contacts
    full_boundary_xyz, full_boundary_normals = boundary_xyz, boundary_normals
    iterations = min(config.icp_iterations, 8) if config.quick_preview else config.icp_iterations
    if fast_attempt:
        iterations = min(iterations, 3)
    if config.quick_preview:
        contacts = contacts[_spread_indices(contacts, 256)]
        if boundary_xyz is not None and len(boundary_xyz):
            selected = _spread_indices(boundary_xyz, 256, boundary_normals)
            boundary_xyz, boundary_normals = boundary_xyz[selected], boundary_normals[selected]
    has_limits = boundary_xyz is not None and len(boundary_xyz) > 0
    collision_samples = model.samples[_spread_indices(model.samples, 1536)] if has_limits else np.empty((0, 3))
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
        return d/config.tolerance_mm

    preview_time = 0.
    preview_stage = None
    preview_step = 0
    def publish_pose(stage, r, t, force=False):
        nonlocal preview_time, preview_stage, preview_step
        if progress is None:
            return
        now = time.monotonic()
        if not force and stage == preview_stage and now-preview_time < .25:
            return
        preview_time, preview_stage = now, stage
        preview_step += 1
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = r, t
        progress(dict(stage=stage, transform=transform.tolist(), pose_step=preview_step,
                      preview=True))

    def evaluated(stage, r, t, surface=False):
        publish_pose(stage, r, t)
        return search_residual(r, t, surface=surface)

    # Empirical poses are supplied only for matching CAD in left_base_link.
    # Parameterize translation by the bounding-box center, not the CAD origin.
    model_center = model.mesh.bounds.mean(axis=0)
    if initial_pose is not None:
        prior_center = np.array(initial_pose['center_mm'], dtype=float)
        prior_r = Rotation.from_euler('xyz', initial_pose['rotation_xyz_deg'], degrees=True).as_matrix()
        position_window = np.array(initial_pose['position_window_mm'], dtype=float)
        angular_window = np.deg2rad([30., 180., 30.])
        prior_bounds = (np.r_[-angular_window, -position_window], np.r_[angular_window, position_window])
        def prior_rotation(p):
            # Apply shaft roll before tilt to avoid a gimbal at a 90-degree roll.
            return prior_r @ Rotation.from_euler('yxz', [p[1], p[0], p[2]]).as_matrix()
        def decode_prior(p):
            rotation = prior_rotation(p)
            return rotation, prior_center+p[3:]-rotation@model_center
        def encode_prior(rotation, translation):
            angles = Rotation.from_matrix(prior_r.T@rotation).as_euler('yxz')
            angles = angles[[1, 0, 2]]
            values = np.r_[angles, rotation@model_center+translation-prior_center]
            return np.clip(values, prior_bounds[0]+1e-9, prior_bounds[1]-1e-9)
    coarse = []
    if initial_pose is not None and not fast_attempt:
        r, t = decode_prior(np.zeros(6))
        publish_pose('standard_placement', r, t, force=True)
        seed = least_squares(lambda p: evaluated('alignment', *decode_prior(p)), np.zeros(6),
                             bounds=prior_bounds, x_scale=[1., 1., 1., 20., 20., 20.],
                             max_nfev=min(config.max_nfev, 18), diff_step=1e-4)
        r, t = decode_prior(seed.x)
        coarse.append((float(np.mean(search_residual(r, t)**2)), r, t, 'empirical_grasp_prior'))
    staged = fast_attempt and (initial_pose is not None or bool(preferred) or (thumb_xyz is not None and len(thumb_xyz) >= 3))
    if staged:
        # One standard placement. Rotate about the CAD center, then slide;
        # never let rotation implicitly swing the object around the STL origin.
        model_center = model.mesh.bounds.mean(axis=0)
        r = preferred[0][0].copy() if preferred else ca @ ma.T
        anchor = center.copy()
        if preferred:
            palm_center = palm_xyz.mean(axis=0)
            anchor = palm_center + palm_normal * np.dot(center-palm_center, palm_normal)
            low = ((model.mesh.vertices-model_center) @ r.T + anchor) @ palm_normal
            anchor += palm_normal * max(0., np.median(palm_xyz @ palm_normal)-low.min())
        if initial_pose is not None:
            r, anchor = prior_r.copy(), prior_center.copy()
        t = anchor-r@model_center
        initial_transform = np.eye(4)
        initial_transform[:3, :3], initial_transform[:3, 3] = r, t
        thumb = thumb_xyz if thumb_xyz is not None and len(thumb_xyz) else contacts
        thumb = thumb[_spread_indices(thumb, 128)]

        placement_stage = 'adjusting_angle'
        def placement_residual(rotation, translation):
            error = evaluated(placement_stage, rotation, translation)
            distance = model.tree.query((thumb-translation)@rotation)[0]
            return np.r_[error, distance/config.tolerance_mm,
                         distance.max()*np.sqrt(len(thumb))/config.tolerance_mm]

        publish_pose('standard_placement', r, t, force=True)
        if progress:
            progress(dict(stage='adjusting_angle', completed=0, total=1))
        r0 = r.copy()
        def turn(p):
            rotation = (prior_rotation(p) if initial_pose is not None
                        else Rotation.from_rotvec(p).as_matrix()@r0)
            return rotation, anchor-rotation@model_center
        angle_options = {'bounds': (-angular_window, angular_window)} if initial_pose is not None else {}
        angle_fit = least_squares(lambda p: placement_residual(*turn(p)), np.zeros(3), **angle_options,
                                  max_nfev=min(config.max_nfev, 12), diff_step=1e-4)
        r, t = turn(angle_fit.x)
        publish_pose('adjusting_angle', r, t, force=True)
        placement_stage = 'moving_to_thumb'
        if progress:
            progress(dict(stage='moving_to_thumb', completed=0, total=1))
        slide_options = {'bounds': (t-position_window, t+position_window)} if initial_pose is not None else {}
        slide_fit = least_squares(lambda p: placement_residual(r, p), t, **slide_options,
                                  x_scale=[20., 20., 20.], max_nfev=min(config.max_nfev, 18),
                                  diff_step=1e-4)
        t = slide_fit.x
        publish_pose('moving_to_thumb', r, t, force=True)
        coarse.append((float(np.mean(search_residual(r, t)**2)), r, t, 'empirical_grasp_prior' if initial_pose is not None else 'center_angle_thumb_slide'))
        # The existing final refinement and full CAD audit still apply.
        starts, preferred = [], []
    # First slide/turn a longitudinal grasp along the palm (four DOF). Tilt
    # becomes available in sampled-surface six-DOF refinement.
    for index, (r0, t0) in enumerate(preferred):
        if progress:
            progress(dict(stage='palm_initialization', completed=index, total=len(preferred)))
        def decode(p):
            return Rotation.from_rotvec(p[0]*palm_normal).as_matrix()@r0, t0+p[1:]
        opt = least_squares(lambda p: evaluated('palm_initialization', *decode(p)), np.zeros(4),
                            x_scale=[1., 20., 20., 20.], max_nfev=iterations)
        r, t = decode(opt.x)
        error = search_residual(r, t)
        coarse.append((float(np.mean(error**2)), r, t, 'palm_aligned'))
    for start_index, (r, t) in enumerate(starts):
        if progress and start_index % 8 == 0:
            progress(dict(stage='alignment', completed=start_index, total=len(starts)))
        publish_pose('alignment', r, t, force=True)
        for _ in range(iterations):
            local = (contacts-t)@r
            _, index = model.tree.query(local)
            nr, nt = _rigid(model.samples[index], contacts)
            delta = np.linalg.norm(nt-t) + np.linalg.norm(nr-r)
            r, t = nr, nt
            publish_pose('alignment', r, t)
            if delta < 1e-5:
                break
        if has_limits or has_directions:
            # Correct positional ICP with hand limits and reliable patch
            # directions before deciding which starts survive.
            r0, t0 = r.copy(), t.copy()
            def decode_free(p):
                return Rotation.from_rotvec(p[:3]).as_matrix()@r0, t0+p[3:]
            opt = least_squares(lambda p: evaluated('alignment', *decode_free(p)), np.zeros(6),
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

    def residual(parameters):
        r = Rotation.from_rotvec(parameters[:3]).as_matrix()
        t = parameters[3:]
        return evaluated('refining', r, t, surface=True)
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
    if initial_pose is not None and not fast_attempt:
        empirical = next(item for item in coarse if item[3] == 'empirical_grasp_prior')
        if not any(item is empirical for item in selected):
            selected = [empirical] + selected
    refinement_total = len(selected)
    for refinement_index, (_, r, t, initialization) in enumerate(selected):
        if progress:
            progress(dict(stage='refining', completed=refinement_index, total=refinement_total))
        use_prior = initialization == 'empirical_grasp_prior'
        refine_fun = (lambda p: evaluated('refining', *decode_prior(p), surface=True)) if use_prior else residual
        refine_start = encode_prior(r, t) if use_prior else np.r_[Rotation.from_matrix(r).as_rotvec(), t]
        refine_options = {'bounds': prior_bounds, 'x_scale': [1., 1., 1., 20., 20., 20.]} if use_prior else {}
        opt = least_squares(refine_fun, refine_start, **refine_options,
                            max_nfev=min(config.max_nfev, 15) if config.quick_preview else config.max_nfev, diff_step=1e-4,
                            ftol=1e-5, xtol=1e-5, gtol=1e-5)
        if use_prior:
            r, t = decode_prior(opt.x)
        else:
            r = Rotation.from_rotvec(opt.x[:3]).as_matrix()
            t = opt.x[3:]
        publish_pose('checking_limits', r, t, force=True)
        final_nearest = model.nearest_sampled if config.quick_preview else model.nearest
        _, d = final_nearest((full_contacts-t)@r)
        ed = final_nearest((exclusions-t)@r)[1] if len(exclusions) else np.array([])
        contradiction = np.maximum(0., radii-ed)
        rmse = float(np.sqrt(np.mean(d*d)))
        penalty = float(np.sqrt(np.mean(contradiction**2))) if len(ed) else 0.
        # A nearest surface can hide a deeper crossing or an enclosing shape.
        # Audit every triangle in each hand patch before accepting a pose.
        if progress:
            progress(dict(stage='checking_limits', completed=refinement_index, total=refinement_total))
        # Quick fitting may approximate contacts, but acceptance always audits
        # the complete CAD against every retained representative hand patch.
        boundary_error = (triangle_penetrations(model, r, t, full_boundary_xyz, full_boundary_normals)
                          if has_limits else np.array([]))
        boundary_rmse = float(np.sqrt(np.mean(boundary_error**2))) if len(boundary_error) else 0.
        boundary_max = float(boundary_error.max()) if len(boundary_error) else 0.
        angular = direction_residual(model, r, t, directions)
        direction_rmse = float(np.sqrt(np.sum(angular**2)/len(directions[0]))) if has_directions else 0.
        score = float(np.sqrt(rmse**2+penalty**2+boundary_rmse**2+direction_rmse**2)/config.tolerance_mm)
        transform = np.eye(4)
        transform[:3, :3], transform[:3, 3] = r, t
        fits.append(dict(score=score, rmse_mm=rmse, max_error_mm=float(d.max()),
                         inlier_fraction=float(np.mean(d <= config.tolerance_mm)),
                         exclusion_rmse_mm=penalty,
                         exclusion_max_mm=float(contradiction.max()) if len(contradiction) else 0.,
                         hand_limit_rmse_mm=boundary_rmse, hand_limit_max_mm=boundary_max,
                         hand_collision_count=int(np.count_nonzero(boundary_error > config.hand_tolerance_mm)),
                         hand_constraints_checked=len(boundary_error),
                         transform=transform.tolist(), optimizer_success=bool(opt.success),
                         initialization=initialization,
                         pressure_direction_patches=len(directions[0]),
                         pressure_direction_rmse_mm=direction_rmse,
                         pressure_direction_method='pressure_weighted_local_planes_unsigned_soft',
                         refinement_method='sample_guided_local_triangles',
                         validation_method='sampled_contacts_exact_hand' if config.quick_preview else 'exact_full_mesh',
                         quick_preview=config.quick_preview,
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
    best['starts_evaluated'] = 1 if staged else len(starts)+len(preferred)
    if staged:
        best['initial_transform'] = initial_transform.tolist()
        best['placement_sequence'] = ['hand_center', 'angle', 'thumb_translation', 'refinement', 'full_hand_audit']
        best['angle_evaluations'] = int(angle_fit.nfev)
        best['translation_evaluations'] = int(slide_fit.nfev)
    if initial_pose is not None:
        best['pose_prior'] = initial_pose
        best['pose_prior_used'] = best['initialization'] == 'empirical_grasp_prior'
        best['starts_evaluated'] += int(not staged)
        if staged:
            best['placement_sequence'][0] = 'empirical_grasp_pose'
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
