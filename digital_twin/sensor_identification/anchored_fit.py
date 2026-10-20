"""Local tactile-to-CAD registration near the usual in-hand placement.

Measured contacts are matched to the object, not the entire object to the sparse
contact cloud. The prior constrains placement, never establishes identity.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from .visible_surface import first_surface, _weights, _balanced_indices
from .visible_correspondence import visible_correspondences
from .finite_hand import FiniteHand
from .axial_search import axial_pose, screen_rolls


def _contact_surface(model, points, normals, rotation, translation, *, return_surface=False, exact=False):
    surface, error, face_normal, hit, _ = visible_correspondences(model, points, normals, rotation, translation, exact=exact)
    error = np.where(hit, error, np.inf)
    return (error, surface, face_normal) if return_surface else error


def fit_anchored(model, points, normals, groups, exclusions, radii, config, prior,
                 *, hand=None, progress=None):
    points, normals, groups = np.asarray(points), np.asarray(normals), np.asarray(groups)
    # Balance physical fingers, not each named patch: two isolated tip taxels
    # must not outweigh dozens of observations on the same finger's nail.
    physical_groups = np.array([str(group).split('_')[0] for group in groups])
    hand = hand or FiniteHand(reason='recorded_hand_pose_unavailable')
    r0 = Rotation.from_euler('xyz', prior['rotation_xyz_deg'], degrees=True).as_matrix()
    center = np.asarray(prior['center_mm'], dtype=float)
    model_center = model.mesh.bounds.mean(axis=0)
    bounds = np.r_[np.deg2rad([20., 20., 20.]), prior['position_window_mm'], 2*np.pi]
    selected = _balanced_indices(physical_groups, 128 if config.quick_preview else 256)
    p, n, g = points[selected], normals[selected], physical_groups[selected]
    w = _weights(g)

    def decode(x):
        return axial_pose(x, r0, center, model_center, int(np.argmax(model.mesh.extents)))

    def evidence(x, exact=False):
        r, t = decode(x)
        depth = _contact_surface(model, points, normals, r, t, exact=exact)
        hit = np.isfinite(depth)
        distance = (model.nearest if exact else model.nearest_sampled)((points-t)@r)[1]
        error = np.where(hit, abs(depth), distance+2*config.tolerance_mm)
        weights = _weights(physical_groups)
        # Huber-like tails keep a missing whole thumb influential. Capping at
        # 2*tolerance would let many good finger taxels hide that failure.
        scaled = np.minimum(error, 30.)/config.tolerance_mm
        support_cost = np.average(2.*(np.sqrt(1.+scaled**2)-1.), weights=weights)
        prior_cost = .015*np.mean((x[:6]/bounds[:6])**2)
        return float(support_cost+prior_cost), depth, hit, distance, error

    seeds = [np.zeros(7)]
    axis = r0[:, int(np.argmax(model.mesh.extents))]
    for offset in (-8., 8.):
        seeds.append(np.r_[np.zeros(3), np.clip(axis*offset, -bounds[3:6]*.9, bounds[3:6]*.9), 0.])
    for sign in (-1., 1.):
        seeds.append(np.r_[sign*np.deg2rad([0., 12., 0.]), np.zeros(3), 0.])
    if not config.quick_preview:
        for sign in (-1., 1.):
            seeds.append(np.r_[sign*np.deg2rad([12., 0., 0.]), np.zeros(3), 0.])
    roll_seeds, roll_info = screen_rolls(model, points, normals, physical_groups, r0, center,
        prior['position_window_mm'], config.tolerance_mm, quick=config.quick_preview, progress=progress)
    seeds.extend(seed for seed in roll_seeds if not any(np.allclose(seed, s) for s in seeds))
    trials = []
    for seed_number, seed in enumerate(seeds):
        x = seed.copy()
        for iteration in range(8 if config.quick_preview else 14):
            cost, *_ = evidence(x)
            trials.append((cost, x.copy()))
            r, t = decode(x)
            if progress:
                transform = np.eye(4); transform[:3, :3], transform[:3, 3] = r, t
                progress(dict(stage='refining_known_placement', completed=seed_number, total=len(seeds),
                              transform=transform.tolist(), preview=True))
            depth, surface, face_normal = _contact_surface(model, p, n, r, t, return_surface=True)
            hit = np.isfinite(depth)
            surface[~hit] = model.nearest_sampled((p[~hit]-t)@r)[0]
            face_normal[~hit] = 0.
            world_normal = face_normal@r.T
            denominator = np.maximum(.3, abs(np.sum(world_normal*n, axis=1)))

            def residual(y):
                rr, tt = decode(y)
                delta = surface@rr.T+tt-p
                plane = np.sum(delta*world_normal, axis=1)/denominator
                positional = delta*np.where(hit, .25, 1.)[:, None]
                return np.r_[plane*np.sqrt(w), (positional*np.sqrt(w[:, None])).ravel(), .15*y[:6]/bounds[:6]]

            opt = least_squares(residual, x, bounds=(-bounds, bounds), loss='soft_l1',
                                f_scale=config.tolerance_mm, max_nfev=25,
                                x_scale=[.1, .1, .1, 5., 5., 5., .25])
            if np.linalg.norm(opt.x-x) < 1e-5:
                break
            x = opt.x
        trials.append((evidence(x)[0], x.copy()))
    # Include the unchanged starting pose. A step cannot silently replace a
    # better-supported start with the optimizer's last iterate.
    trials.sort(key=lambda item: item[0])
    distinct = []
    for cost, x in trials:
        if all(np.linalg.norm(x[3:6]-other[1][3:6]) > 2. or np.linalg.norm(x[:3]-other[1][:3]) > .08 or abs(np.arctan2(np.sin(x[6]-other[1][6]), np.cos(x[6]-other[1][6]))) > .08 for other in distinct):
            distinct.append((cost, x))
        if len(distinct) >= 3:
            break
    # Contact ranking alone must not discard the usual placement before its
    # finite hand audit. Reserve each seed so a better contact-only iterate
    # cannot replace a feasible prior with a penetrating pose.
    for seed in seeds:
        if not any(np.allclose(seed, other[1], atol=1e-9, rtol=0.) for other in distinct):
            distinct.append((evidence(seed)[0], seed.copy()))
    candidates = []
    repair_samples = model.samples[np.linspace(0, len(model.samples)-1, 1200, dtype=int)]

    def repair_pose(x):
        x = x.copy()
        for _ in range(4):
            r, t = decode(x)
            ids, target, outward = hand.correction_targets(repair_samples@r.T+t, config.hand_tolerance_mm*.9)
            if not len(ids):
                break
            depth, surface, face_normal = _contact_surface(model, p, n, r, t, return_surface=True)
            hit = np.isfinite(depth)
            surface[~hit] = model.nearest_sampled((p[~hit]-t)@r)[0]
            face_normal[~hit] = 0.
            world_normal = face_normal@r.T
            denominator = np.maximum(.3, abs(np.sum(world_normal*n, axis=1)))
            collision_weight = 5.*np.sqrt(len(p)/len(ids))

            def residual(y):
                rr, tt = decode(y)
                delta = surface@rr.T+tt-p
                plane = np.sum(delta*world_normal, axis=1)/denominator
                positional = delta*np.where(hit, .25, 1.)[:, None]
                penetration = np.maximum(0., np.sum((target-(repair_samples[ids]@rr.T+tt))*outward, axis=1)-.85*config.hand_tolerance_mm)
                return np.r_[plane*np.sqrt(w), (positional*np.sqrt(w[:, None])).ravel(),
                             collision_weight*penetration, .15*y[:6]/bounds[:6]]
            opt = least_squares(residual, x, bounds=(-bounds, bounds), loss='soft_l1',
                                f_scale=config.tolerance_mm, max_nfev=30,
                                x_scale=[.1, .1, .1, 5., 5., 5., .25])
            x = opt.x
        return x

    # A bounded repair uses finite CAD escape directions, with contact terms
    # retained. Re-audit independently; a repaired pose can still be rejected.
    for candidate_index, (cost, x) in enumerate(distinct):
        r, t = decode(x)
        _, depth, hit, distance, errors = evidence(x, exact=True)
        audit = hand.audit(model, r, t, config.hand_tolerance_mm)
        if candidate_index == 0 and audit['hand_limit_max_mm'] > config.hand_tolerance_mm and hand.parts:
            repaired = repair_pose(x)
            distinct.append((evidence(repaired)[0], repaired))
        clearances = model.nearest((exclusions-t)@r)[1] if len(exclusions) else np.empty(0)
        violations = np.maximum(0., radii-clearances)
        inliers = hit & (abs(depth) <= config.tolerance_mm)
        transform = np.eye(4); transform[:3, :3], transform[:3, 3] = r, t
        normal_ray = first_surface(model, points, normals, r, t)
        weights = _weights(physical_groups)
        region_support = {str(label): float(np.mean(inliers[physical_groups == label]))
                          for label in dict.fromkeys(physical_groups)}
        weak_regions = [str(label) for label in dict.fromkeys(physical_groups)
                        if np.sum(physical_groups == label) >= 6 and region_support[str(label)] < .5]
        balanced = float(np.sqrt(np.average(np.minimum(errors, 30.)**2, weights=weights)))
        reasons = []
        if not np.all(inliers):
            reasons.append('visible_contact_mismatch')
        if not hand.parts or hand.skipped:
            reasons.append('hand_volume_partially_validated')
        if audit['hand_limit_max_mm'] > config.hand_tolerance_mm:
            reasons.append('finite_hand_penetration')
        if len(violations) and violations.max() > config.tolerance_mm:
            reasons.append('certified_clearance_violated')
        fit = dict(transform=transform.tolist(), score=cost+audit['hand_limit_rmse_mm']/config.hand_tolerance_mm,
            rmse_mm=float(np.sqrt(np.mean(distance**2))), max_error_mm=float(distance.max()),
            inlier_fraction=float(np.mean(distance <= config.tolerance_mm)),
            visible_surface_inlier_fraction=float(inliers.mean()), visible_surface_hit_fraction=float(hit.mean()),
            visible_surface_score_mm=balanced, contact_metric='nearest_visible_surface',
            normal_ray_inlier_fraction=float(np.mean(np.isfinite(normal_ray)&(abs(normal_ray)<=config.tolerance_mm))),
            physical_contact_region_support=region_support, weak_contact_regions=weak_regions,
            placement_quality='partial_contact_support' if weak_regions else 'supported_contacts',
            visible_surface_rmse_mm=float(np.sqrt(np.mean(depth[hit]**2))) if hit.any() else None,
            visible_surface_max_mm=float(abs(depth[hit]).max()) if hit.any() else None,
            visible_surface_region_stats={str(label): dict(count=int(np.sum(groups == label)),
                inlier_fraction=float(np.mean(inliers[groups == label])),
                median_error_mm=float(np.median(errors[groups == label]))) for label in dict.fromkeys(groups)},
            contact_fit_compatible=bool(np.all(inliers)), compatible=False,
            placement_usable=bool(hand.parts and np.mean(inliers) >= .5
                and audit['hand_limit_max_mm'] <= config.hand_tolerance_mm
                and (not len(violations) or violations.max() <= config.tolerance_mm)),
            exclusion_max_mm=float(violations.max()) if len(violations) else 0.,
            exclusion_rmse_mm=float(np.sqrt(np.mean(violations**2))) if len(violations) else 0.,
            **audit, rejection_reasons=reasons, quick_preview=config.quick_preview,
            pose_evidence='known_placement_finite_cad', search_method='bounded_contact_to_visible_cad',
            search_pass='known_placement', validation_method='exact_contact_rays_and_sampled_closed_hand_volumes',
            prior_rotation_delta_deg=float(np.rad2deg(np.linalg.norm(x[:3]))),
            axial_roll_deg=float(np.rad2deg(x[6]) % 360), **roll_info,
            prior_center_delta_mm=x[3:6].tolist(), prior_orientation_component_bounds_deg=[20., 20., 20.],
            initialization='recorded_grasp_prior', optimizer_success=bool(opt.success),
            input_contact_points=len(points), scored_contact_points=len(points),
            starts_evaluated=len(seeds), refinements=len(seeds), consistent_refinements=1,
            prior_seeds_audited=len(seeds),
            placement_interpretation='Provisional pose near usual insertion; contact outliers, open hand links and sampled collision coverage remain explicit. Identity is unconfirmed.')
        candidates.append(fit)
    # Real finite penetration takes precedence over small contact improvements.
    best = min(candidates, key=lambda f: (not f['placement_usable'], f['score']))
    return best
