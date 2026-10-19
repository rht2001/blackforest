"""Fit the first CAD surface seen along the outward taxel direction.

Unsigned nearest distance can match an interior wall or the far side of a
housing. Ray entry is independent of assembly winding and audits visibility.
No sensor name, run label or recorded reference pose is used here.
"""
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation


def first_surface(model, points, normals, rotation, translation, *, return_surface=False):
    local = (points-translation)@rotation
    directions = normals@rotation
    # Start beyond the complete CAD envelope, even for an off-center pose.
    travel = np.linalg.norm(local-model.mesh.bounds.mean(axis=0), axis=1)+np.linalg.norm(model.mesh.extents)+1.
    origins = local-directions*travel[:, None]
    faces, rays, locations = model.mesh.ray.intersects_id(origins, directions, multiple_hits=True, return_locations=True)
    locations = np.asarray(locations).reshape(-1,3)
    depths = np.full(len(points), np.inf)
    np.minimum.at(depths, rays, np.einsum('ij,ij->i', locations-origins[rays], directions[rays])-travel[rays])
    if not return_surface:
        return depths
    selected = np.full(len(points), -1, dtype=int)
    values = np.einsum('ij,ij->i', locations-origins[rays], directions[rays])-travel[rays]
    # Every ray retains its first triangle, with deterministic ties.
    for i in np.argsort(values)[::-1]:
        selected[rays[i]]=i
    hit=selected>=0
    surface=np.empty_like(points);face_normals=np.empty_like(points)
    surface[hit]=locations[selected[hit]]
    face_normals[hit]=model.mesh.face_normals[faces[selected[hit]]]
    return depths, surface, face_normals


def _balanced_indices(groups, budget):
    result = []
    labels = list(dict.fromkeys(groups))
    for label in labels:
        ids = np.flatnonzero(groups == label)
        result.extend(ids[np.linspace(0, len(ids)-1, min(len(ids), max(3, budget//len(labels))), dtype=int)])
    return np.asarray(result, dtype=int)


def _weights(groups):
    weights = np.array([1./np.count_nonzero(groups == g) for g in groups])
    return weights/weights.mean()


def fit_visible_surface(model, points, normals, groups, config, progress=None):
    points, normals = np.asarray(points), np.asarray(normals)
    groups = np.asarray(groups)
    weights = _weights(groups)
    # Finger group centers describe the shaft direction without counting a
    # dense palm patch as hundreds of independent axial observations.
    labels = [g for g in dict.fromkeys(groups) if not g.startswith(('thumb', 'palm'))]
    centers = np.array([np.median(points[groups == g], axis=0) for g in labels])
    support = centers if len(centers) >= 3 else points
    _, _, axes = np.linalg.svd(support-support.mean(axis=0), full_matrices=False)
    axis = axes[0]
    helper = np.eye(3)[np.argmin(abs(axis))]
    u = np.cross(axis, helper); u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    center = np.average(points, axis=0, weights=weights)
    # Algebraic circle in the transverse plane supplies a cheap shaft seed.
    radial = np.column_stack((points@u, points@v))
    mask = np.array([not g.startswith('thumb') for g in groups])
    a = np.column_stack((2*radial[mask], np.ones(mask.sum())))
    w = np.sqrt(weights[mask])
    circle = np.linalg.lstsq(a*w[:, None], np.sum(radial[mask]**2, axis=1)*w, rcond=None)[0]
    center += (circle[0]-center@u)*u+(circle[1]-center@v)*v
    local_axis = int(np.argmax(model.mesh.extents))
    other = [i for i in range(3) if i != local_axis]
    idx = _balanced_indices(groups, 36)

    def residual(r, t, selected):
        p, n = points[selected], normals[selected]
        w = _weights(groups[selected])
        depth = first_surface(model, p, n, r, t)
        closest, distance = model.nearest_sampled((p-t)@r)
        # Misses retain a positional gradient; they are never treated as a
        # successful ray observation in the final audit.
        ray = np.where(np.isfinite(depth), depth, distance+6.)
        return np.r_[ray*np.sqrt(w), .15*((closest@r.T+t-p)*np.sqrt(w[:,None])).ravel()]

    starts = []
    for sign in (-1., 1.):
        for roll in np.linspace(0, 2*np.pi, 8, endpoint=False):
            r = np.zeros((3,3));r[:,local_axis]=sign*axis
            r[:,other[0]]=np.cos(roll)*u+np.sin(roll)*v
            r[:,other[1]]=np.cross(r[:,local_axis],r[:,other[0]])
            if np.linalg.det(r)<0:r[:,other[1]]*=-1
            # Sweep axial placement instead of assuming contact centroid is
            # the CAD center. Grasped shafts are often near one end.
            lo,hi=model.mesh.bounds[:,local_axis]
            for anchor in np.linspace(lo+.15*(hi-lo),hi-.15*(hi-lo),5):
                local_center=np.zeros(3);local_center[local_axis]=anchor
                t=center-r@local_center
                e=residual(r,t,idx);cost=np.mean(9*(np.sqrt(1+(e/3)**2)-1))
                starts.append((cost,r,t))
    starts.sort(key=lambda item:item[0])
    finalists=[]
    for _,r,t in starts:
        if any(np.linalg.norm(t-tt)<10 and Rotation.from_matrix(r@rr.T).magnitude()<.5 for _,rr,tt in finalists):continue
        finalists.append((_,r,t))
        if len(finalists)>= (2 if config.quick_preview else 5):break
    fits=[]
    for number,(_,r0,t0) in enumerate(finalists):
        if progress:progress(dict(stage='visible_surface_refining',completed=number,total=len(finalists)))
        selected=_balanced_indices(groups,72)
        r,t=r0.copy(),t0.copy();evaluations=0
        for iteration in range(5 if config.quick_preview else 14):
            if not config.quick_preview and iteration == 10:
                selected=np.arange(len(points))
            p,n=points[selected],normals[selected]
            w=_weights(groups[selected])
            depth,surface,face_normal=first_surface(model,p,n,r,t,return_surface=True)
            hit=np.isfinite(depth)
            near=model.nearest_sampled((p-t)@r)[0]
            surface[~hit]=near[~hit]
            face_normal[~hit]=0
            # Freeze correspondences during this small least-squares step.
            # The expensive visibility rays are recomputed only between steps.
            denominator=np.maximum(.25,abs(np.sum((face_normal@r.T)*n,axis=1)))
            def decode(parameters):return Rotation.from_rotvec(parameters[:3]).as_matrix()@r,t+parameters[3:]
            def fixed_residual(parameters):
                rr,tt=decode(parameters);delta=surface@rr.T+tt-p
                plane=np.sum(delta*(face_normal@rr.T),axis=1)/denominator
                positional=delta*np.where(hit,.15,1.)[:,None]
                return np.r_[plane*np.sqrt(w),(positional*np.sqrt(w[:,None])).ravel()]
            opt=least_squares(fixed_residual,np.zeros(6),loss='soft_l1',f_scale=3.,max_nfev=20,
                              x_scale=np.array([.1,.1,.1,5.,5.,5.]),
                              bounds=(np.r_[[-.12]*3,[-5.]*3],np.r_[[.12]*3,[5.]*3]))
            r,t=decode(opt.x);evaluations+=opt.nfev
            if progress:
                live=np.eye(4);live[:3,:3]=r;live[:3,3]=t
                progress(dict(stage='visible_surface_refining',completed=number,total=len(finalists),
                              transform=live.tolist(),pose_step=number*20+iteration+1,preview=True))
            if np.linalg.norm(opt.x)<1e-4:break
        depth=first_surface(model,points,normals,r,t)
        distance=model.nearest((points-t)@r)[1]
        hit=np.isfinite(depth);errors=np.where(hit,abs(depth),distance+6.)
        balanced=float(np.sum(weights*np.minimum(errors,30.)**2)/np.sum(weights))
        transform=np.eye(4);transform[:3,:3]=r;transform[:3,3]=t
        fits.append(dict(transform=transform.tolist(),score=float(np.sqrt(balanced)/config.tolerance_mm),
                         rmse_mm=float(np.sqrt(np.mean(distance**2))),max_error_mm=float(distance.max()),
                         visible_surface_score_mm=float(np.sqrt(balanced)),
                         visible_surface_rmse_mm=float(np.sqrt(np.average(depth[hit]**2,weights=weights[hit]))) if np.any(hit) else None,
                         visible_surface_max_mm=float(np.max(abs(depth[hit]))) if np.any(hit) else None,
                         visible_surface_hit_fraction=float(np.mean(hit)),
                         visible_surface_inlier_fraction=float(np.mean(hit&(abs(depth)<=config.tolerance_mm))),
                         visible_surface_region_stats={str(g):dict(hit_fraction=float(np.mean(hit[groups==g])),
                             inlier_fraction=float(np.mean((hit&(abs(depth)<=config.tolerance_mm))[groups==g])),
                             median_error_mm=float(np.median(errors[groups==g]))) for g in dict.fromkeys(groups)},
                         inlier_fraction=float(np.mean(distance<=config.tolerance_mm)),
                         compatible=False,contact_fit_compatible=bool(np.all(hit)&np.all(abs(depth)<=config.tolerance_mm)),
                         hand_limit_max_mm=0.,hand_limit_rmse_mm=0.,hand_collision_count=0,hand_constraints_checked=0,
                         hand_validation_complete=False,optimizer_success=bool(opt.success),evaluations=int(evaluations),
                         contact_tolerance_mm=config.tolerance_mm,hand_tolerance_mm=config.hand_tolerance_mm,
                         rejection_reasons=['hand_volume_not_validated']+(['visible_contact_mismatch'] if not np.all(hit&(abs(depth)<=config.tolerance_mm)) else []),
                         search_method='balanced_first_visible_surface_multistart',search_pass='visible_surface',
                         score_basis='region_balanced_first_surface_error_capped_30mm_misses_nearest_plus_6mm',pose_evidence='oriented_surface_preview',
                         validation_method='full_mesh_first_ray_intersection_and_exact_distance',
                         refinements=len(finalists),consistent_refinements=1,scored_contact_points=len(points),
                         input_contact_points=len(points),starts_evaluated=len(starts),quick_preview=config.quick_preview))
    return min(fits,key=lambda f:f['score'])


def fit_conflicted_surface(model, contacts, exclusions, radii, config, *,
                           contact_normals=None, contact_groups=None,
                           boundary_xyz=None, boundary_normals=None, progress=None):
    """Replace an infeasible unsigned preview only when observed normals exist.

    Consistent hand constraints and older position-only callers retain their
    existing solver and complete collision validation.
    """
    if contact_normals is None or contact_groups is None or len(set(contact_groups))<3:
        return None
    from .evidence import contact_hand_conflicts
    from .limits import triangle_penetrations
    conflict=contact_hand_conflicts(contacts,boundary_xyz,boundary_normals,config.tolerance_mm,config.hand_tolerance_mm)
    if not conflict['boundary_count']:
        return None
    fit=fit_visible_surface(model,contacts,contact_normals,contact_groups,config,progress)
    transform=np.asarray(fit['transform']);r,t=transform[:3,:3],transform[:3,3]
    ids=np.asarray(conflict['boundary_indices'],dtype=int)
    depth=triangle_penetrations(model,r,t,np.asarray(boundary_xyz)[ids],np.asarray(boundary_normals)[ids])
    contradiction=np.maximum(0.,radii-model.nearest((exclusions-t)@r)[1]) if len(exclusions) else np.empty(0)
    clearance=float(np.sqrt(np.mean(contradiction**2))) if len(contradiction) else 0.
    fit.update(geometry_conflict=conflict,input_hand_points=len(boundary_xyz),
               hand_limit_rmse_mm=float(np.sqrt(np.mean(depth**2))),hand_limit_max_mm=float(depth.max()),
               hand_limit_max_is_lower_bound=True,hand_collision_count=int(np.count_nonzero(depth>config.hand_tolerance_mm)),
               hand_constraints_checked=len(ids),exclusion_rmse_mm=clearance,
               exclusion_max_mm=float(contradiction.max()) if len(contradiction) else 0.,
               point_spacing_mm=config.point_spacing_mm)
    fit['score']=float(np.hypot(fit['score'],clearance/config.tolerance_mm))
    fit['rejection_reasons'].append('input_geometry_conflict')
    if len(contradiction) and contradiction.max()>config.tolerance_mm:
        fit['rejection_reasons'].append('certified_clearance_violated')
    return fit
