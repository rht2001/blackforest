"""Periodic shaft roll, independent of bounded insertion tilt and position."""
import numpy as np
from scipy.spatial.transform import Rotation


def roll_angles(quick=False):
    """A half-open full turn, including the unchanged placement exactly once."""
    return np.arange(12 if quick else 24)*2*np.pi/(12 if quick else 24)


def axial_pose(parameters, reference, center, model_center, local_axis):
    axis = reference[:, local_axis]
    tilt = parameters[:3]-axis*np.dot(parameters[:3], axis)
    roll = np.eye(3)[local_axis]*parameters[6]
    rotation = Rotation.from_rotvec(tilt).as_matrix()@reference@Rotation.from_rotvec(roll).as_matrix()
    return rotation, center+parameters[3:6]-rotation@model_center


def screen_rolls(model, points, normals, groups, reference, center, window,
                 tolerance, *, quick=False, progress=None):
    from .visible_correspondence import visible_correspondences
    from .visible_surface import _balanced_indices, _weights
    groups=np.asarray(groups);selected=_balanced_indices(groups,72)
    p,n,g=points[selected],normals[selected],groups[selected]
    weights=_weights(g);axis=int(np.argmax(model.mesh.extents))
    model_center=model.mesh.bounds.mean(axis=0);world_axis=reference[:,axis]
    trials=[];angles=roll_angles(quick)
    for index,angle in enumerate(angles):
        for offset in (0.,-8.,8.):
            x=np.r_[np.zeros(3),np.clip(world_axis*offset,-np.asarray(window)*.9,np.asarray(window)*.9),angle]
            r,t=axial_pose(x,reference,center,model_center,axis)
            _,error,_,hit,_=visible_correspondences(model,p,n,r,t)
            if not np.all(hit):
                error[~hit]=model.nearest_sampled((p[~hit]-t)@r)[1]+2*tolerance
            scaled=np.minimum(error,30.)/tolerance
            cost=float(np.average(2*(np.sqrt(1+scaled**2)-1),weights=weights))
            trials.append((cost,x))
        if progress:
            best=min(trials[-3:],key=lambda trial:trial[0])[1]
            r,t=axial_pose(best,reference,center,model_center,axis)
            transform=np.eye(4);transform[:3,:3]=r;transform[:3,3]=t
            progress(dict(stage='screening_axial_roll',completed=index+1,total=len(angles),
                          transform=transform.tolist(),axial_roll_deg=float(np.rad2deg(angle)),preview=True))
    # Refine several angular basins, rather than filling the shortlist with
    # translations at one angle. The unchanged prior is retained by the caller.
    seeds=[]
    for cost,x in sorted(trials,key=lambda trial:trial[0]):
        if all(abs(np.arctan2(np.sin(x[6]-s[6]),np.cos(x[6]-s[6])))>=np.deg2rad(30)-1e-9 for s in seeds):
            seeds.append(x)
        if len(seeds)>=(2 if quick else 4):break
    return seeds, dict(axial_roll_search_degrees=360,axial_roll_step_degrees=30 if quick else 15,
                      axial_roll_angles_screened=len(angles),axial_roll_placements_screened=len(trials),
                      axial_roll_basins_refined=len(seeds),axial_roll_axis_local=axis,
                      axial_roll_screen_profile=[dict(angle_deg=float(np.rad2deg(angle)),
                          cost=float(min(cost for cost,x in trials if x[6]==angle))) for angle in angles])
