"""Nearest CAD surfaces visible from the hand side of an observed taxel."""
import numpy as np
from .ray_bvh import cached_bvh


def visible_correspondences(model,points,normals,rotation,translation,*,exact=False):
    local=(np.asarray(points)-translation)@rotation
    directions=np.asarray(normals)@rotation
    nearest,distance=(model.nearest(local) if exact else model.nearest_sampled(local))
    travel=np.linalg.norm(local-model.mesh.bounds.mean(axis=0),axis=1)+np.linalg.norm(model.mesh.extents)+1.
    origins=local-directions*travel[:,None]
    # A taxel observes a finite positional tolerance ball, not an infinitely
    # thin force ray. Aim at the nearest actual triangle point. Its occluding
    # first surface must still be used, so far walls remain invalid matches.
    rays=nearest-origins
    lengths=np.linalg.norm(rays,axis=1)
    rays/=lengths[:,None]
    first,faces=cached_bvh(model).first(origins,rays,max_distance=lengths+1e-7)
    hit=faces>=0
    surface=nearest.copy();surface[hit]=origins[hit]+rays[hit]*first[hit,None]
    errors=np.linalg.norm(surface-local,axis=1)
    errors[~hit]=distance[~hit]+6.
    face_normals=np.zeros_like(surface);face_normals[hit]=model.mesh.face_normals[faces[hit]]
    occlusion=np.linalg.norm(surface-nearest,axis=1)
    return surface,errors,face_normals,hit,occlusion
