"""Conservative local contact planes, in the observation frame and millimetres."""
import numpy as np
from scipy.spatial import cKDTree


def pressure_directions(points, pressures, threshold):
    """Pressure-weighted PCA supplies unoriented normals, never a force vector.

    Only locally planar, two-dimensional patches contribute. Sparse points,
    lines, and corners are deliberately ignored. A patch cannot determine the
    sign of its normal or rotation about that normal.
    """
    points = np.asarray(points, dtype=float).reshape(-1, 3)
    empty = (np.empty((0, 3)), np.empty((0, 3)), np.empty(0))
    if pressures is None or len(points) < 6:
        return empty
    # Repeated observations must not invent independent spatial support.
    unique, inverse = np.unique(points, axis=0, return_inverse=True)
    weights = np.zeros(len(unique))
    np.maximum.at(weights, inverse, np.maximum(0., np.asarray(pressures)-threshold))
    points = unique
    if len(points) < 6 or not np.any(weights > 0):
        return empty
    weights = np.minimum(weights, np.median(weights[weights > 0])*3.)
    tree = cKDTree(points)
    centers, normals, strengths = [], [], []
    for index in np.linspace(0, len(points)-1, min(128, len(points)), dtype=int):
        distances, neighbors = tree.query(points[index], k=min(16, len(points)))
        neighbors = neighbors[distances <= 12.]
        if len(neighbors) < 6:
            continue
        w = weights[neighbors]
        if w.sum() <= 0:
            continue
        center = np.average(points[neighbors], axis=0, weights=w)
        delta = points[neighbors]-center
        values, vectors = np.linalg.eigh((delta*w[:, None]).T@delta/w.sum())
        # At least 1 mm of independent width; reject curved/volumetric groups.
        if values[1] < 1. or values[0] > .12*values[1]:
            continue
        centers.append(center)
        normals.append(vectors[:, 0])
        strengths.append(min(3., np.sqrt(values[1]))*(1.-values[0]/(.12*values[1])))
    return (np.asarray(centers).reshape(-1, 3), np.asarray(normals).reshape(-1, 3),
            np.asarray(strengths))


def direction_residual(model, rotation, translation, directions):
    centers, normals, strengths = directions
    if not len(centers):
        return np.empty(0)
    local = (centers-translation)@rotation
    _, _, surface_normals = model.nearest_sampled(local, return_normals=True)
    # Cross product is invariant to STL winding / PCA sign and is smooth for
    # small rotations. Strength is a bounded mm lever arm, not a hard gate.
    return (np.cross(normals@rotation, surface_normals)*strengths[:, None]).ravel()
