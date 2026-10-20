"""Pose-independent contradictions in the existing local hand-patch model."""
import numpy as np


def contact_hand_conflicts(contacts, points, normals, contact_tolerance_mm,
                          hand_tolerance_mm, radius_mm=4.):
    """Certify contact balls wholly inside a forbidden inward patch prism.

    For any fitting pose, each contact needs a CAD triangle within its radius
    `contact_tolerance_mm`. If that entire ball lies inside one patch and
    deeper than the allowed hand penetration, no rigid pose of ANY CAD can
    satisfy both checks. This is a certificate about the approximate input
    constraints, not proof that the physical measurements are wrong.
    """
    empty = dict(contact_indices=[], boundary_indices=[], witnesses=[],
                 contact_count=0, boundary_count=0)
    margin = radius_mm-contact_tolerance_mm
    if points is None or not len(points) or not len(contacts) or margin <= 0:
        return empty
    points = np.asarray(points, dtype=float)
    normals = np.asarray(normals, dtype=float)
    contacts = np.asarray(contacts, dtype=float)
    helper = np.eye(3)[np.argmin(np.abs(normals), axis=1)]
    u = np.cross(normals, helper)
    u /= np.linalg.norm(u, axis=1)[:, None]
    v = np.cross(normals, u)
    contact_mask = np.zeros(len(contacts), dtype=bool)
    boundary_mask = np.zeros(len(points), dtype=bool)
    witnesses = []
    for first in range(0, len(contacts), 32):
        delta = contacts[first:first+32, None, :]-points[None, :, :]
        x = np.einsum('ijk,jk->ij', delta, u)
        y = np.einsum('ijk,jk->ij', delta, v)
        depth = -np.einsum('ijk,jk->ij', delta, normals)
        bad = ((np.abs(x) < margin-1e-8) & (np.abs(y) < margin-1e-8)
               & (depth > contact_tolerance_mm+hand_tolerance_mm+1e-8))
        contact_mask[first:first+len(delta)] = bad.any(axis=1)
        boundary_mask |= bad.any(axis=0)
        # One deepest witness per contradictory patch, bounded report size.
        for boundary in np.flatnonzero(bad.any(axis=0)):
            local = int(np.argmax(np.where(bad[:, boundary], depth[:, boundary], -np.inf)))
            witnesses.append(dict(contact_index=first+local, boundary_index=int(boundary),
                                  depth_mm=float(depth[local, boundary]),
                                  minimum_required_penetration_mm=float(depth[local, boundary]-contact_tolerance_mm)))
    witnesses.sort(key=lambda p: (-p['depth_mm'], p['contact_index'], p['boundary_index']))
    return dict(contact_indices=np.flatnonzero(contact_mask).tolist(),
                boundary_indices=np.flatnonzero(boundary_mask).tolist(),
                witnesses=witnesses[:32], contact_count=int(contact_mask.sum()),
                boundary_count=int(boundary_mask.sum()),
                contact_tolerance_mm=float(contact_tolerance_mm),
                hand_tolerance_mm=float(hand_tolerance_mm), patch_radius_mm=float(radius_mm),
                interpretation='No CAD pose can satisfy these contact balls and the supplied inward hand prisms simultaneously.')
