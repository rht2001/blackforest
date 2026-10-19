"""STL geometry in millimetres; never infer scale from an observed grasp."""
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from scipy.spatial import cKDTree
import trimesh


@dataclass
class CandidateModel:
    name: str
    mesh: trimesh.Trimesh
    sample_count: int = 24000
    samples: np.ndarray = field(init=False, repr=False)
    sample_faces: np.ndarray = field(init=False, repr=False)
    tree: cKDTree = field(init=False, repr=False)

    def __post_init__(self):
        self.mesh = self.mesh.copy()
        if (not len(self.mesh.faces) or not np.isfinite(self.mesh.vertices).all()
                or self.mesh.area <= 0 or self.sample_count < 100):
            raise ValueError('Expected a finite, nonempty triangular surface')
        self.samples, self.sample_faces = trimesh.sample.sample_surface(
            self.mesh, self.sample_count, seed=1701)
        self.tree = cKDTree(self.samples)

    def nearest_sampled(self, points, neighbors=8, *, return_normals=False):
        """Fast local surface query guided by cached STL samples.

        Project onto triangles attached to nearby samples. This avoids fitting
        sampling gaps and does not build global point/triangle candidate lists.
        It is approximate: final acceptance still uses nearest() and the full
        hand-patch triangle audit.
        """
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        closest = np.empty_like(points)
        distance = np.empty(len(points))
        normals = np.empty_like(points) if return_normals else None
        k = min(neighbors, len(self.samples))
        triangles = self.mesh.triangles
        for start in range(0, len(points), 256):
            query = points[start:start+256]
            indices = self.tree.query(query, k=k)[1].reshape(len(query), k)
            faces = self.sample_faces[indices].ravel()
            repeated = np.repeat(query, k, axis=0)
            projected = trimesh.triangles.closest_point(triangles[faces], repeated).reshape(-1, k, 3)
            squared = np.sum((projected-query[:, None, :])**2, axis=2)
            best = squared.argmin(axis=1)
            rows = np.arange(len(query))
            closest[start:start+len(query)] = projected[rows, best]
            distance[start:start+len(query)] = np.sqrt(squared[rows, best])
            if return_normals:
                normals[start:start+len(query)] = self.mesh.face_normals[self.sample_faces[indices[rows, best]]]
        return (closest, distance, normals) if return_normals else (closest, distance)

    @classmethod
    def from_stl(cls, name, path, *, units):
        scale = {'mm': 1.0, 'm': 1000.0}.get(units)
        if scale is None:
            raise ValueError('STL units must explicitly be mm or m')
        mesh = trimesh.load_mesh(path, process=True)
        mesh.apply_scale(scale)
        return cls(name, mesh)

    def nearest(self, points):
        """Exact unsigned triangle distance: valid for open/assembly meshes too."""
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        closest = np.empty_like(points)
        distance = np.empty(len(points))
        # An on-surface sample supplies a conservative upper distance bound.
        # Every potentially nearer triangle intersects that query-centered box.
        # Reuse the cached sample KD-tree and triangle R-tree instead of
        # rebuilding a vertex KD-tree inside trimesh.closest_point per batch.
        upper = self.tree.query(points)[0]+1e-7
        tree = self.mesh.triangles_tree
        triangles = self.mesh.triangles
        for start in range(0, len(points), 32):
            query = points[start:start+32]
            radius = upper[start:start+32, None]
            lower, higher = query-radius, query+radius
            candidates = [np.asarray(list(tree.intersection(np.r_[lo, hi])), dtype=int)
                          for lo, hi in zip(lower, higher)]
            for offset, (point, faces) in enumerate(zip(query, candidates)):
                if not len(faces):
                    # Numerical fallback, retaining exact full-mesh semantics.
                    p, d, _ = trimesh.proximity.closest_point(self.mesh, point[None, :])
                    closest[start+offset], distance[start+offset] = p[0], d[0]
            # Batch across nearby queries too, while bounding the large
            # projection arrays for distant queries. Never discard faces.
            faces = np.concatenate(candidates)
            query_ids = np.repeat(np.arange(len(query)), [len(c) for c in candidates])
            best_squared = np.full(len(query), np.inf)
            for first in range(0, len(faces), 16384):
                ids = query_ids[first:first+16384]
                batch = triangles[faces[first:first+16384]]
                projected = trimesh.triangles.closest_point(batch, query[ids])
                squared = np.sum((projected-query[ids])**2, axis=1)
                groups, offsets = np.unique(ids, return_index=True)
                stops = np.r_[offsets[1:], len(ids)]
                for group, low, high in zip(groups, offsets, stops):
                    best = low+int(squared[low:high].argmin())
                    if squared[best] < best_squared[group]:
                        best_squared[group] = squared[best]
                        closest[start+group] = projected[best]
            present = np.isfinite(best_squared)
            distance[start:start+len(query)][present] = np.sqrt(best_squared[present])
        return closest, distance

    def report(self):
        return dict(name=self.name, units='mm', bounds_mm=self.mesh.bounds.tolist(),
                    extents_mm=self.mesh.extents.tolist(), faces=len(self.mesh.faces),
                    vertices=len(self.mesh.vertices), watertight=bool(self.mesh.is_watertight),
                    winding_consistent=bool(self.mesh.is_winding_consistent),
                    degenerate_faces=int((self.mesh.area_faces < 1e-12).sum()))


def load_candidates(root=None):
    root = Path(root) if root else Path(__file__).resolve().parents[2]
    # STL is unitless. A's companion STEP declares millimetres. B uses the
    # same export convention; its physical dimensions still need confirmation.
    return {name: CandidateModel.from_stl(name, root / 'documents' / filename, units='mm')
            for name, filename in [('SENSOR_A', 'VEGABAR_29_STL.stl'),
                                   ('SENSOR_B', 'VEGABAR_38.stl')]}
