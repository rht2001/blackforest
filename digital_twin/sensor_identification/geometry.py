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

    def nearest_sampled(self, points, neighbors=8):
        """Fast local surface query guided by cached STL samples.

        Project onto triangles attached to nearby samples. This avoids fitting
        sampling gaps and does not build global point/triangle candidate lists.
        It is approximate: final acceptance still uses nearest() and the full
        hand-patch triangle audit.
        """
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        closest = np.empty_like(points)
        distance = np.empty(len(points))
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
        return closest, distance

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
        # Far-away hand limits can each visit many CAD triangles. Bound
        # temporary point/triangle arrays without dropping any observations.
        points = np.asarray(points, dtype=float).reshape(-1, 3)
        closest = np.empty_like(points)
        distance = np.empty(len(points))
        for start in range(0, len(points), 32):
            stop = start+32
            closest[start:stop], distance[start:stop], _ = trimesh.proximity.closest_point(
                self.mesh, points[start:stop])
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
