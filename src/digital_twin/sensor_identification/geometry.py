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
    tree: cKDTree = field(init=False, repr=False)

    def __post_init__(self):
        self.mesh = self.mesh.copy()
        if (not len(self.mesh.faces) or not np.isfinite(self.mesh.vertices).all()
                or self.mesh.area <= 0 or self.sample_count < 100):
            raise ValueError('Expected a finite, nonempty triangular surface')
        self.samples, _ = trimesh.sample.sample_surface(
            self.mesh, self.sample_count, seed=1701)
        self.tree = cKDTree(self.samples)

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
        closest, distance, _ = trimesh.proximity.closest_point(self.mesh, points)
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
