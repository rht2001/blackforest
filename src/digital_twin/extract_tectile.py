"""extract_tactile.py

Non-invasive extraction of kinematic link frames, local taxels,
and unified base-frame taxels from build_tactile_geometry output.
"""
from dataclasses import dataclass
from pathlib import Path
import json
import re
from typing import Union
import numpy as np
from numpy.typing import NDArray

# Reuse unchanged functions and constants from your original module
from digital_twin.build_tactile_geometry import build, origin, PROFILE, ROOT


# ==========================================
# Type Definitions
# ==========================================

@dataclass
class LocalTaxel:
    """Taxel geometry expressed in its own local sensor link coordinate frame."""
    key: str
    link: str
    wire_index: int
    row: int
    col: int
    position: NDArray[np.float64]  # (3,), in meters [x, y, z]
    normal: NDArray[np.float64]    # (3,), unit surface normal


@dataclass
class BaseTaxel:
    """Taxel geometry transformed into the global hand frame (left_base_link)."""
    key: str
    link: str
    wire_index: int
    row: int
    col: int
    position: NDArray[np.float64]  # (3,), in meters [x, y, z]
    normal: NDArray[np.float64]    # (3,), unit surface normal


@dataclass
class TactileHandKinematics:
    """Complete kinematic frames and taxel geometry."""
    # 4x4 SE(3) transform: link frame -> left_base_link frame
    link_frames_base: dict[str, NDArray[np.float64]]

    # 4x4 SE(3) transform: link frame -> root URDF frame
    link_frames_root: dict[str, NDArray[np.float64]]

    # 1,062 taxels in local sensor link frames
    local_taxels: list[LocalTaxel]

    # 1,062 taxels in left_base_link frame
    base_taxels: list[BaseTaxel]

    @property
    def base_positions_array(self) -> NDArray[np.float64]:
        """Convenience array of shape (1062, 3) for vectorized spatial computation."""
        return np.array([t.position for t in self.base_taxels])

    @property
    def base_normals_array(self) -> NDArray[np.float64]:
        """Convenience array of shape (1062, 3) for vectorized normal vectors."""
        return np.array([t.normal for t in self.base_taxels])


# ==========================================
# Extraction Function
# ==========================================

def compute_link_frames(
    data: dict,
    profile: str = PROFILE
) -> tuple[dict[str, NDArray[np.float64]], dict[str, NDArray[np.float64]]]:
    """Computes FK for all links relative to root and relative to left_base_link."""
    p = data['reference']['primary_profiles'][profile]
    frames_root: dict[str, NDArray[np.float64]] = {p['root_link']: np.eye(4)}
    pending = p['joints'].copy()
    while pending:
        j = next(j for j in pending if j['parent'] in frames_root)
        frames_root[j['child']] = frames_root[j['parent']] @ origin(j['origin_xyz_m'], j['origin_rpy_rad'])
        pending.remove(j)

    base_inv = np.linalg.inv(frames_root['left_base_link'])
    frames_base = {link: base_inv @ tf for link, tf in frames_root.items()}
    return frames_base, frames_root


def extract_tactile_kinematics(
    source: Union[str, Path, dict, None] = None,
    geometry: Union[dict, None] = None
) -> TactileHandKinematics:
    """Extracts link frames, local taxels, and base-frame taxels.

    Parameters:
        source: Optional path to hand_twin_6dof.html or pre-parsed data dict.
                Defaults to loading hand_twin_6dof.html from the repo ROOT.
        geometry: Optional pre-built geometry dict (from build(data)).
                  If None, build(data) will be called.
    """
    if source is None:
        html = (ROOT / 'hand_twin_6dof.html').read_text(encoding='utf-8')
        data = json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>', html, re.S).group(1))
    elif isinstance(source, (str, Path)):
        html = Path(source).read_text(encoding='utf-8')
        data = json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>', html, re.S).group(1))
    else:
        data = source

    # Compute FK frames
    frames_base, frames_root = compute_link_frames(data)

    # Use original, untouched build()
    if geometry is None:
        geometry = build(data)

    local_taxels: list[LocalTaxel] = []
    base_taxels: list[BaseTaxel] = []

    for region in geometry['regions']:
        key = region['key']
        link = region['link']
        link_to_base = frames_base[link]

        for c in region['cells']:
            p_local = np.array(c['point_m'], dtype=np.float64)
            n_local = np.array(c['normal'], dtype=np.float64)

            # Transform into left_base_link frame: T * [x, y, z, 1]^T
            p_base = (link_to_base @ np.append(p_local, 1.0))[:3]
            # Rotate normal vector by the 3x3 rotation component
            n_base = link_to_base[:3, :3] @ n_local
            n_base /= np.linalg.norm(n_base)

            local_taxels.append(LocalTaxel(
                key=key,
                link=link,
                wire_index=c['wire_index'],
                row=c['row'],
                col=c['col'],
                position=p_local,
                normal=n_local
            ))

            base_taxels.append(BaseTaxel(
                key=key,
                link=link,
                wire_index=c['wire_index'],
                row=c['row'],
                col=c['col'],
                position=p_base,
                normal=n_base
            ))

    return TactileHandKinematics(
        link_frames_base=frames_base,
        link_frames_root=frames_root,
        local_taxels=local_taxels,
        base_taxels=base_taxels
    )


def reframe_to_palm_taxel(
    kinematics: Union[TactileHandKinematics, None] = None,
    row: int = 0,
    col: int = 7,
    align_orientation: bool = True
) -> TactileHandKinematics:
    """Re-roots all kinematics and taxels so the center-top palm taxel is the origin (0,0,0).

    Can be called with zero arguments. If kinematics is omitted, it automatically
    loads and parses hand_twin_6dof.html from the repository root.
    """
    # Auto-load kinematics if not explicitly provided
    if kinematics is None:
        kinematics = extract_tactile_kinematics()

    # 1. Locate the target palm taxel
    palm_taxels = [t for t in kinematics.base_taxels if t.key == 'palm']
    target = next((t for t in palm_taxels if t.row == row and t.col == col), None)

    if target is None:
        raise ValueError(f"Palm taxel at row={row}, col={col} not found.")

    origin_base = target.position
    normal_base = target.normal

    # 2. Build T_base_target (target frame relative to left_base_link)
    t_base_target = np.eye(4, dtype=np.float64)
    t_base_target[:3, 3] = origin_base

    if align_orientation:
        # Z-axis = surface normal (pointing straight out from the palm)
        z_axis = normal_base / np.linalg.norm(normal_base)

        # Tangent vector pointing up toward fingers
        ref_up = np.array([0.0, 1.0, 0.0], dtype=np.float64)
        if abs(np.dot(ref_up, z_axis)) > 0.95:
            ref_up = np.array([0.0, 0.0, 1.0], dtype=np.float64)

        # Orthonormalize basis (Gram-Schmidt)
        y_axis = ref_up - np.dot(ref_up, z_axis) * z_axis
        y_axis /= np.linalg.norm(y_axis)
        x_axis = np.cross(y_axis, z_axis)
        x_axis /= np.linalg.norm(x_axis)
        y_axis = np.cross(z_axis, x_axis)

        t_base_target[:3, :3] = np.column_stack([x_axis, y_axis, z_axis])

    # 3. Invert SE(3): T_target_base = [R^T | -R^T * p]
    r_mat = t_base_target[:3, :3]
    p_vec = t_base_target[:3, 3]
    t_target_base = np.eye(4, dtype=np.float64)
    t_target_base[:3, :3] = r_mat.T
    t_target_base[:3, 3] = -r_mat.T @ p_vec

    # 4. Transform kinematic link frames
    reframed_links = {
        link: t_target_base @ tf for link, tf in kinematics.link_frames_base.items()
    }

    # 5. Transform all 1,062 taxel points and normals
    rot_target_base = t_target_base[:3, :3]
    reframed_taxels = []

    for t in kinematics.base_taxels:
        p_new = (t_target_base @ np.append(t.position, 1.0))[:3]
        n_new = rot_target_base @ t.normal
        n_new /= np.linalg.norm(n_new)

        reframed_taxels.append(BaseTaxel(
            key=t.key,
            link=t.link,
            wire_index=t.wire_index,
            row=t.row,
            col=t.col,
            position=p_new,
            normal=n_new
        ))

    return TactileHandKinematics(
        link_frames_base=reframed_links,
        link_frames_root=kinematics.link_frames_root,
        local_taxels=kinematics.local_taxels,
        base_taxels=reframed_taxels
    )