"""Finite CAD volume queries at measured hand poses, never inward prisms.

Open links are explicitly excluded from containment. Sampling does not certify
continuous mesh/mesh clearance or motion between selected measured poses.
"""
from functools import lru_cache
import json
from pathlib import Path
import re

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation

from . import geometry
from .geometry import CandidateModel
from .ray_bvh import cached_bvh


def _rigid(xyz, rpy):
    values = np.r_[xyz, rpy].astype(float)
    if not np.isfinite(values).all() or np.max(abs(values[:3])) > 2 or np.max(abs(values[3:])) > 20:
        raise ValueError('Invalid recorded hand transform')
    out = np.eye(4)
    out[:3, :3] = Rotation.from_euler('xyz', values[3:]).as_matrix()
    out[:3, 3] = values[:3]*1000.
    return out


@lru_cache(maxsize=1)
def _assets():
    root = Path(geometry.__file__).resolve().parents[1]
    text = (root/'hand_twin_6dof.html').read_text(encoding='utf-8')
    data = json.loads(re.search(r'<script id="jointProfileData" type="application/json">(.*?)</script>', text, re.S)[1])
    profile = json.loads((root/'joint_geometry_profiles_for_codex.json').read_text(encoding='utf-8'))['primary_profiles']['rh56_tactile_toolkit_left']
    closed, skipped = [], []
    for asset in data['assets']['rh56_tactile_toolkit_left']:
        transform = _rigid(asset['origin_xyz_m'], asset['origin_rpy_rad'])
        vertices = np.asarray(asset['vertices']).reshape(-1, 3)*np.asarray(asset['scale'])*1000.
        mesh = trimesh.Trimesh(vertices=vertices@transform[:3, :3].T+transform[:3, 3],
                               faces=np.asarray(asset['indices']).reshape(-1, 3), process=True)
        if mesh.is_watertight and mesh.is_winding_consistent:
            closed.append((asset['link'], CandidateModel(asset['link'], mesh, sample_count=3000)))
        else:
            skipped.append(asset['link'])
    return profile, closed, skipped, data['reference_sha256']


class FiniteHand:
    def __init__(self, parts=(), skipped=(), reason=None):
        self.parts, self.skipped, self.reason = list(parts), list(skipped), reason

    @classmethod
    def from_snapshot(cls, snapshot):
        if not isinstance(snapshot, dict) or snapshot.get('frame') != 'left_base_link':
            return cls(reason='recorded_hand_pose_unavailable')
        if snapshot.get('profile') != 'rh56_tactile_toolkit_left':
            return cls(reason='unsupported_hand_profile')
        try:
            profile, assets, skipped, reference = _assets()
            if snapshot.get('reference_sha256') != reference:
                return cls(reason='hand_reference_mismatch')
            calibration = snapshot['kinematics_calibration']
            q = snapshot['joint_angles_rad']
            frames = {profile['root_link']: np.eye(4)}
            pending = list(profile['joints'])
            while pending:
                ready = [joint for joint in pending if joint['parent'] in frames]
                if not ready:
                    raise ValueError('Invalid hand joint hierarchy')
                for joint in ready:
                    settings = calibration['joints'][joint['name']]
                    transform = _rigid(settings['origin_xyz_m'], settings['origin_rpy_rad'])
                    if joint['type'] != 'fixed':
                        angle = float(q[joint['name']]); axis = np.asarray(settings['axis_xyz'], dtype=float)
                        if not np.isfinite(angle) or abs(angle) > 20 or not np.isfinite(axis).all() or np.linalg.norm(axis) < 1e-8:
                            raise ValueError('Invalid recorded joint angle')
                        turn = np.eye(4); turn[:3, :3] = Rotation.from_rotvec(axis/np.linalg.norm(axis)*angle).as_matrix()
                        transform = transform@turn
                    frames[joint['child']] = frames[joint['parent']]@transform
                    pending.remove(joint)
            base_inverse = np.linalg.inv(frames['left_base_link'])
            parts=[(name, model, base_inverse@frames[name]) for name, model in assets]
            motion=snapshot.get('motion_samples', [])
            if not isinstance(motion, list) or len(motion)>68:
                raise ValueError('Invalid recorded hand motion coverage')
            seen=set()
            count=0
            for sample in motion:
                if not isinstance(sample, dict):
                    raise ValueError('Invalid recorded hand motion sample')
                prefix=sample.get('link_prefix')
                if prefix not in ['left_'+finger+'_' for finger in ('thumb','index','middle','ring','little')]:
                    raise ValueError('Invalid motion link prefix')
                pose=cls.from_snapshot({**snapshot, 'joint_angles_rad':sample['joint_angles_rad'], 'motion_samples':[]})
                if not pose.parts:
                    raise ValueError('Invalid motion joint angles')
                count+=1
                for name, model, transform in pose.parts:
                    if name.startswith(prefix):
                        key=(name, tuple(np.round(transform.ravel(), 8)))
                        if key not in seen:
                            seen.add(key);parts.append((name, model, transform))
            hand=cls(parts, skipped)
            hand.motion_pose_count=count
            return hand
        except (ValueError, KeyError, TypeError, OSError) as error:
            return cls(reason='invalid_recorded_hand_pose: '+str(error))

    def depths(self, world_points):
        points = np.asarray(world_points, dtype=float).reshape(-1, 3)
        depth = np.zeros(len(points))
        for _, model, transform in self.parts:
            local = (points-transform[:3, 3])@transform[:3, :3]
            selected = np.flatnonzero(np.all((local >= model.mesh.bounds[0]) & (local <= model.mesh.bounds[1]), axis=1))
            if not len(selected):
                continue
            inside = selected[cached_bvh(model).contains(local[selected])]
            if len(inside):
                depth[inside] = np.maximum(depth[inside], model.nearest(local[inside])[1])
        return depth

    def correction_targets(self, world_points, tolerance_mm):
        """Freeze nearest outward escape directions for penetrating samples."""
        points = np.asarray(world_points, dtype=float).reshape(-1, 3)
        ids, targets, normals = [], [], []
        for _, model, transform in self.parts:
            local = (points-transform[:3, 3])@transform[:3, :3]
            selected = np.flatnonzero(np.all((local >= model.mesh.bounds[0]) & (local <= model.mesh.bounds[1]), axis=1))
            if not len(selected):
                continue
            inside = selected[cached_bvh(model).contains(local[selected])]
            if not len(inside):
                continue
            surface, distance = model.nearest(local[inside])
            keep = distance > tolerance_mm
            if np.any(keep):
                indices = inside[keep]
                target = surface[keep]@transform[:3, :3].T+transform[:3, 3]
                normal = (target-points[indices])/distance[keep, None]
                ids.extend(indices); targets.extend(target); normals.extend(normal)
        return np.asarray(ids, dtype=int), np.asarray(targets).reshape(-1, 3), np.asarray(normals).reshape(-1, 3)

    def audit(self, model, rotation, translation, tolerance_mm):
        # Deterministic surface coverage; count samples, never pretend these
        # are continuous collision events or every point of the source mesh.
        samples = model.samples[np.linspace(0, len(model.samples)-1, 2400, dtype=int)]
        points = samples@rotation.T+translation
        depth = self.depths(points) if self.parts else np.zeros(len(points))
        return dict(hand_limit_max_mm=float(depth.max()), hand_limit_rmse_mm=float(np.sqrt(np.mean(depth*depth))),
                    hand_collision_count=int(np.count_nonzero(depth > tolerance_mm)),
                    hand_constraints_checked=len(points) if self.parts else 0,
                    hand_validation_complete=False, finite_hand_available=bool(self.parts),
                    finite_hand_closed_links=list(dict.fromkeys(part[0] for part in self.parts)),
                    finite_hand_recorded_motion_poses=getattr(self, 'motion_pose_count', 0),
                    finite_hand_open_links=list(self.skipped), finite_hand_unavailable_reason=self.reason,
                    hand_validation_scope='2400 object surface samples against closed CAD links at frozen and selected measured poses; open links and motion between samples unverified')
