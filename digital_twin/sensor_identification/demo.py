"""Reproducible surface observations and an offline interactive 3D report."""
from pathlib import Path
import html
import json

import numpy as np
import trimesh
from scipy.spatial.transform import Rotation


def synthetic_points(model, *, count=64, seed=9, noise_mm=.2, region='full'):
    """Full coverage is a favorable benchmark, not a simulated physical grasp."""
    rng = np.random.default_rng(seed)
    cloud, _ = trimesh.sample.sample_surface(model.mesh, 12000, seed=seed)
    if region == 'shared':
        # The near-origin metal shaft: radial profile is inspected in README.
        radius = np.linalg.norm(cloud[:, [0, 2]], axis=1)
        cloud = cloud[(cloud[:, 1] > -25) & (cloud[:, 1] < -15)
                      & (radius > 8) & (radius < 14)]
    if len(cloud) < count:
        raise ValueError('Requested synthetic region has too few samples')
    source = cloud[rng.choice(len(cloud), count, replace=False)]
    r = Rotation.from_euler('xyz', [33, -24, 71], degrees=True).as_matrix()
    t = np.array([42., 18., 130.])
    xyz = source@r.T + t + rng.normal(0, noise_mm, source.shape)
    points = [dict(id=f'synthetic:{i}', x=float(p[0]), y=float(p[1]), z=float(p[2]),
                   pressure=float(np.clip(900+rng.normal(0, 50), 0, 4095)))
              for i, p in enumerate(xyz)]
    # Illustrative valid zero readings well outside the object. They are shown
    # in the report, but intentionally carry no certified exclusion radius.
    for i in range(8):
        p = r@(model.mesh.bounds[1]+[20.+4*i, 20., 20.])+t
        points.append(dict(id=f'noncontact:{i}', x=float(p[0]), y=float(p[1]),
                           z=float(p[2]), pressure=0.))
    return points


def write_viewer(path, points, models, result, *, threshold=200., coordinate_units='mm', boundary_points=()):
    """Self-contained Plotly HTML; triangle surfaces are not decimated for fitting."""
    import plotly.graph_objects as go
    from .classifier import valid_measurements
    clean, _ = valid_measurements(points)
    scale = 1000. if coordinate_units == 'm' else 1.
    fig = go.Figure()
    for active, label, color in [(True, 'Contact', '#eb4141'), (False, 'Valid non-contact', '#559bd8')]:
        xyz = np.array([[p[k]*scale for k in ('x', 'y', 'z')] for p in clean
                        if (p['pressure'] > threshold) == active]).reshape(-1, 3)
        fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
                                 mode='markers', marker=dict(size=4, color=color), name=label))
    unknown = []
    for p in points if isinstance(points, (list, tuple)) else []:
        if (isinstance(p, dict) and p.get('pressure') is None
                and all(isinstance(p.get(k), (int, float)) and not isinstance(p[k], bool)
                        and np.isfinite(p[k]) for k in ('x', 'y', 'z'))):
            unknown.append([p[k]*scale for k in ('x', 'y', 'z')])
    xyz = np.array(unknown, dtype=float).reshape(-1, 3)
    fig.add_trace(go.Scatter3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2], mode='markers',
                             marker=dict(size=2, color='#8496a6'), name='Pressure unknown'))
    for kind, label, color in [('inactive_taxel', 'Inactive taxel limits', '#58bfff'),
                               ('unknown_taxel', 'Unknown pressure surface limits', '#94a3b8'),
                               ('other', 'Hand boundary limits', '#b88aff')]:
        layer = [p for p in boundary_points if
                 (p.get('kind') == kind if kind != 'other' else p.get('kind') not in ('inactive_taxel', 'unknown_taxel'))]
        xyz = np.array([[p[k]*scale for k in ('x','y','z')] for p in layer], dtype=float).reshape(-1,3)
        fig.add_trace(go.Scatter3d(x=xyz[:,0], y=xyz[:,1], z=xyz[:,2], mode='markers',
                                 marker=dict(size=3,color=color,symbol='diamond'), name=label))
    for name, color in [('SENSOR_A', '#18b79c'), ('SENSOR_B', '#d7a936')]:
        fit = result['fits'].get(name)
        if fit is None:
            continue
        mesh = models[name].mesh
        transform = np.array(fit['transform'])
        xyz = mesh.vertices@transform[:3, :3].T+transform[:3, 3]
        faces = mesh.faces
        fig.add_trace(go.Mesh3d(x=xyz[:, 0], y=xyz[:, 1], z=xyz[:, 2],
                               i=faces[:, 0], j=faces[:, 1], k=faces[:, 2], color=color,
                               opacity=.25, name=f"{name} {'valid' if fit['compatible'] else 'REJECTED'} score={fit['score']:.2f}",
                               visible=True if fit['compatible'] else 'legendonly', showlegend=True))
    fig.update_layout(title=result['classification'],
                      scene=dict(aspectmode='data', xaxis_title='X (mm)',
                                 yaxis_title='Y (mm)', zaxis_title='Z (mm)'),
                      legend=dict(orientation='h', x=0, y=1.05),
                      autosize=True, margin=dict(l=0, r=0, b=0, t=80))
    body = fig.to_html(full_html=False, include_plotlyjs=True,
                       default_height='max(220px, calc(100vh - 90px))',
                       config={'responsive': True, 'displaylogo': False})
    details = html.escape(json.dumps(result, indent=2, allow_nan=False))
    Path(path).write_text('<!doctype html><meta charset="utf-8"><title>CAD tactile identification</title>'
                         '<style>body{font:14px system-ui;margin:16px}pre{overflow:auto;max-width:100%;'
                         'padding:12px;background:#f4f6f8}p{max-width:100%}</style>'
                         +body+'<p>Drag to rotate; click legend entries to hide each candidate. '
                         'Purple hand points and known inactive taxels with normals limit inward penetration. '
                         'Active pressure points guide surface alignment. Limits are approximate local surfaces. '
                         'Transforms map STL mm into the observation frame in mm.</p><details><summary>Fit details</summary><pre>'+details+'</pre></details>',
                         encoding='utf-8')
