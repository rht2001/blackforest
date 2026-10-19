"""Empirical starting guesses, never identity or contact acceptance evidence."""

POSES = {
    'bcf73f55eb4674d01f7de225ce0e07d3e3d15954aff24f2f3e1519a6b050c974':
        ([20.0090457787, 23.5735870142, 129.2507983112], [-3.1826635762, -2.7735447350, 91.1918856940]),
    'ec9ec5eebfb97d6af6623371953eb2bb4836858568952cb12f6c77e5b38e2366':
        ([13.3468481156, 35.8566909211, 128.9623045688], [-179.5800221948, -3.2957311234, 90.4413873626]),
}


def empirical_pose(model, coordinate_frame):
    """The learned coordinates are only meaningful in this explicit base frame."""
    pose = POSES.get(getattr(model, 'source_sha256', None))
    if coordinate_frame != 'left_base_link' or pose is None:
        return None
    center, angles = pose
    return dict(center_mm=list(center), rotation_xyz_deg=list(angles),
                position_window_mm=[30., 12., 15.], tilt_window_deg=30.,
                shaft_roll_window_deg=180., source_recordings=53,
                source_sha256=model.source_sha256, frame='left_base_link',
                method='empirical_initialization_not_identity_evidence')
