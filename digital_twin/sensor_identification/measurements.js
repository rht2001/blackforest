/* Adapt current viewer exports; do not recalculate kinematics or read hardware. */
(function(root) {
  'use strict';
  function capture(viewer) {
    const cloud = viewer.getContactCloud?.();
    const state = viewer.ProfileTwin?.snapshot?.();
    const saved = !!state?.replay;
    const manual = !!state && !saved && !state.live && !state.demo;
    const available = viewer.getTaxelPositions?.() || [];
    const valid = (cloud?.valid === true || (manual && available.length > 0)) && cloud?.frame === 'left_base_link' && cloud?.units === 'metres';
    let points = [];
    if (valid) {
      if (!saved) points = available.map(p => ({...p, pressure:manual?null:p.pressure}));
      if (saved && viewer.HandHistory?.points) points = viewer.HandHistory.points();
      // Saved exports contain activated contacts only. Do not reconstruct
      // non-contact pressure from the hand mesh or the capture's other cells.
      if (!points.length) points = (cloud.points || []).map(p => ({
        id:p.id, x:p.position_m[0], y:p.position_m[1], z:p.position_m[2], pressure:p.value
      }));
    }
    return {
      schema_version:1, valid, frame:'left_base_link', units:'m',
      geometry_status:cloud?.geometry_status || 'UNCALIBRATED',
      measurement_source:saved ? 'saved_capture' : manual ? 'manual_pose' : state?.demo ? 'motion_demo' : 'live_viewer',
      saved_name:state?.replay?.name || null,
      received_at:cloud?.received_at ?? null,
      field_received_at:{...cloud?.field_received_at},
      timestamp_basis:cloud?.timestamp_basis || 'bridge_reception_not_hardware_capture',
      geometry_sha256:cloud?.geometry_sha256 ?? null,
      reference_sha256:cloud?.reference_sha256 ?? null,
      boundary_points:valid?(viewer.getHandBoundaryPoints?.()||[]).map(p=>({...p,normal:p.normal?.slice()})):[],
      hand_snapshot:valid && cloud?.angles_act ? JSON.parse(JSON.stringify(cloud)) : null,
      points
    };
  }
  root.IdentificationMeasurements = {capture};
  root.getIdentificationFrame = () => capture(root);
  if (typeof module !== 'undefined') module.exports = {capture};
})(typeof globalThis === 'undefined' ? this : globalThis);
