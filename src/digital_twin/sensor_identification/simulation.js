/* Start the independent fitter using the simulation's existing measurement snapshot. */
(() => {
  const wait = setInterval(() => {
    const anchor = document.getElementById('profileControls');
    if (!anchor) return;
    clearInterval(wait);
    attach(anchor);
  }, 250);

  function attach(anchor) {
    const panel = document.createElement('section');
    panel.id = 'identificationPanel';
    panel.innerHTML = `<style>
      #identificationPanel{font-size:12px;line-height:1.45}
      #identificationPanel button{font:inherit;color:#deedf7;background:#1c4055;border:1px solid #42647b;border-radius:6px;padding:7px;margin:3px 2px 3px 0;cursor:pointer}
      #identificationPanel button:disabled{opacity:.5;cursor:wait}
      #identificationOptions label{display:block;margin:7px 0;font-size:11px}
      #identificationOptions input{width:72px;margin-left:5px}
      #identificationPanel a{color:#78daf1}
      #identificationPanel h2{font-size:14px;font-weight:650;margin:0 0 8px}
      #identificationPanel select{display:block;width:100%;padding:9px;border:1px solid #42647b;border-radius:6px;background:#122c40;color:#deedf7;margin:7px 0}
      #identificationPanel [data-identify]{width:100%;background:#14536b;border-color:#32829a;margin:3px 0 8px}
      #identificationStatus{padding:10px;border-radius:7px;background:#112638;font-size:12px;margin:8px 0}
      #identificationPanel .muted{font-size:11px;margin:6px 0}
    </style>
      <select id="identificationSource" aria-label="Contact source">
        <option value="SENSOR_B">Sample · Sensor B</option>
        <option value="SENSOR_A">Sample · Sensor A</option>
        <option value="shared">Sample · Shared shaft</option>
        <option value="current">Current displayed contacts</option>
      </select>
      <p id="identificationHint" class="muted" hidden>Generated CAD contacts · no hardware needed</p>
      <button data-identify>Identify sensor</button>
      <p id="identificationStatus" role="status" aria-live="polite">Ready to compare both shapes.</p>
      <a id="identificationReport" target="_blank" rel="noopener" hidden>View fit in 3D ↗</a>
      <details id="identificationOptions"><summary>Identification settings</summary>
      <label>Contact threshold (counts) <input id="identifyThreshold" type="number" min="0" max="4095" value="200"></label>
      <label>Allowed position error (mm) <input id="identifyTolerance" type="number" min="0.1" max="20" step="0.1" value="1.5"></label>
      <p class="muted">Both shapes are moved and rotated to fit the contacts. Similar fits, poor fits, or insufficient evidence give AMBIGUOUS. Hand geometry still needs calibration. Motion-demo pressure is artificial and is not object-contact physics. Saved captures are analyzed in their recorded frame.</p>
      <p id="identificationScores" class="muted"></p></details>`;
    const group = document.createElement('details');
    group.id = 'sensorIdentificationPanel';
    group.open = true;
    const summary = document.createElement('summary');
    summary.textContent = 'Sensor identification';
    group.append(summary, panel);
    anchor.insertAdjacentElement('beforebegin', group);
    const options = panel.querySelector('#identificationOptions');

    const base = location.protocol === 'file:' ? 'http://127.0.0.1:18768' : location.origin;
    const status = panel.querySelector('#identificationStatus');
    const report = panel.querySelector('#identificationReport');
    const buttons = [...panel.querySelectorAll('[data-identify]')];
    const source = panel.querySelector('#identificationSource');
    function updateHint() {
      panel.querySelector('#identificationHint').hidden = source.value !== 'current';
      panel.querySelector('#identificationHint').textContent = source.value === 'current'
        ? 'Uses live, demo, or saved contacts · geometry uncalibrated'
        : 'Generated CAD contacts · no hardware needed';
    }
    source.onchange = () => {
      updateHint();
      status.textContent = 'Ready to compare both shapes.';
      report.hidden = true;
      options.querySelector('#identificationScores').textContent = '';
    };
    let timer = null;
    const reasons = {
      one_candidate_clearly_more_consistent: 'One sensor fits the contacts clearly better.',
      both_candidates_compatible: 'Both shapes fit. Touching this region cannot tell them apart.',
      score_margin_too_small: 'The difference is too small to choose safely.',
      neither_candidate_compatible_within_search: 'Neither shape explains these contacts well.',
      calibration_unverified: 'The hand geometry needs calibration before a confident result.',
      insufficient_contacts: 'There are too few valid contact points.',
      insufficient_spatial_coverage: 'The contacts cover too little of the object.',
      invalid_or_stale_frame: 'No valid contact snapshot is available. Select live feedback, a motion demo, or a valid saved capture.',
      alignment_search_unstable: 'The alignment search did not settle reliably.'
    };
    function display(state) {
      const running = state.status === 'running';
      buttons.forEach(button => { button.disabled = running; });
      source.disabled = running;
      if (state.mode) {
        source.value = state.mode;
        updateHint();
      }
      report.hidden = true;
      if (running) {
        status.textContent = 'Comparing shapes… This may take a few minutes.';
      } else if (state.status === 'complete') {
        const result = state.result;
        const label = result.classification === 'AMBIGUOUS' ? 'AMBIGUOUS — cannot choose safely' : result.classification.replace('_', ' ');
        const scores = Object.entries(result.fits || {}).map(([name, fit]) =>
          `${name.replace('_', ' ')} error: ${fit.rmse_mm.toFixed(2)} mm`).join(' · ');
        status.textContent = `${label}. ${reasons[result.reason] || result.reason.replaceAll('_', ' ')}`;
        options.querySelector('#identificationScores').textContent = scores;
        report.href = base + state.report_url;
        report.hidden = false;
      } else if (state.status === 'error') {
        status.textContent = 'Could not run identification: ' + state.error;
      }
      clearTimeout(timer);
      if (running) timer = setTimeout(refresh, 1500);
    }
    async function refresh() {
      try {
        const response = await fetch(base + '/api/identification', {signal: AbortSignal.timeout(5000)});
        const state = await response.json();
        if (!response.ok) throw Error(state.error || 'Identification service unavailable');
        display(state);
      } catch (error) {
        buttons.forEach(button => { button.disabled = false; });
        source.disabled = false;
        status.textContent = 'Start this simulation with Start Sensor Identification.cmd. ' + error.message;
      }
    }
    for (const button of buttons) button.onclick = async () => {
      const mode = source.value;
      const threshold = Number(options.querySelector('#identifyThreshold').value);
      const tolerance = Number(options.querySelector('#identifyTolerance').value);
      if (!Number.isFinite(threshold) || threshold < 0 || threshold > 4095 || !Number.isFinite(tolerance) || tolerance < .1 || tolerance > 20) {
        status.textContent = 'Use a threshold from 0 to 4095 and a position error from 0.1 to 20 mm.';
        return;
      }
      const frame = mode === 'current' ? window.getIdentificationFrame?.() : undefined;
      if (mode === 'current' && !frame?.valid) {
        status.textContent = reasons.invalid_or_stale_frame;
        return;
      }
      display({status:'running'});
      try {
        const response = await fetch(base + '/api/identification', {method:'POST',
          headers:{'Content-Type':'application/json'}, signal:AbortSignal.timeout(10000),
          body:JSON.stringify({mode, threshold, tolerance_mm:tolerance, frame})});
        const state = await response.json();
        if (!response.ok && response.status !== 409) throw Error(state.error || 'Could not start');
        display(state);
      } catch (error) { display({status:'error',error:error.message}); }
    };
    refresh();
  }
})();
