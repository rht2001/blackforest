/* ESTIMATE_FEEDBACK_COMPONENT */
/* Estimate feedback; formatters are pure so malformed/missing metrics are safe. */
(function(root){
 'use strict';
 const escape=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
 const mm=value=>typeof value==='number'&&Number.isFinite(value)?value.toFixed(2)+' mm':'—';
 const elapsed=(start,end)=>{if(!Number.isFinite(start))return '';const seconds=Math.max(0,Math.floor((end-start)));return seconds<60?seconds+'s':Math.floor(seconds/60)+'m '+String(seconds%60).padStart(2,'0')+'s'};
 const stages={thumb_first_guess:'Thumb nearly closed: trying the smaller sensor first',queued:'Starting estimate',loading_models:'Loading sensor models',preparing:'Checking measurements',palm_initialization:'Fitting a palm-aligned starting pose',alignment:'Matching surfaces',refining:'Refining pose',checking_limits:'Checking hand penetration',writing_report:'Preparing result'};
 const rejection={contact_outside_tolerance:'Some red contacts are too far from the surface.',hand_penetration:'The model crosses the hand limits.',certified_clearance_violated:'The model violates a certified clearance region.'};
 const sensorName=(name,context={})=>context.models?.[['SENSOR_A','SENSOR_B'].indexOf(name)]?.name||({SENSOR_A:'VEGABAR 29',SENSOR_B:'VEGABAR 38'}[name])||name;
 function identityBox(result,compatible,context){
  const confirmed=result.classification&&result.classification!=='AMBIGUOUS'&&result.fits?.[result.classification]?.compatible===true;
  const selected=result.placement_candidate&&result.fits?.[result.placement_candidate]?.compatible===true?result.placement_candidate:compatible.length===1?compatible[0][0]:null;
  const label=confirmed?sensorName(result.classification,context):selected?sensorName(selected,context)+' (provisional)': 'Undetermined';
  const detail=confirmed?'Identified by the estimate.':selected?'Provisional placement; sensor identity is not confirmed.':compatible.length?'Possible sensors: '+compatible.map(([name])=>sensorName(name,context)).join(' or ')+'.':'No sensor model passed the fit checks.';
  return '<section class="sensor-identity" aria-label="Sensor identification"><strong>Sensor: '+escape(label)+'</strong><p class="fit-small">'+escape(detail)+'</p></section>';
 }
 // STL bounding-box center transformed into the observation frame (millimetres).
 function sensorPose(fit,model){
  const t=fit?.transform,c=model?.source_center_mm;
  if(!Array.isArray(t)||t.length!==4||t.some(row=>!Array.isArray(row)||row.length!==4||!row.every(Number.isFinite))||!Array.isArray(c)||c.length!==3||!c.every(Number.isFinite))return null;
  const center=t.slice(0,3).map(row=>row[3]+c.reduce((sum,v,k)=>sum+row[k]*v,0));
  // R = Rz(yaw) Ry(pitch) Rx(roll); choose roll=0 at gimbal lock.
  const horizontal=Math.hypot(t[0][0],t[1][0]),pitch=Math.atan2(-t[2][0],horizontal);
  const roll=horizontal>1e-8?Math.atan2(t[2][1],t[2][2]):0;
  const yaw=horizontal>1e-8?Math.atan2(t[1][0],t[0][0]):Math.atan2(-t[0][1],t[1][1]);
  return {center,pitch:pitch*180/Math.PI,yaw:yaw*180/Math.PI,roll:roll*180/Math.PI};
 }
 function poseBox(name,fit,context){
  const index=['SENSOR_A','SENSOR_B'].indexOf(name),model=index<0?null:context.models?.[index],pose=sensorPose(fit,model);
  const frame=escape(context.frame||'observation frame');
  if(!pose)return '<section class="sensor-pose"><h4>Estimated sensor center &amp; orientation</h4><p class="fit-small">Pose unavailable for this candidate.</p></section>';
  const value=n=>(Math.abs(n)<.005?0:n).toFixed(2);
  const metric=(label,n,unit)=>'<div><dt>'+label+'</dt><dd>'+value(n)+' '+unit+'</dd></div>';
  return '<section class="sensor-pose" aria-label="'+escape(name)+' estimated pose"><h4>Estimated sensor center &amp; orientation</h4><p class="fit-small">'+escape(model.name||name)+' · '+frame+(fit.compatible===true?'':' · Rejected candidate pose')+'</p><dl>'+['X','Y','Z'].map((axis,k)=>metric(axis,pose.center[k],'mm')).join('')+['pitch','yaw','roll'].map(axis=>metric(axis[0].toUpperCase()+axis.slice(1),pose[axis],'°')).join('')+'</dl><p class="fit-small">Center: STL bounding-box midpoint. Pitch about Y, yaw about Z, roll about X; R = Rz(yaw) Ry(pitch) Rx(roll).</p></section>';
 }
 function card(name,fit,contacts,context){
  const compatible=fit.compatible===true;
  const fraction=Number.isFinite(fit.inlier_fraction)?Math.max(0,Math.min(1,fit.inlier_fraction)):null;
  const within=fraction===null?'—':Number.isInteger(contacts)?Math.round(fraction*contacts)+' / '+contacts:Math.round(fraction*100)+'%';
  const reasons=(fit.rejection_reasons||[]).map(key=>rejection[key]||key);
  if(fit.optimizer_success===false)reasons.push('The pose search did not converge.');
  if(!fit.quick_preview&&!compatible&&!reasons.length)reasons.push('This candidate did not satisfy the fit checks.');
  const metric=(label,value)=>'<div><dt>'+escape(label)+'</dt><dd>'+escape(value)+'</dd></div>';
  return '<details class="fit-card" open><summary>'+escape(sensorName(name,context))+' <span class="fit-badge '+(compatible?'fit-good':'fit-bad')+'">'+(compatible?(fit.quick_preview?'Quick placement':'Geometry fits'):'Rejected')+'</span></summary><dl>'+
   metric('Contact error (RMS)',mm(fit.rmse_mm))+metric('Worst contact error',mm(fit.max_error_mm))+
   metric('Contacts in tolerance',within)+metric('Max hand penetration',mm(fit.hand_limit_max_mm))+metric('Hand limits exceeded',Number.isInteger(fit.hand_collision_count)?fit.hand_collision_count+' / '+fit.hand_constraints_checked:'—')+'</dl><p class="fit-small">Allowed: contact '+escape(mm(fit.contact_tolerance_mm))+' · hand '+escape(mm(fit.hand_tolerance_mm))+'</p>'+
   (Number.isInteger(fit.input_contact_points)?'<p class="fit-small">Checked '+escape(fit.scored_contact_points)+' / '+escape(fit.input_contact_points)+' contact points · '+escape(fit.hand_constraints_checked)+' / '+escape(fit.input_hand_points)+' hand limits · spacing '+escape(mm(fit.point_spacing_mm))+'</p>':'')+
   (Number.isInteger(fit.pressure_direction_patches)?'<p class="fit-small">Pressure direction: '+escape(fit.pressure_direction_patches)+(fit.pressure_direction_patches?' local patches guide orientation.':' reliable local patches; orientation uses contact positions.')+'</p>':'')+
   reasons.map(reason=>'<p class="fit-reason">'+escape(reason)+'</p>').join('')+poseBox(name,fit,context)+'</details>';
 }
 function render(state,explanation='',now=Date.now()/1000,context={}){
  const source=state.input_label?'<p class="fit-small">Snapshot: '+escape(state.input_label)+'</p>':'';
  if(state.status==='running'){
   const p=state.progress||{},hasCount=Number.isFinite(p.completed)&&Number.isFinite(p.total)&&p.total>0;
   const label=(p.candidate?p.candidate+' · ':'')+(stages[p.stage]||'Estimating frozen snapshot');
   return '<h3>Estimating… <span class="fit-time">'+escape(elapsed(state.started_at,now))+'</span></h3>'+source+'<p>'+escape(label)+'</p><progress aria-label="Current estimation stage" '+(hasCount?'max="'+p.total+'" value="'+Math.max(0,Math.min(p.total,p.completed))+'"':'')+'></progress>'+
    (hasCount?'<p class="fit-small">'+escape(p.completed+' / '+p.total)+' '+(p.stage==='alignment'?'starting poses':'pose refinements')+'</p>':'')+
    (state.first_guess?'<p class="fit-small">First guess: '+escape(sensorName(state.first_guess.candidate,context))+' · thumb '+Math.round(state.first_guess.closure_fraction*100)+'% closed with contact. Provisional; checking geometry.</p>':'')+'<p class="fit-small">You can keep inspecting the hand while the pose is fitted.</p>';
  }
  if(state.status==='error'||state.status==='connection_error')return '<h3>'+(state.status==='connection_error'?'Connection lost':'Estimate failed')+'</h3>'+source+'<p class="fit-reason">'+escape(state.error||'No error details were returned.')+'</p>'+(state.status==='connection_error'?'<p class="fit-small">The job may still be running.</p><button id="retryEstimateStatus">Check status</button>':'');
  if(state.status!=='complete')return '';
  const result=state.result||{},fits=Object.entries(result.fits||{}),compatible=fits.filter(([,f])=>f.compatible===true);
  const title=result.quick_preview&&compatible.length?'Quick placement':result.classification&&result.classification!=='AMBIGUOUS'?'Estimated '+sensorName(result.classification,context):!fits.length?'No anchored pose':compatible.length===2?'Both models fit':compatible.length===1?'Provisional fit':'Neither model fits';
  const duration=elapsed(state.started_at,state.finished_at??now);
  const counts=[];if(Number.isInteger(result.contacts))counts.push(result.contacts+' red contacts');
  const limits=(result.hand_boundary_points||0)+(result.surface_taxel_limits||0);if(limits)counts.push(limits+' hand limits');
  if(result.invalid_points)counts.push(result.invalid_points+' invalid readings ignored');
  return '<h3>'+escape(title)+(duration?' <span class="fit-time">'+escape(duration)+'</span>':'')+'</h3>'+source+'<p>'+escape(explanation||result.reason||'Estimate complete.')+'</p><p class="fit-small">'+escape(counts.join(' · '))+'</p>'+
   (result.classification==='AMBIGUOUS'&&compatible.length?'<p class="fit-small">A geometric fit is provisional; the sensor identity is not confirmed.</p>':'')+
   (result.placement_candidate?'<p class="fit-small">'+(result.first_guess?'Thumb nearly closed · smaller sensor first · selected ':'Larger sensor first · selected ')+escape(sensorName(result.placement_candidate,context))+(result.placement_reason==='smaller_candidate_after_larger_rejected'?' after the larger sensor failed the fit limits.':'.')+'</p>':'')+
   identityBox(result,compatible,context)+fits.map(([name,fit])=>card(name,fit,fit.scored_contact_points??result.contacts,context)).join('')+
   (!fits.length?'<p class="fit-small">More valid, spatially spread contact readings are needed to locate the object.</p>':'')+
   '<p class="fit-small">Open Fit report for the full geometry comparison.</p>';
 }
 root.EstimateFeedback={render,elapsed,sensorPose};
 if(typeof module!=='undefined')module.exports=root.EstimateFeedback;
})(typeof globalThis==='undefined'?this:globalThis);

/* One page: select the data source, inspect it, then estimate that frozen cloud. */

(() => {

  const ready=setInterval(()=>{

    if(!window.HandHistory||!document.querySelector('#profileScene0')?._renderer)return;

    clearInterval(ready);start();

  },200);

  function start(){

    const $=id=>document.getElementById(id),panel=$('panel'),old=[...panel.children];

    const app=document.createElement('div');app.id='onePageApp';

    app.innerHTML=`<style>
      :root{--bg:#101419;--panel:#191e24;--line:#343c46;--muted:#a6afb9;--cyan:#98b9dc;color:#e3e7ec;font-family:"Segoe UI",Arial,sans-serif;color-scheme:dark}
      body{background:var(--bg)}
      #panel{top:0;left:0;width:360px;height:100vh;max-height:100vh;padding:20px 24px;border:0;border-right:1px solid var(--line);border-radius:0;background:var(--panel);box-shadow:none;backdrop-filter:none;scrollbar-width:thin;scrollbar-color:#454e59 transparent}
      #panel::-webkit-scrollbar{width:5px}#panel::-webkit-scrollbar-thumb{background:#454e59;border-radius:0}
      body.profile-mode #profileWorkspace{left:360px;background:var(--bg)}body.profile-mode .profile-card{border:0}
      body.profile-mode .profile-caption{top:25px;left:28px}.profile-caption h2{font-size:13px;font-weight:600;letter-spacing:.02em;color:#c4cbd3}
      .profile-caption p,body.profile-mode .profile-caption .profile-status{font-size:12px;color:#a6afb9}
      .profile-angles{background:#101419e6;border-radius:0;color:#a6afb9}
      #onePageApp{font-size:13px;line-height:1.5}
      #onePageApp .app-heading{margin:0 0 18px;padding-bottom:16px;border-bottom:1px solid var(--line)}
      #onePageApp .app-heading h1{font-size:21px;line-height:1.25;font-weight:600;letter-spacing:-.5px;margin:0 0 5px}
      #onePageApp .app-heading p{margin:0;font-size:11px;letter-spacing:.04em;color:var(--muted)}
      #onePageApp h2{font-size:12px;font-weight:600;margin:16px 0 8px;color:#dce2e9}
      #onePageApp>h2:first-of-type{margin-top:0}
      #onePageApp p{margin:8px 0}
      #onePageApp .hint{font-size:12px;line-height:1.55;color:var(--muted)}
      #onePageApp button,#onePageApp select,#onePageApp input{font:inherit}
      #onePageApp button{min-height:34px;background:#252c34;color:#dce2e9;border:1px solid #444e5a;border-radius:4px;padding:7px 10px;cursor:pointer;line-height:1.35}
      #onePageApp button:hover:not(:disabled){background:#313a45;border-color:#778596}
      #onePageApp button:disabled{opacity:.4;cursor:not-allowed}
      #onePageApp :is(button,input,select,summary):focus-visible{outline:2px solid #a8c6e6;outline-offset:3px}
      #onePageApp input[type=checkbox],#onePageApp input[type=range]{accent-color:#98b9dc}
      #onePageApp .data-modes{display:grid;grid-template-columns:repeat(3,1fr);gap:0;margin-bottom:10px;border-bottom:1px solid #444e5a}
      #onePageApp .data-modes button{background:transparent;border:0;border-bottom:2px solid transparent;border-radius:0;color:#a6afb9;padding:8px 4px}
      #onePageApp .data-modes button.active{background:transparent;color:#edf3fa;border-bottom-color:#98b9dc;font-weight:600}
      .data-actions{display:flex;gap:6px;flex-wrap:wrap;margin:10px 0}.data-actions button{flex:1}
      #onePageApp select{width:100%;min-height:34px;padding:6px 8px;background:#14191f;color:#e3e7ec;border:1px solid #444e5a;border-radius:4px}
      #onePageApp input[type=number]{max-width:90px;background:#14191f;color:#e3e7ec;border:1px solid #444e5a;border-radius:4px;padding:6px;font-variant-numeric:tabular-nums}
      #onePageApp input[type=file]{max-width:100%;font-size:12px}
      #onePageApp #estimateButton{width:100%;background:#b9cfe5;color:#142333;border-color:#b9cfe5;font-weight:600;margin:2px 0 4px;min-height:38px}
      #onePageApp #estimateButton:hover:not(:disabled){background:#d0dfee;border-color:#d0dfee}
      #estimateStatus{border-left:2px solid #536476;padding:3px 0 3px 10px;font-size:12px;color:#a6afb9;background:transparent;border-radius:0}
      #onePageApp section[aria-label="Received probing runs"]{margin-top:16px;padding-top:2px;border-top:1px solid var(--line)}
      #receivedRunStatus{font-size:12px;color:#c4cbd3;overflow-wrap:anywhere}
      #onePageApp details{border-top:1px solid var(--line);margin-top:14px;padding-top:12px}
      #onePageApp summary{font-size:12px;font-weight:500;color:#b8c2ce;cursor:pointer}
      #onePageApp label{display:block;margin:9px 0;font-size:12px}
      #onePageApp [hidden]{display:none!important}
      #onePageApp a{color:#b9cfe5;text-underline-offset:3px}
      #onePageApp .channel{padding:10px 0;background:transparent;border:0;border-bottom:1px solid var(--line);border-radius:0}
      #onePageApp .channel-name{color:#dce2e9!important;font-weight:500}
      #onePageApp .channel-info{color:#a6afb9}
      #onePageApp :is(#profileControls,#savedCaptures,#sensorEditor){background:transparent;border:0;border-radius:0;padding:0}
      #estimateFeedback{margin:16px 0;padding:14px 0;border:0;border-top:1px solid var(--line);border-radius:0;background:transparent}
      #estimateFeedback h3{font-size:13px;margin:0 0 7px}#estimateFeedback progress{width:100%;height:6px;accent-color:#98b9dc}
      #estimateFeedback .fit-small{font-size:12px;color:#a6afb9;overflow-wrap:anywhere}#estimateFeedback .fit-time{float:right;font-size:12px;color:#a6afb9;font-weight:400}
      #estimateFeedback .fit-card{padding:10px 0 2px;margin:12px 0 0;border-top:1px solid var(--line)}#estimateFeedback summary{font-weight:600;cursor:pointer}
      #estimateFeedback .fit-badge{font-size:10px;float:right;padding:2px 5px;border-radius:3px}.fit-good{background:#253e33;color:#b2d6bf}.fit-bad{background:#462e32;color:#edb6be}
      #estimateFeedback dl{display:grid;grid-template-columns:1fr 1fr;gap:12px 8px;margin:12px 0}#estimateFeedback dt{font-size:11px;color:#a6afb9}#estimateFeedback dd{font-size:14px;margin:2px 0 0;font-variant-numeric:tabular-nums}
      #estimateFeedback .sensor-identity{border:1px solid #536476;background:#202c38;padding:12px;margin:12px 0}
      #estimateFeedback .sensor-identity strong{font-size:14px;color:#d8e6f4}
      #estimateFeedback .sensor-identity p{margin:6px 0 0}
      #estimateFeedback .sensor-pose{border:1px solid #536476;background:#202c38;padding:12px;margin:12px 0 8px}
      #estimateFeedback .sensor-pose h4{font-size:12px;margin:0 0 6px;color:#d8e6f4}
      #estimateFeedback .sensor-pose dl{grid-template-columns:repeat(3,minmax(0,1fr));gap:12px 6px}
      #estimateFeedback .sensor-pose dd{white-space:nowrap;font-size:13px}
      #estimateFeedback .fit-reason{font-size:12px;color:#edb6be}
      #estimateView{position:absolute;inset:0;z-index:8;background:#fff}#estimateView[hidden]{display:none}#estimateView iframe{width:100%;height:100%;border:0}
      @media(min-width:621px) and (max-width:900px){#panel{width:310px;padding:22px 18px}body.profile-mode #profileWorkspace{left:310px}}
      @media(max-width:620px){#panel{width:100%;height:50vh;max-height:50vh;padding:18px;border-right:0;border-bottom:1px solid var(--line)}body.profile-mode #profileWorkspace{left:0;top:50vh}body.profile-mode .profile-caption{top:14px;left:18px}#onePageApp .app-heading{margin-bottom:18px;padding-bottom:14px}#onePageApp .app-heading h1{font-size:19px}}
    </style><header class="app-heading"><h1>Hand twin</h1><p>RH56DFTP / Sensor identification</p></header><h2>Data source</h2>

      <div class="data-modes" role="tablist" aria-label="Data mode">

        <button role="tab" id="dataManual">Manual</button><button role="tab" id="dataLive">Live</button><button role="tab" id="dataHistory">History</button>

      </div><p id="sourceInfo" class="hint"></p>

      <section id="manualData"><div class="data-actions" id="manualActions"></div><details id="manualAngles"><summary>Set finger angles</summary></details></section>

      <section id="liveData" hidden></section>

      <section id="historyData" hidden><button id="chooseHistory" type="button">Choose JSON or CSV…</button><input id="historyFiles" type="file" accept=".json,.csv,application/json,text/csv" hidden><p id="selectedHistory" class="hint">No history file selected.</p>

        <div id="csvControls" hidden><label>Use CSV as <select id="csvScope"><option value="run">Whole probing run</option><option value="row">Single row</option></select></label>

        <div id="runControls"><label>Run <select id="historyRun"></select></label><label>Contacts <select id="runRegions"><option value="all">All hand regions</option><option value="thumb">Thumb only</option></select></label><label>Point spacing (mm) <input id="runVoxel" type="number" min="0.1" max="10" step="0.1" value="1"></label><button id="buildRun">Build run point cloud</button><p class="hint">Keeps the first contact patch for each closing finger. Finger surface limits use each finger’s most closed recorded pose. Thumb probing and palm contacts use all valid recorded poses. Coordinates assume the object stays fixed relative to the hand base.</p></div>

        <div id="rowControls" hidden><label>CSV row <input id="historyRow" type="number" value="1" min="1"></label><div class="data-actions"><button id="historyPrev">Previous</button><button id="historyNext">Next</button></div></div><p id="historyRowInfo" class="hint"></p></div>

        <button id="downloadRun" hidden>Save run point cloud JSON</button>

        <div id="runLayers" hidden><label><input id="runShowAxes" type="checkbox" checked> Show coordinate frame</label><p class="hint"><span style="color:#ff595e">X</span> · <span style="color:#49d17d">Y</span> · <span style="color:#58a6ff">Z</span> · left_base_link · origin (0, 0, 0) · arrows 40 mm</p><label><input id="runShowLimits" type="checkbox" checked> Show hand-space limits</label><p class="hint"><span style="color:#b88aff">Purple: hand surfaces</span> · <span style="color:#58bfff">Blue: inactive taxels</span> · <span style="color:#94a3b8">Gray: pressure unknown</span>. Bright pressure colors mark active contacts. Each limit follows its recorded hand pose.</p></div>

        <p id="historyStatus" class="hint">Choose a file to preview it before estimating.</p>

      </section>

      <section id="receivedRuns" aria-label="Received probing runs">
        <h2>Received probing runs</h2>
        <p class="hint">Capture live readings during a probing run. Forwarded probing states can also mark a run.</p>
        <label><input id="captureOnClose" type="checkbox" checked> Record when fingers close; finish when hand opens</label>
        <p id="closureCaptureStatus" class="hint">Waiting to observe an open hand before arming.</p>
        <button id="startReceivedCapture">Start capture</button><button id="finishReceivedCapture" disabled>Finish capture</button>
        <label><input id="autoReceiveRun" type="checkbox" checked> Open completed probing runs automatically</label>
        <p id="receivedRunStatus" role="status">Waiting for probing telemetry…</p>
        <label>Saved capture <select id="receivedRunSelect"><option value="">No captures yet</option></select></label>
        <button id="openReceivedRun" disabled>Build captured point cloud</button>
        <a id="downloadReceivedCSV" hidden>Download captured CSV</a>
      </section>
      <h2>Sensor estimation</h2><button id="estimateButton">Estimate sensor</button><p id="estimateStatus" role="status" aria-live="polite">Ready. The selected data source supplies the point cloud.</p><section id="estimateFeedback" hidden aria-label="Estimate feedback"></section>

      <div class="data-actions"><button id="showHand">Hand view</button><button id="showCloud" disabled>Point cloud</button><button id="showEstimate" disabled>Estimate in hand</button><button id="showFitReport" disabled>Fit report</button></div><label id="estimateCandidateLabel" hidden>Fitted candidate <select id="estimateCandidate"></select></label><p id="estimatePoseWarning" role="status" hidden style="color:#edb6be">Rejected fit shown for comparison. Contact or collision limits are exceeded.</p><div id="viewActions" class="data-actions"></div>

      <p class="hint">Purple and gray: hand surface limits · red: measured contacts</p><div id="boundaryDisplay"></div><details id="estimateSettings"><summary>Estimation settings</summary><label>Fit quality <select id="estimateQuality"><option value="quick" selected>Quick placement</option><option value="detailed">Detailed validation</option></select></label><label>Contact threshold <input id="estimateThreshold" type="number" min="0" max="4095" value="200"></label><label>Position tolerance (mm) <input id="estimateTolerance" type="number" min="0.1" max="20" step="0.1" value="3"></label><label>Hand penetration allowance (mm) <input id="estimateHandTolerance" type="number" min="0.1" max="20" step="0.1" value="3"></label><label>Estimation point spacing (mm) <input id="estimatePointSpacing" type="number" min="0" max="20" step="0.1" value="2"></label><p class="hint">Nearby contacts and hand limits share search representatives within this spacing; final checks retain every supplied point. 0 also keeps all search points. Opposing hand surfaces remain separate. A thumb at least 85% closed with measured thumb contact gives a provisional smaller-sensor first guess. Quick placement stops at the first pose passing its fit checks. Both modes check every supplied contact and hand limit against the full CAD with the adjustable hand penetration allowance (default 3 mm). Detailed validation searches more starting poses. Fitting starts with a longitudinal palm-aligned grasp when rigid palm geometry is available: several rolls around the full axis, then free rotation and translation. Hand limits guide the search and use conservative local surface patches requiring calibrated geometry.</p></details>

      <details id="advancedData"><summary>Advanced display & connection</summary></details>`;

    panel.append(app);old.forEach(node=>$('advancedData').append(node));

    old.filter(n=>n.matches('.kicker,h1')).forEach(n=>{n.hidden=true});

    $('modeManual').parentElement.hidden=true;

    $('manualActions').append($('quickOpen'),$('quickClose'));

    $('manualAngles').append($('controls'));$('viewActions').append($('resetView'));

    const connection=$('liveStatus').closest('.connection');if(connection)$('liveData').append(connection);

    if($('dataSource'))$('liveData').append($('dataSource'));

    $('boundaryDisplay').append($('showHandBoundaries').closest('label'));

    $('advancedData').append($('savedCaptures'));$('savedFiles').closest('label').hidden=true;

    const estimateView=document.createElement('section');estimateView.id='estimateView';estimateView.hidden=true;

    estimateView.innerHTML='<iframe title="Captured point cloud and fitted sensor geometries" sandbox="allow-scripts allow-same-origin allow-downloads"></iframe>';

    $('profileWorkspace').append(estimateView);

    const runView=document.createElement('section');runView.id='runCloudView';runView.hidden=true;

    runView.style.cssText='position:absolute;inset:0;z-index:7;background:#101419';

    runView.innerHTML='<div id="runCloudScene" style="width:100%;height:100%"></div>';

    $('profileWorkspace').append(runView);let runPlot=null,runFrame=null,buildVersion=0;

    const base=location.protocol==='file:'?'http://127.0.0.1:18765':location.origin;

    let mode='manual',switching=false,historyValid=false,csvRows=null,csvName='',job=null,pollTimer=null,running=false,loading=false,historyFrame=null,loadVersion=0;

    const status=$('estimateStatus');
    const poseModels=JSON.parse($('jointProfileData').textContent).sensor_objects;
    let feedbackState=null;
    function paintFeedback(state){
      feedbackState=state;const host=$('estimateFeedback');host.hidden=state.status==='idle';
      host.innerHTML=EstimateFeedback.render(state,reasons[state.result?.reason]||state.result?.reason||'',Date.now()/1000,{models:poseModels,frame:fittedFrame?.frame});
      const retry=$('retryEstimateStatus');if(retry)retry.onclick=refresh;
    }
    setInterval(()=>{if(feedbackState?.status==='running'){
      const clock=$('estimateFeedback').querySelector('.fit-time');
      if(clock)clock.textContent=EstimateFeedback.elapsed(feedbackState.started_at,Date.now()/1000);
    }},1000);


    let fittedFrame=null,fittedResult=null,fittedThreshold=200;

    function clearEstimate(){window.HandEstimate?.clear();fittedFrame=null;fittedResult=null;$('estimatePoseWarning').hidden=true;$('showEstimate').disabled=true;$('showFitReport').disabled=true;$('estimateCandidateLabel').hidden=true;}

    function placeEstimate(){

      if(!fittedFrame||!fittedResult)return;

      estimateView.hidden=true;runView.hidden=true;

      const candidate=$('estimateCandidate').value;
      $('estimatePoseWarning').hidden=fittedResult.fits[candidate]?.compatible===true;

      try{

        if(!fittedFrame.hand_snapshot)throw Error('This cloud has no saved hand pose. Open Fit report to inspect its alignment.');

        window.HandHistory.show(fittedFrame.hand_snapshot,'Estimate snapshot');

        historyFrame=fittedFrame;historyValid=true;layoutMode('history');

        window.HandEstimate.show(candidate,fittedResult.fits[candidate],fittedFrame,fittedThreshold);

      }catch(error){window.HandEstimate?.clear();status.textContent+=' '+error.message;}

    }

    $('estimateCandidate').onchange=placeEstimate;

    function updateReady(){

      $('estimateButton').disabled=running||loading||(mode==='history'&&!historyValid);

      $('showCloud').disabled=mode!=='history'||!historyValid;

      $('buildRun').disabled=loading||!csvRows;

      $('downloadRun').hidden=!runFrame||!historyValid;

      $('runLayers').hidden=!runFrame||!historyValid;

    }

    function invalidateHistory(message){

      clearEstimate();historyValid=false;historyFrame=null;estimateView.hidden=true;

      runFrame=null;runView.hidden=true;

      $('historyStatus').textContent=message;updateReady();

    }

    function previewReady(){

      clearEstimate();

      runFrame=null;runView.hidden=true;

      const frame=window.getIdentificationFrame();

      if(!frame.valid||!frame.points.length)throw Error('This file does not contain a usable point cloud.');

      historyFrame=JSON.parse(JSON.stringify(frame));historyValid=true;

      $('selectedHistory').textContent=frame.saved_name||'Saved capture';

      status.textContent='Preview ready · '+frame.points.length+' points. Inspect the hand or point cloud, then press Estimate.';

      updateReady();

    }

    $('chooseHistory').onclick=()=>$('historyFiles').click();

    function layoutMode(next){

      $('receivedRuns').hidden=next==='history';
      mode=next;for(const key of ['manual','live','history']){

        $(key+'Data').hidden=key!==mode;const tab=$('data'+key[0].toUpperCase()+key.slice(1));tab.classList.toggle('active',key===mode);tab.setAttribute('aria-selected',String(key===mode));

      }

      $('sourceInfo').textContent=mode==='manual'?'Set the angles yourself. Pressure is unknown; angles alone cannot identify an object.':mode==='live'?'Current actual angles and pressure from the connected hand.': 'Historical data only. Live updates are paused.';

      updateReady();

    }

    function selectMode(next){
      // A pending live-capture load must not replace the file selected in History.
      if(next==='history'&&receiverLoading){buildVersion++;loading=false;}


      clearEstimate();

      switching=true;try{

        if(next==='manual')$('modeManual').click();

        if(next==='live')$('modeLive').click();

        if(next==='history'){

          if(!window.ProfileTwin.snapshot().replay)$('modeManual').click();

          if(historyValid&&runFrame)historyFrame=runFrame;

          else if(historyValid&&!$('savedSelect').disabled){$('savedSelect').onchange();previewReady()}

        }

        layoutMode(next);estimateView.hidden=true;runView.hidden=true;

      }finally{switching=false}

    }

    for(const key of ['manual','live','history'])$('data'+key[0].toUpperCase()+key.slice(1)).onclick=()=>selectMode(key);

    $('modeManual').addEventListener('click',()=>{if(!switching)layoutMode('manual')});

    $('modeLive').addEventListener('click',()=>{if(!switching)layoutMode('live')});

    $('savedSelect').addEventListener('change',()=>{try{layoutMode('history');estimateView.hidden=true;previewReady()}catch(e){invalidateHistory(e.message)}});

    function showHistory(payload,name){switching=true;try{window.HandHistory.show(payload,name);layoutMode('history');estimateView.hidden=true;previewReady()}finally{switching=false}}

    function showCSV(){

      if(!csvRows)return;

      runFrame=null;runView.hidden=true;

      const row=Number($('historyRow').value);if(!Number.isInteger(row)||row<1||row>csvRows.length)throw Error('Choose a row from 1 to '+csvRows.length+'.');

      const packet=IdentificationHistory.telemetry(csvRows[row-1],JointPressure.regions);

      switching=true;try{window.HandHistory.telemetry(packet,csvName+' · row '+row);layoutMode('history');previewReady()}finally{switching=false}

      $('historyRowInfo').textContent='Row '+row+' / '+csvRows.length+' · index '+packet.index+(packet.state?' · '+packet.state:'');

      $('historyStatus').textContent='CSV uses the saved angles and recorded counts. Offsets and failed-read validity are not supplied.';

      estimateView.hidden=true;

    }

    async function drawRun(){

      if(!runFrame)return;

      estimateView.hidden=true;runView.hidden=false;

      const points=runFrame.points,limits=runFrame.boundary_points||[],visible=$('runShowLimits').checked;

      const layer=(pp,color,size,shown)=>({type:'scatter3d',mode:'markers',visible:shown,x:pp.map(p=>p.x*1000),y:pp.map(p=>p.y*1000),z:pp.map(p=>p.z*1000),marker:{size,color:pp.map(p=>p.pressure||0),...(color?{rgb:pp.map(()=>color)}:{})}});

      const traces=[layer(limits.filter(p=>p.kind==='inactive_taxel'),[.35,.75,1],3,visible),layer(limits.filter(p=>p.kind==='unknown_taxel'),[.58,.64,.72],2,visible),layer(limits.filter(p=>!['inactive_taxel','unknown_taxel'].includes(p.kind)),[.72,.54,1],3,visible),layer(points,null,5,true)];

      // Run points are already in left_base_link metres; draw the base axes in mm.
      const axes=['X','Y','Z'].map((name,axis)=>{
        const vertices=[],origin=[0,0,0],end=[0,0,0];end[axis]=40;
        vertices.push(origin,end);
        for(const perpendicular of [0,1,2].filter(k=>k!==axis))for(const sign of [-1,1]){
          const wing=end.slice();wing[axis]-=4;wing[perpendicular]=sign*2.5;vertices.push(end,wing);
        }
        const color=[[1,89/255,94/255],[73/255,209/255,125/255],[88/255,166/255,1]][axis];
        return {name:'Global '+name,type:'scatter3d',mode:'lines',xray:true,visible:$('runShowAxes').checked,
          x:vertices.map(p=>p[0]),y:vertices.map(p=>p[1]),z:vertices.map(p=>p[2]),marker:{rgb:vertices.map(()=>color)}};
      });
      traces.push(...axes);

      if(!runPlot)runPlot=await HandRenderer.create('runCloudScene',traces);

      else await HandRenderer.restyle(runPlot,{'x':traces.map(t=>t.x),'y':traces.map(t=>t.y),'z':traces.map(t=>t.z),'visible':traces.map(t=>t.visible),'marker.rgb':traces.map(t=>t.marker.rgb||null),'marker.color':traces.map(t=>t.marker.color)},traces.map((_,i)=>i));

      const shown=[...(visible?[...points,...limits]:points.length?points:limits)];
      if($('runShowAxes').checked)for(const trace of axes)trace.x.forEach((x,i)=>shown.push({x:x/1000,y:trace.y[i]/1000,z:trace.z[i]/1000}));
      if(!shown.length)return;

      const center=['x','y','z'].map(k=>shown.reduce((s,p)=>s+p[k]*1000,0)/shown.length);

      const radius=Math.max(...shown.map(p=>Math.hypot(p.x*1000-center[0],p.y*1000-center[1],p.z*1000-center[2])));

      runPlot._renderer.focus(center,[.5,1,.4],Math.max(30,radius*3));

    }

    $('runShowLimits').onchange=()=>{if(runFrame)drawRun()};
    $('runShowAxes').onchange=()=>{if(runFrame)drawRun()};

    async function acceptRun(frame){

      clearEstimate();runFrame=IdentificationRuns.validateFrame(frame);historyFrame=runFrame;historyValid=true;

      layoutMode('history');$('selectedHistory').textContent=frame.saved_name;

      const a=frame.accumulation;

      status.textContent=frame.points.length+' active contacts · '+(frame.boundary_points?.length||0)+' hand-space limits'+(a?' from '+(a.frames_reconstructed??a.frames_used)+' selected angle poses.':'.')+(frame.points.length?' Press Estimate to fit with these limits.':' No contacts: the hand sweep constrains space, but does not uniquely locate the object.');

      $('historyStatus').textContent='Constraints include poses without activation. Inactive readings do not certify empty space beyond the local hand surface.';

      updateReady();await drawRun();

    }

    async function buildRun(){

      if(!csvRows)return;

      const version=++buildVersion;invalidateHistory('Building run contacts…');loading=true;updateReady();

      try{

        const selected=IdentificationRuns.select(csvRows,$('historyRun').value);

        const frame=await IdentificationRuns.buildAsync(selected.rows,window.HandHistory.context(),{

          run_key:selected.run.key,name:csvName+' · '+selected.run.label+' · '+selected.run.first_index+'–'+selected.run.last_index,

          threshold:Number($('estimateThreshold').value),voxel_mm:Number($('runVoxel').value),thumb_only:$('runRegions').value==='thumb',

          cancelled:()=>version!==buildVersion,

          progress:(n,total)=>{$('historyStatus').textContent='Building run: '+n+' / '+total+' samples';}

        });

        const capture=receivedMetadata.get(selected.run.explicit);
        if(capture)frame.accumulation.received_capture=capture;
        if(version===buildVersion)await acceptRun(frame);

      }catch(e){if(version===buildVersion)invalidateHistory(e.message.startsWith('No valid ')?'CSV loaded. '+e.message+' Choose All hand regions for finger contacts, or select a run with thumb contact.':e.message)}

      finally{if(version===buildVersion){loading=false;updateReady()}}

    }

    function chooseScope(){

      buildVersion++;loading=false;

      const whole=$('csvScope').value==='run';$('runControls').hidden=!whole;$('rowControls').hidden=whole;

      if(whole){invalidateHistory('Choose a run, then build its point cloud.');$('historyRowInfo').textContent='Runs remain separate; no registration across grasps is assumed.';}

      else try{showCSV()}catch(e){invalidateHistory(e.message)}

    }

    $('csvScope').onchange=chooseScope;$('buildRun').onclick=buildRun;

    for(const id of ['historyRun','runRegions','runVoxel'])$(id).onchange=()=>{buildVersion++;loading=false;invalidateHistory('Settings changed. Build the run point cloud again.');};

    $('estimateThreshold').onchange=()=>{if(runFrame){buildVersion++;invalidateHistory('Threshold changed. Build the run point cloud again.');}};

    $('downloadRun').onclick=()=>{

      if(!runFrame)return;const url=URL.createObjectURL(new Blob([JSON.stringify(runFrame,null,2)],{type:'application/json'})),a=document.createElement('a');

      a.href=url;a.download='thumb_run_cloud.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000);

    };

    $('historyFiles').onchange=async event=>{

      const file=event.target.files[0];if(!file)return;

      const version=++loadVersion;loading=true;

      buildVersion++;

      invalidateHistory('Loading '+file.name+'…');

      $('selectedHistory').textContent=file.name;

      csvRows=null;$('csvControls').hidden=true;

      try{

        if(file.size>256*1024*1024)throw Error('History files must be at most 256 MB; use individual run CSVs for larger recordings.');

        const text=await file.text();if(version!==loadVersion)return;

        if(file.name.toLowerCase().endsWith('.csv')){

          csvRows=IdentificationHistory.records(text);csvName=file.name;

          const runs=IdentificationRuns.groups(csvRows);

          if(!runs.length)throw Error('CSV contains no non-idle run samples.');

          $('historyRun').replaceChildren(...runs.map(run=>{const o=document.createElement('option');o.value=run.key;o.textContent=run.label+' · '+run.first_index+'–'+run.last_index+' · '+run.count+' samples';return o}));

          $('csvControls').hidden=false;$('historyRow').max=csvRows.length;$('historyRow').value=1;$('csvScope').value='run';chooseScope();

          if(runs.length===1)await buildRun();

        }else{

          const payload=JSON.parse(text);

          if(payload.measurement_source==='csv_run')await acceptRun(payload);

          else{showHistory(payload,file.name);$('historyStatus').textContent='Loaded JSON coordinates and pressure.';}

        }

      }catch(error){if(version===loadVersion)invalidateHistory(file.name+': '+error.message+' Choose a valid file before estimating.')}

      finally{if(version===loadVersion){loading=false;updateReady();event.target.value=''}}

    };

    $('historyRow').onchange=()=>{try{showCSV()}catch(e){invalidateHistory(e.message)}};

    $('historyPrev').onclick=()=>{if(Number($('historyRow').value)>1){$('historyRow').value=Number($('historyRow').value)-1;$('historyRow').onchange()}};

    $('historyNext').onclick=()=>{if(csvRows&&Number($('historyRow').value)<csvRows.length){$('historyRow').value=Number($('historyRow').value)+1;$('historyRow').onchange()}};

    $('showHand').onclick=()=>{estimateView.hidden=true;runView.hidden=true;if(mode==='history'&&historyValid){if(runFrame?.hand_snapshot)window.HandHistory.show(runFrame.hand_snapshot,'Run: last recorded probing pose');$('savedHand').click()}};

    $('showCloud').onclick=()=>{if(mode==='history'&&historyValid){if(runFrame)drawRun();else{estimateView.hidden=true;$('savedCloud').click()}}};

    $('showEstimate').onclick=placeEstimate;

    $('showFitReport').onclick=()=>{const report=estimateView.querySelector('iframe');if(report.dataset.reportUrl&&report.getAttribute('src')!==report.dataset.reportUrl)report.src=report.dataset.reportUrl;estimateView.hidden=false;runView.hidden=true};

    const reasons={quick_placement:'Quick approximate placement from sampled sensor surfaces.',space_constraints_only:'Hand sweep constrains space; object pose is unanchored without contacts.',both_candidates_compatible:'Both shapes fit.',neither_candidate_compatible_within_search:'Neither shape fits sufficiently within this search.',insufficient_contacts:'Not enough valid contact pressure.',insufficient_spatial_coverage:'Contacts cover too little of the object.',calibration_unverified:'Geometry needs calibration before a confident label.',score_margin_too_small:'The score difference is too small.',alignment_search_unstable:'Alignment did not settle reliably.',invalid_or_stale_frame:'No valid synchronized snapshot.',one_candidate_clearly_more_consistent:'One shape fits clearly better.'};

    function display(state){

      if(state.estimator_revision!=='thumb_first_v6')throw Error('Restart the simulation launcher to load the updated estimator, then reload this page.');
      if(!fittedFrame&&state.estimate_frame){
        fittedFrame=state.estimate_frame;
        fittedThreshold=state.estimate_threshold??200;
      }
      paintFeedback(state);

      running=state.status==='running';updateReady();clearTimeout(pollTimer);

      if(state.status==='running'){

        status.textContent='Estimating frozen snapshot: '+(state.input_label||mode)+'. You can keep inspecting the hand.';

        job=state.job_id;pollTimer=setTimeout(refresh,1500);

      }else if(state.status==='complete'){

        const result=state.result,errors=Object.entries(result.fits||{}).map(([name,fit])=>name.replace('SENSOR_','')+': '+(fit.compatible?(fit.quick_preview?'approximate':'valid'):'rejected')+' (max contact '+(fit.max_error_mm??fit.rmse_mm).toFixed(2)+' mm, hand '+(fit.hand_limit_max_mm??0).toFixed(2)+' mm)').join(' · ');

        status.textContent='Estimate complete · '+(reasons[result.reason]||result.reason);

        estimateView.querySelector('iframe').dataset.reportUrl=base+state.report_url+'?job='+state.job_id;

        estimateView.hidden=true;runView.hidden=true;$('showFitReport').disabled=false;

        fittedResult=result;

        const candidates = Object.entries(result.fits || {})

        .filter(([, fit]) =>

          Number.isFinite(fit.score) &&

          Array.isArray(fit.transform)

        )

        .sort((a, b) => {

          if (a[0] === result.placement_candidate) return -1;
          if (b[0] === result.placement_candidate) return 1;
          if (a[1].compatible !== b[1].compatible)

            return a[1].compatible ? -1 : 1;



          return a[1].score - b[1].score;

        });

        $('estimateCandidate').replaceChildren(...candidates.map(([name])=>{const option=document.createElement('option');option.textContent = (poseModels[['SENSOR_A','SENSOR_B'].indexOf(name)]?.name||name) +

  (result.fits[name].compatible ? (result.fits[name].quick_preview ? ' · approximate placement' : ' · compatible') : ' · REJECTED fit');
        option.value=name;return option;}));

        $('estimateCandidateLabel').hidden=candidates.length<2;

        $('showEstimate').disabled=!candidates.length||!fittedFrame?.hand_snapshot;

        if(candidates.length&&fittedFrame?.hand_snapshot){status.textContent+=(result.fits[candidates[0][0]].compatible?' Provisional pose shown in the frozen hand':' Best rejected fit shown in the frozen hand')+(fittedFrame.measurement_source==='csv_run'?' at the last contact sample.':'.');placeEstimate();}

        else{window.HandEstimate?.clear();status.textContent+=result.contacts===0&&result.hand_boundary_points>0?' Swept limits are retained for later contact or pose evidence.':candidates.length?' No reproducible hand pose; see Fit report.':' No valid STL pose; see Fit report for rejected candidates.';}

      }else if(state.status==='error')status.textContent='Estimate failed: '+state.error;

    }

    async function refresh(){try{

      const response=await fetch(base+'/api/identification',{signal:AbortSignal.timeout(10000)}),state=await response.json();

      if(!response.ok)throw Error(state.error||'Service unavailable');

      if(job&&state.job_id!==job){clearEstimate();throw Error('Another estimate replaced this job. Run Estimate again.');}

      display(state);

    }catch(error){running=false;updateReady();status.textContent=error.message;paintFeedback({status:'connection_error',error:error.message,input_label:feedbackState?.input_label})}}

    $('estimateButton').onclick=async()=>{

      if(loading||running)return;

      if(mode==='history'&&!historyValid){status.textContent='Select a valid history frame before estimating.';return}

      const threshold=Number($('estimateThreshold').value),tolerance=Number($('estimateTolerance').value),handTolerance=Number($('estimateHandTolerance').value),pointSpacing=Number($('estimatePointSpacing').value);

      if(!Number.isFinite(threshold)||threshold<0||threshold>4095||!Number.isFinite(tolerance)||tolerance<.1||tolerance>20||!Number.isFinite(handTolerance)||handTolerance<.1||handTolerance>20||!Number.isFinite(pointSpacing)||pointSpacing<0||pointSpacing>20){status.textContent='Check threshold (0–4095), tolerances (0.1–20 mm), and point spacing (0–20 mm).';return}

      window.ProfileTwin.tick();const frame=mode==='history'?historyFrame:window.getIdentificationFrame();

      if(!frame.valid){status.textContent=mode==='live'?'Live angles and pressure are missing, stale or unsynchronized.':'Load a valid history frame or choose a manual pose first.';return}

      const frozen=JSON.parse(JSON.stringify(frame));

      clearEstimate();fittedFrame=frozen;fittedThreshold=threshold;

      const count=frozen.points.filter(p=>typeof p.pressure==='number'&&p.pressure>threshold).length;

      const label=mode==='history'?'History · '+(frame.saved_name||'capture'):mode==='manual'?'Manual pose':'Live hand';

      paintFeedback({status:'running',started_at:Date.now()/1000,input_label:label,progress:{stage:'queued'}});
      status.textContent='Captured '+frozen.points.length+' positions · '+count+' contact points · '+(frozen.boundary_points?.length||0)+' hand limits. Starting estimate…';running=true;updateReady();

      try{

        const check=await fetch(base+'/api/identification',{signal:AbortSignal.timeout(10000)});

        const backend=await check.json();

        if(!check.ok)throw Error(backend.error||'Could not reach estimator');

        if(backend.estimator_revision!=='thumb_first_v6')throw Error('Restart the simulation launcher to load the updated estimator, then reload this page.');

        const response=await fetch(base+'/api/identification',{method:'POST',headers:{'Content-Type':'application/json'},signal:AbortSignal.timeout(15000),body:JSON.stringify({mode:'current',frame:frozen,threshold,tolerance_mm:tolerance,hand_tolerance_mm:handTolerance,point_spacing_mm:pointSpacing,quick_preview:$('estimateQuality').value==='quick',input_label:label})});

        const state=await response.json();if(response.status===409){clearEstimate();throw Error('An estimate is already running. Wait for it to finish, then press Estimate again.');}if(!response.ok)throw Error(state.error||'Could not estimate');job=state.job_id;display(state);

      }catch(error){running=false;updateReady();status.textContent='Estimate failed: '+error.message;paintFeedback({status:'error',error:error.message,input_label:label})}

    };

    let receiverBusy=false,receiverLoading=false,lastAutoRun=null,receivedMetadata=new Map();
    async function openReceivedRun(id){
      if(!id||mode==='history'||loading||running||receiverLoading)return;
      receiverLoading=true;loading=true;updateReady();
      const version=++buildVersion,fileVersion=loadVersion;
      try{
        const response=await fetch(base+'/api/probing-runs/'+encodeURIComponent(id),{signal:AbortSignal.timeout(15000)});
        if(!response.ok)throw Error((await response.json()).error||'Could not load capture');
        const rows=IdentificationHistory.records(await response.text());
        if(mode==='history'||fileVersion!==loadVersion||version!==buildVersion)return;
        const frame=await IdentificationRuns.buildAsync(rows,window.HandHistory.context(),{
          run_key:id,name:'Received probe '+id,threshold:Number($('estimateThreshold').value),
          voxel_mm:Number($('runVoxel').value),thumb_only:$('runRegions').value==='thumb',
          cancelled:()=>version!==buildVersion||mode==='history'||fileVersion!==loadVersion,
          progress:(n,total)=>{$('receivedRunStatus').textContent='Building received run: '+n+' / '+total+' poses';}
        });
        if(version===buildVersion&&mode!=='history'&&fileVersion===loadVersion){
          csvRows=rows;csvName='Received probe '+id;
          const groups=IdentificationRuns.groups(rows);
          $('historyRun').replaceChildren(...groups.map(run=>{const option=document.createElement('option');option.value=run.key;option.textContent=run.label;return option}));
          $('csvControls').hidden=false;$('runControls').hidden=false;$('rowControls').hidden=true;
          $('historyRow').max=rows.length;$('historyRow').value=1;$('csvScope').value='run';
          frame.accumulation.received_capture=receivedMetadata.get(id)||{id};
          await acceptRun(frame);
          $('receivedRunStatus').textContent='Captured run ready · '+rows.length+' received samples. Press Estimate when ready.';
        }
      }catch(error){$('receivedRunStatus').textContent='Capture could not be built: '+error.message;}
      finally{receiverLoading=false;if(version===buildVersion){loading=false;updateReady()}}
    }
    function selectReceivedRun(){
      const id=$('receivedRunSelect').value;
      $('openReceivedRun').disabled=!id;
      $('downloadReceivedCSV').hidden=!id;
      if(id){$('downloadReceivedCSV').href=base+'/api/probing-runs/'+encodeURIComponent(id);$('downloadReceivedCSV').download=id+'.csv';}
    }
    async function captureAction(action,enabled){
      $('startReceivedCapture').disabled=true;$('finishReceivedCapture').disabled=true;
      try{
        const response=await fetch(base+'/api/probing-runs',{method:'POST',headers:{'Content-Type':'application/json'},
          body:JSON.stringify({action,...(enabled===undefined?{}:{enabled})}),signal:AbortSignal.timeout(5000)});
        const data=await response.json();if(!response.ok)throw Error(data.error||'Capture action failed');
      }catch(error){$('receivedRunStatus').textContent=error.message;}
      finally{await pollReceivedRuns();}
    }
    $('captureOnClose').onchange=()=>captureAction('auto',$('captureOnClose').checked);
    $('startReceivedCapture').onclick=()=>captureAction('start');
    $('finishReceivedCapture').onclick=()=>captureAction('finish');
    $('receivedRunSelect').onchange=selectReceivedRun;
    $('openReceivedRun').onclick=()=>openReceivedRun($('receivedRunSelect').value);
    async function pollReceivedRuns(){
      if(receiverBusy||receiverLoading)return;
      receiverBusy=true;
      try{
        const response=await fetch(base+'/api/probing-runs',{signal:AbortSignal.timeout(5000)});
        const data=await response.json();if(!response.ok)throw Error(data.error||'Receiver unavailable');
        $('startReceivedCapture').disabled=!!data.active;
        $('finishReceivedCapture').disabled=!['manual','closure'].includes(data.active?.capture_mode);
        $('captureOnClose').checked=data.auto_on_close;
        $('closureCaptureStatus').textContent=data.active?.capture_mode==='closure'?'Recording finger-motion window. Reopen the hand or press Finish capture.':!data.auto_on_close?'Automatic finger-motion capture is off.':data.closure_armed?'Armed: waiting for the fingers to close.':'Waiting to observe an open hand before arming.';
        receivedMetadata=new Map(data.runs.map(run=>[run.id,run]));
        const previous=$('receivedRunSelect').value,select=$('receivedRunSelect');
        const signature=JSON.stringify(data.runs.map(r=>[r.id,r.status,r.samples]));
        if(select.dataset.signature!==signature){
          select.replaceChildren();
          for(const run of data.runs){const option=document.createElement('option');option.value=run.id;
            option.textContent=run.id+' · '+run.status+' · '+run.samples+' samples';select.append(option);}
          if(data.runs.some(r=>r.id===previous))select.value=previous;
          select.dataset.signature=signature;selectReceivedRun();
        }
        if(data.error)$('receivedRunStatus').textContent='Recording error: '+data.error;
        else if(data.active)$('receivedRunStatus').textContent='Recording probe · '+data.active.samples+' received samples'+(data.active.missing_source_samples?' · '+data.active.missing_source_samples+' source samples missed':'');
        else if(!data.runs.length)$('receivedRunStatus').textContent='Waiting for probing telemetry with state, actual angles and tactile readings.';
        const latest=data.runs.find(r=>r.status==='complete'&&r.samples>0);
        if(mode!=='history'&&latest&&lastAutoRun!==latest.id&&$('autoReceiveRun').checked&&!loading&&!running){
          lastAutoRun=latest.id;select.value=latest.id;selectReceivedRun();await openReceivedRun(latest.id);
          if(latest.missing_source_samples||latest.skipped_samples)$('receivedRunStatus').textContent+=' Incomplete input: '+latest.missing_source_samples+' source samples missed; '+latest.skipped_samples+' invalid samples skipped.';
        }
      }catch(error){$('receivedRunStatus').textContent='Probing receiver unavailable: '+error.message;}
      finally{receiverBusy=false}
    }
    setInterval(pollReceivedRuns,1000);pollReceivedRuns();

    selectMode('manual');
    // Recover feedback for a job that was started before a page reload.
    fetch(base+'/api/identification',{signal:AbortSignal.timeout(10000)})
      .then(async response=>{const state=await response.json();if(!response.ok)throw Error(state.error||'Estimator unavailable');
        if(['running','complete','error'].includes(state.status)){job=state.job_id;display(state)}})
      .catch(error=>paintFeedback({status:'connection_error',error:error.message}));


  }

})();
