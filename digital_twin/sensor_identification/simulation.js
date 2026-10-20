/* ESTIMATE_FEEDBACK_COMPONENT */
/* Estimate feedback; formatters are pure so malformed/missing metrics are safe. */
(function(root){
 'use strict';
 const escape=value=>String(value??'').replace(/[&<>"']/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[c]));
  const elapsed=(start,end)=>{if(!Number.isFinite(start))return '';const seconds=Math.max(0,Math.floor((end-start)));return seconds<60?seconds+'s':Math.floor(seconds/60)+'m '+String(seconds%60).padStart(2,'0')+'s'};
 const sensorName=(name,context={})=>context.models?.[['SENSOR_A','SENSOR_B'].indexOf(name)]?.name||({SENSOR_A:'VEGABAR 29',SENSOR_B:'VEGABAR 38'}[name])||name;
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
 function render(state,explanation='',now=Date.now()/1000,context={}){
  if(state.status==='idle')return '';
  const result=state.result||{},fits=result.fits||{},running=state.status==='running';
  const unresolved=!!result.candidate_comparison;
  const candidate=running?(state.progress?.candidate||state.first_guess?.candidate):
   (context.candidate||(!unresolved&&(result.placement_candidate||result.best_candidate||Object.keys(fits).find(name=>fits[name]?.compatible===true)||Object.keys(fits)[0])));
  const model=context.models?.[['SENSOR_A','SENSOR_B'].indexOf(candidate)];
  const pose=sensorPose(fits[candidate],model);
  const currentStages={screening_axial_roll:'Checking shaft rotation (360°)',refining_known_placement:'Refining usual placement',visible_surface_refining:'Fitting contact surfaces',queued:'Starting',loading_models:'Loading sensors',preparing:'Checking data',thumb_first_guess:'Choosing sensor',standard_placement:'Positioning',adjusting_angle:'Adjusting angle',moving_to_thumb:'Moving to thumb',palm_initialization:'Positioning',alignment:'Matching surfaces',refining:'Refining position',checking_limits:'Checking fit',candidate_checked:'Fit checked',writing_report:'Finishing'};
  const status=running?(currentStages[state.progress?.stage]||'Estimating'):
   state.status==='connection_error'?'Connection lost':state.status==='error'?'Estimate failed':
   unresolved?(candidate?'Comparison only · size unresolved':'Sensor size unresolved'):!candidate?'No position found':fits[candidate]?.placement_usable===true?(fits[candidate]?.placement_quality==='partial_contact_support'?'Partial placement':'Provisional placement'):fits[candidate]?.compatible!==true?'Rejected fit':
   result.classification===candidate?'Identified':'Provisional fit';
  const metric=(label,value,unit)=>'<div><dt>'+label+'</dt><dd>'+(Number.isFinite(value)?escape((Math.abs(value)<.005?0:value).toFixed(2)+' '+unit):'—')+'</dd></div>';
  return '<h3>'+escape(candidate?sensorName(candidate,context):'Sensor')+'</h3><p class="current-state" role="status">'+escape(status)+'</p><dl class="current-pose">'+
   ['X','Y','Z'].map((axis,k)=>metric(axis,pose?.center[k],'mm')).join('')+
   ['pitch','yaw','roll'].map(axis=>metric(axis[0].toUpperCase()+axis.slice(1),pose?.[axis],'°')).join('')+'</dl>'+
   (Number.isFinite(fits[candidate]?.visible_surface_inlier_fraction)?'<p class="fit-small">Contacts within tolerance: '+Math.round(fits[candidate].visible_surface_inlier_fraction*100)+'% of contacts. '+(fits[candidate]?.pose_evidence==='known_placement_finite_cad'?'Near usual insertion; open hand links and sampled collision coverage remain unverified.':'Inspection preview.')+'</p>':'')+
   (fits[candidate]?.weak_contact_regions?.length?'<p class="fit-small">Poorly matched areas: '+escape(fits[candidate].weak_contact_regions.join(', '))+'.</p>':'')+
   (unresolved?'<p class="fit-small">Both sensor sizes fit the contacts similarly. '+(result.candidate_comparison.missing_thumb_contacts?'No measured thumb contacts distinguish the head size. ':'')+'Choose a fitted candidate to compare; no sensor selected automatically.</p>':'')+
   (state.status==='connection_error'?'<button id="retryEstimateStatus">Retry</button>':'');
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
      #onePageApp .data-modes{display:grid;grid-template-columns:repeat(2,1fr);gap:0;margin-bottom:10px;border-bottom:1px solid #444e5a}
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
      #estimateFeedback{position:fixed;top:16px;right:16px;z-index:60;box-sizing:border-box;width:min(340px,calc(100vw - 392px));max-height:calc(100vh - 32px);overflow-y:auto;overscroll-behavior:contain;margin:0;padding:16px;border:1px solid var(--line);border-radius:6px;background:var(--panel);box-shadow:0 8px 24px #0005;scrollbar-width:thin}
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
      #estimateFeedback .sensor-decisions{border:1px solid #536476;background:#202c38;padding:12px;margin:12px 0}#estimateFeedback .sensor-decisions h4{margin:0 0 8px;font-size:12px}#estimateFeedback .sensor-decisions ol{padding-left:18px;margin:0}#estimateFeedback .sensor-decisions li{margin:8px 0;font-size:12px;line-height:1.5}
      #estimateFeedback .current-pose{grid-template-columns:repeat(3,minmax(0,1fr));gap:12px 6px;margin:12px 0 0}#estimateFeedback .current-pose dd{font-size:13px;white-space:nowrap}#estimateFeedback .current-state{margin:4px 0;color:#a6afb9;font-size:12px}
      #estimateFeedback .fit-reason{font-size:12px;color:#edb6be}
      #estimateView{position:absolute;inset:0;z-index:8;background:#fff}#estimateView[hidden]{display:none}#estimateView iframe{width:100%;height:100%;border:0}
      @media(min-width:621px) and (max-width:900px){#estimateFeedback{width:min(340px,calc(100vw - 342px))}#panel{width:310px;padding:22px 18px}body.profile-mode #profileWorkspace{left:310px}}
      @media(max-width:620px){#estimateFeedback{top:calc(50vh + 16px);width:calc(100vw - 32px);max-height:calc(50vh - 32px)}#panel{width:100%;height:50vh;max-height:50vh;padding:18px;border-right:0;border-bottom:1px solid var(--line)}body.profile-mode #profileWorkspace{left:0;top:50vh}body.profile-mode .profile-caption{top:14px;left:18px}#onePageApp .app-heading{margin-bottom:18px;padding-bottom:14px}#onePageApp .app-heading h1{font-size:19px}}
    </style><header class="app-heading"><h1>Hand twin</h1><p>RH56DFTP / Sensor identification</p></header><h2>Data source</h2>

      <div class="data-modes" role="tablist" aria-label="Data mode">

        <button role="tab" id="dataLive">Live</button><button role="tab" id="dataHistory">History</button>

      </div><p id="sourceInfo" class="hint"></p>

      <section id="manualData"><div class="data-actions" id="manualActions"></div><section id="manualAngles" aria-label="Finger positions"><h2>Finger positions</h2></section></section>

      <section id="liveData" hidden><label><input id="followTelemetry" type="checkbox" checked> Follow live telemetry</label></section>

      <section id="historyData" hidden><button id="chooseHistory" type="button">Choose JSON or CSV…</button><input id="historyFiles" type="file" accept=".json,.csv,application/json,text/csv" hidden><p id="selectedHistory" class="hint">No history file selected.</p>

        <div id="csvControls" hidden>

        <div id="runControls"><label>Run <select id="historyRun"></select></label><label>Contacts <select id="runRegions"><option value="all">All hand regions</option><option value="thumb">Thumb only</option></select></label><label>Point spacing (mm) <input id="runVoxel" type="number" min="0.1" max="10" step="0.1" value="1"></label><button id="buildRun">Build run point cloud</button><p class="hint">Shows each contacting finger at its first contact pose, removing its other angles from the cloud. Thumb and palm retain their recorded poses. Estimation retains the full hand sweep. Coordinates assume the object stays fixed relative to the hand base.</p></div>

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

      <div class="data-actions"><button id="showHand">Hand view</button><button id="showCloud" disabled>Run overview</button><button id="showEstimate" disabled>Estimate in hand</button><button id="showFitReport" disabled>Fit report</button></div><label id="estimateCandidateLabel" hidden>Fitted candidate <select id="estimateCandidate"></select></label><section id="estimatePreview" hidden aria-label="Estimation pose history"><label><input id="estimateFollow" type="checkbox" checked> Follow live STL position</label><p id="estimatePreviewStatus" role="status"></p><label>Inspect search poses <input id="estimatePoseStep" type="range" min="0" max="0" value="0" step="1"></label><p class="hint">Trial pose · not yet validated. Drag the slider to pause live following and inspect earlier positions.</p></section><p id="estimatePoseWarning" role="status" hidden style="color:#edb6be">Rejected fit shown for comparison. Contact or collision limits are exceeded.</p><div id="viewActions" class="data-actions"></div>

      <p class="hint">Purple and gray: hand surface limits · red: measured contacts</p><div id="boundaryDisplay"></div><details id="estimateSettings"><summary>Estimation settings</summary><label>Fit quality <select id="estimateQuality"><option value="quick" selected>Quick placement</option><option value="detailed">Detailed validation</option></select></label><label>Contact threshold <input id="estimateThreshold" type="number" min="0" max="4095" value="200"></label><label>Position tolerance (mm) <input id="estimateTolerance" type="number" min="0.1" max="20" step="0.1" value="3"></label><label>Hand penetration allowance (mm) <input id="estimateHandTolerance" type="number" min="0.1" max="20" step="0.1" value="3"></label><label>Estimation point spacing (mm) <input id="estimatePointSpacing" type="number" min="0" max="20" step="0.1" value="2"></label><p class="hint">Nearby contacts and hand limits share one representative within this spacing; 0 keeps all points. Opposing hand surfaces remain separate. A thumb at least 85% closed with measured thumb contact gives a provisional smaller-sensor first guess. Known sensors are fitted near their usual in-hand placement, with limited position and angle adjustments. Quick uses a short local search; Detailed tries additional local starts. A provisional placement does not confirm sensor identity. Known-sensor fits check exact contact rays and sampled penetration into closed hand CAD parts at the frozen pose and selected recorded finger/thumb poses. Similar fits require an explicit model choice for comparison. Open hand parts and continuous collision clearance remain unverified. The hand allowance defaults to 3 mm. Detailed fitting tries several palm-aligned orientations, then refines rotation and translation. Hand limits guide the search and use conservative local surface patches requiring calibrated geometry.</p></details>

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

    const viewportTools=document.createElement('div');viewportTools.id='viewportTools';
    viewportTools.innerHTML='<label><input id="handModelVisible" type="checkbox" checked> Hand model</label>';
    const legend=document.createElement('details');legend.id='pointLegend';legend.open=true;
    legend.innerHTML='<summary>Legend</summary><span style="color:#ff766b">● Contacts</span><span style="color:#8dd3ee">● Taxel positions</span><span style="color:#b88aff">● Hand limits</span>';
    const timeline=document.createElement('section');timeline.id='historyTimeline';timeline.hidden=true;
    timeline.innerHTML='<button id="timelinePrev" aria-label="Previous row">‹</button><label for="historyTime">History <output id="historyTimeLabel"></output></label><input id="historyTime" aria-label="History row" type="range" min="1" max="1" value="1" step="1"><button id="timelineNext" aria-label="Next row">›</button>';
    document.body.append(viewportTools,legend,timeline);
    const compact=document.createElement('style');compact.textContent=`
      body #panel{width:310px;padding:18px}html body.profile-mode #profileWorkspace{left:310px}
      #onePageApp #sourceInfo,#onePageApp .app-heading p,.profile-caption p,.profile-angles,#rowControls,#historyRowInfo{display:none!important}
      #onePageApp #runControls>.hint,#onePageApp #runLayers>.hint,#onePageApp #receivedRuns>.hint{display:none}
      #viewportTools{position:fixed;top:16px;left:330px;z-index:70;background:#191e24e8;border:1px solid #343c46;border-radius:6px;padding:8px 12px;font-size:12px}
      #pointLegend{position:fixed;right:16px;bottom:80px;z-index:70;background:#191e24e8;border:1px solid #343c46;border-radius:6px;padding:9px 12px;font-size:12px}
      #pointLegend summary{cursor:pointer;color:#c4cbd3}#pointLegend span{display:block;margin-top:5px}
      #historyTimeline{position:fixed;left:330px;right:16px;bottom:14px;z-index:70;display:flex;align-items:center;gap:12px;padding:10px 14px;background:#191e24ed;border:1px solid #343c46;border-radius:6px;font-size:12px}
      #historyTimeline[hidden]{display:none}
      #historyTimeline>label{flex:0 0 38%;max-width:360px;min-width:0;overflow:hidden;white-space:nowrap;text-overflow:ellipsis;font-size:12px;line-height:18px;font-variant-numeric:tabular-nums}
      #historyTime{flex:1;min-width:40px;accent-color:#98b9dc}
      #historyTimeLabel{font:inherit;white-space:nowrap;font-variant-numeric:tabular-nums}
      #historyTimeline button{background:#252c34;color:#e3e7ec;border:1px solid #444e5a;border-radius:4px;font-size:20px;cursor:pointer}
      .profile-caption{display:none}
      /* Keep all six channels visible, including at 150% desktop scaling. */
      body #onePageApp .app-heading{margin:0 0 8px;padding:0 0 6px}
      body #onePageApp .app-heading h1{font-size:18px;margin:0}
      body #onePageApp h2{margin:8px 0 4px}
      body #onePageApp .data-modes{margin-bottom:6px}
      body #onePageApp .data-modes button{min-height:28px;padding:4px}
      body #onePageApp label{margin:5px 0}
      body #onePageApp #dataSource{display:none}
      #manualActions{margin:8px 0}
      body #onePageApp #manualActions button{min-height:28px;padding:4px 8px}
      body #onePageApp #controls{gap:0;margin-top:4px}
      body #onePageApp .channel{display:grid;grid-template-columns:minmax(0,1fr) 60px;gap:1px 8px;padding:5px 0}
      body #onePageApp .channel-head,body #onePageApp .channel-main{display:contents}
      body #onePageApp .channel-name{grid-column:1;grid-row:1;font-size:12px;line-height:16px}
      body #onePageApp .channel-head .channel-info{display:none}
      body #onePageApp .channel input[type=range]{grid-column:1;grid-row:2;width:100%;height:18px;margin:0}
      body #onePageApp .channel input[type=number]{grid-column:2;grid-row:1 / span 2;width:60px;height:32px;align-self:center;padding:3px;font-size:12px}
      body #onePageApp .channel .tactile{grid-column:1 / -1;grid-row:3;height:2px;margin:0}
      body #onePageApp .channel>.channel-info{grid-column:1 / -1;grid-row:4;margin:0!important;font-size:10px;line-height:12px}
      body #estimateFeedback{top:8px;right:8px;width:clamp(230px,22vw,320px);max-height:calc(100vh - 180px);padding:14px;box-shadow:none}
      #pointLegend{right:8px;bottom:8px}
      body:has(#historyTimeline:not([hidden])) #pointLegend{bottom:80px}
      /* The canvas gets its own clear area beside the result card. */
      @media(min-width:1100px){body.profile-mode #profileCard0{margin-right:calc(clamp(230px,22vw,320px) + 16px)}}
      @media(min-width:621px) and (max-height:650px){body #panel{padding:10px 18px}}
      @media(max-width:620px){body #estimateFeedback{top:calc(50vh + 8px);right:8px;width:min(230px,calc(100vw - 24px));max-height:calc(50vh - 90px)}body #onePageApp #controls{grid-template-columns:repeat(2,minmax(0,1fr));column-gap:12px}}
      @media(max-width:620px){body #panel{width:100%}html body.profile-mode #profileWorkspace{left:0}#viewportTools{left:12px;top:calc(50vh + 10px)}#historyTimeline{left:12px;gap:6px}#pointLegend{bottom:78px}}
    `;document.head.append(compact);
    requestAnimationFrame(()=>$('resetView').click());
    $('handModelVisible').onchange=()=>window.HandHistory.setHandVisible($('handModelVisible').checked);
    // Manual pose controls and telemetry share one Live workspace.
    $('liveData').append($('manualData'));
    const captureDetails=document.createElement('details');captureDetails.innerHTML='<summary>Recordings</summary>';
    $('receivedRuns').before(captureDetails);captureDetails.append($('receivedRuns'));
    const layerDetails=document.createElement('details');layerDetails.innerHTML='<summary>Display settings</summary>';
    $('boundaryDisplay').before(layerDetails);layerDetails.append($('boundaryDisplay'),$('runLayers'));
    for(const p of app.querySelectorAll(':scope > p.hint'))p.hidden=true;
    const runSettings=document.createElement('details');runSettings.id='runSettings';runSettings.innerHTML='<summary>Run settings</summary>';
    $('runControls').append(runSettings);
    runSettings.append($('runRegions').closest('label'),$('runVoxel').closest('label'),$('buildRun'),$('historyStatus'));
    const activity=document.createElement('details');activity.innerHTML='<summary>Status details</summary>';
    $('estimateStatus').before(activity);activity.append($('estimateStatus'));


    const base=location.protocol==='file:'?'http://127.0.0.1:18765':location.origin;

    let mode='manual',switching=false,historyValid=false,csvRows=null,csvName='',job=null,pollTimer=null,running=false,loading=false,historyFrame=null,loadVersion=0;
    let restoringEstimate=true;

    const status=$('estimateStatus');
    const poseModels=JSON.parse($('jointProfileData').textContent).sensor_objects;
    let feedbackState=null;
    function paintFeedback(state){
      feedbackState=state;const host=$('estimateFeedback');host.hidden=state.status==='idle';
      host.innerHTML=EstimateFeedback.render(state,reasons[state.result?.reason]||state.result?.reason||'',Date.now()/1000,{models:poseModels,frame:fittedFrame?.frame,candidate:fittedResult===state.result?$('estimateCandidate').value:null});
      const retry=$('retryEstimateStatus');if(retry)retry.onclick=refresh;
    }
    setInterval(()=>{if(feedbackState?.status==='running'){
      const clock=$('estimateFeedback').querySelector('.fit-time');
      if(clock)clock.textContent=EstimateFeedback.elapsed(feedbackState.started_at,Date.now()/1000);
    }},1000);


    let fittedFrame=null,fittedResult=null,fittedThreshold=200;

    let previewHistory=[],previewSequence=null,previewHandReady=false;
    function showPreviewPose(pose){
      if(!pose||!fittedFrame?.hand_snapshot)return;
      try{
        if(!previewHandReady){
          window.HandHistory.show(fittedFrame.hand_snapshot,'Estimation: frozen hand');
          previewHandReady=true;
        }
        estimateView.hidden=true;runView.hidden=true;
        window.HandEstimate.show(pose.candidate,{transform:pose.transform},fittedFrame,fittedThreshold,{preserveCamera:true});
        const index=['SENSOR_A','SENSOR_B'].indexOf(pose.candidate),name=poseModels[index]?.name||pose.candidate;
        $('estimatePreviewStatus').textContent=name+' · '+pose.stage.replaceAll('_',' ')+' · '+Number(pose.elapsed_seconds||0).toFixed(1)+' s · trial pose';
        $('estimatePoseWarning').hidden=true;
        previewSequence=pose.sequence;
      }catch(error){$('estimatePreviewStatus').textContent='Preview unavailable: '+error.message;}
    }
    function updatePreview(state){
      if(!fittedFrame||!Array.isArray(state.pose_history)||!state.pose_history.length)return;
      const selected=previewHistory[Number($('estimatePoseStep').value)]?.sequence;
      previewHistory=state.pose_history;
      $('estimatePreview').hidden=false;
      $('estimatePoseStep').max=previewHistory.length-1;
      if($('estimateFollow').checked&&state.status==='running'){
        $('estimatePoseStep').value=previewHistory.length-1;
        const latest=previewHistory.at(-1);
        if(latest.sequence!==previewSequence)showPreviewPose(latest);
      }else if(selected!=null){
        const index=previewHistory.findIndex(p=>p.sequence===selected);
        $('estimatePoseStep').value=Math.max(0,index);
      }
    }
    $('estimatePoseStep').oninput=()=>{
      $('estimateFollow').checked=false;
      showPreviewPose(previewHistory[Number($('estimatePoseStep').value)]);
    };
    $('estimateFollow').onchange=()=>{
      if($('estimateFollow').checked&&previewHistory.length){
        $('estimatePoseStep').value=previewHistory.length-1;
        showPreviewPose(previewHistory.at(-1));
      }
    };

    function clearEstimate(){previewHistory=[];previewSequence=null;previewHandReady=false;$('estimatePreview').hidden=true;$('estimateFollow').checked=true;window.HandEstimate?.clear();fittedFrame=null;fittedResult=null;$('estimatePoseWarning').hidden=true;$('showEstimate').disabled=true;$('showFitReport').disabled=true;$('estimateCandidateLabel').hidden=true;}

    function placeEstimate(){

      if(!fittedFrame||!fittedResult)return;

      estimateView.hidden=true;runView.hidden=true;

      const candidate=$('estimateCandidate').value;
      if(!candidate){window.HandEstimate?.clear();return;}
      $('estimatePoseWarning').hidden=fittedResult.fits[candidate]?.compatible===true;
      $('estimatePoseWarning').textContent=fittedResult.candidate_comparison?'Model comparison only; sensor size is unresolved.':fittedResult.fits[candidate]?.placement_usable?'Provisional placement: identity, open hand links and continuous collision clearance are unverified.':'Rejected fit: contact or finite hand penetration limits exceeded.';

      try{

        if(!fittedFrame.hand_snapshot)throw Error('This cloud has no saved hand pose. Open Fit report to inspect its alignment.');

        window.HandHistory.show(fittedFrame.hand_snapshot,'Estimate snapshot');

        historyFrame=fittedFrame;historyValid=true;layoutMode('history');

        window.HandEstimate.show(candidate,fittedResult.fits[candidate],fittedFrame,fittedThreshold);

      }catch(error){window.HandEstimate?.clear();status.textContent+=' '+error.message;}

    }

    $('estimateCandidate').onchange=()=>{placeEstimate();if(feedbackState)paintFeedback(feedbackState)};

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

      next=next==='manual'?'live':next;
      $('receivedRuns').hidden=next==='history';$('receivedRuns').parentElement.hidden=next==='history';
      $('manualData').hidden=next==='history';
      $('historyTimeline').hidden=next!=='history'||!csvRows;
      mode=next;for(const key of ['live','history']){

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

        if(next==='live')$($('followTelemetry').checked?'modeLive':'modeManual').click();

        if(next==='history'){

          if(!window.HandHistory.isReplaying())$('modeManual').click();

          if(historyValid&&runFrame){historyFrame=runFrame;window.HandHistory.show(runFrame.hand_snapshot,runFrame.saved_name);}

          else if(historyValid&&!$('savedSelect').disabled){$('savedSelect').onchange();previewReady()}

        }

        layoutMode(next);estimateView.hidden=true;runView.hidden=true;if(next==='history'&&runFrame)drawRun();

      }finally{switching=false}

    }

    for(const key of ['live','history'])$('data'+key[0].toUpperCase()+key.slice(1)).onclick=()=>selectMode(key);

    $('followTelemetry').onchange=()=>selectMode('live');
    $('modeManual').addEventListener('click',()=>{if(!switching)layoutMode('manual')});

    $('modeLive').addEventListener('click',()=>{if(!switching)layoutMode('live')});

    $('savedSelect').addEventListener('change',()=>{try{layoutMode('history');estimateView.hidden=true;previewReady()}catch(e){invalidateHistory(e.message)}});

    function showHistory(payload,name){switching=true;try{window.HandHistory.show(payload,name);layoutMode('history');estimateView.hidden=true;previewReady()}finally{switching=false}}

    let historySteps=[];
    function timelineLabel(step){
      const item=historySteps[step-1];if(!item)return;
      const state=csvRows[item.row-1].state;
      $('historyTimeLabel').textContent='Pose '+step+' / '+historySteps.length+' · row '+item.row+' / '+csvRows.length+(state?' · '+state:'');
      $('historyTimeLabel').parentElement.title='History '+$('historyTimeLabel').textContent;
      $('historyTime').setAttribute('aria-valuetext','Pose '+step+' of '+historySteps.length+', CSV row '+item.row);
    }
    function configureTimeline(){
      const visible=mode==='history'&&!!csvRows;
      $('historyTimeline').hidden=!visible;
      historySteps=[];if(!visible)return;
      const selected=IdentificationRuns.select(csvRows,$('historyRun').value).run;
      // Collapse held poses by the six actual angles. Pressure/state changes
      // do not add a pose; returning to a pose after moving remains navigable.
      let previous=null;
      for(let i=selected.start;i<=selected.end;i++){
        const angles=IdentificationHistory.actualAngles(csvRows[i]);
        if(!previous||angles.some((value,k)=>value!==previous[k]))historySteps.push({row:i+1,angles});
        previous=angles;
      }
      $('historyTime').min=1;$('historyTime').max=historySteps.length;
      const poseRow=runFrame?.hand_snapshot?.pose_source_row??selected.end+1;
      let step=1;historySteps.forEach((item,i)=>{if(item.row<=poseRow)step=i+1});
      $('historyTime').value=step;timelineLabel(step);
    }
    let scrubQueued=false,pendingHistoryStep=null;
    function scrubHistory(){
      if(!csvRows||!runFrame||mode!=='history')return;
      const step=Number($('historyTime').value);
      if(!historySteps[step-1])return;
      pendingHistoryStep=historySteps[step-1];timelineLabel(step);
      if(scrubQueued)return;scrubQueued=true;
      const currentRun=runFrame;
      requestAnimationFrame(()=>{
        scrubQueued=false;if(!csvRows||runFrame!==currentRun||mode!=='history')return;
        window.HandHistory.setPose(pendingHistoryStep.angles);
      });
    }
    $('historyTime').oninput=scrubHistory;
    for(const [id,delta] of [['timelinePrev',-1],['timelineNext',1]])$(id).onclick=()=>{
      const slider=$('historyTime');slider.value=Math.max(Number(slider.min),Math.min(Number(slider.max),Number(slider.value)+delta));scrubHistory();
    };

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
      estimateView.hidden=true;runView.hidden=true;
      if(runFrame.hand_snapshot&&!window.HandHistory.isReplaying())
        window.HandHistory.show(runFrame.hand_snapshot,runFrame.saved_name);
      window.HandHistory.runOverlay(runFrame,$('runShowLimits').checked,$('runShowAxes').checked);
    }

    $('runShowLimits').onchange=()=>{if(runFrame)drawRun()};
    $('runShowAxes').onchange=()=>{if(runFrame)drawRun()};

    async function acceptRun(frame){

      clearEstimate();runFrame=IdentificationRuns.validateFrame(frame);historyFrame=runFrame;historyValid=true;

      layoutMode('history');$('selectedHistory').textContent=frame.saved_name;

      const a=frame.accumulation;

      status.textContent=frame.points.length+' active contacts · '+(frame.boundary_points?.length||0)+' hand-space limits'+(a?' from '+(a.frames_reconstructed??a.frames_used)+' selected angle poses.':'.')+(frame.points.length?' Press Estimate to fit with these limits.':' No contacts: the hand sweep constrains space, but does not uniquely locate the object.');

      $('historyStatus').textContent='Constraints include poses without activation. Inactive readings do not certify empty space beyond the local hand surface.';

      updateReady();
      if(frame.hand_snapshot){switching=true;try{window.HandHistory.show(frame.hand_snapshot,frame.saved_name)}finally{switching=false}}
      configureTimeline();await drawRun();

    }

    let selectedFileCapture=null;
    async function buildRun(){

      if(!csvRows)return;

      const version=++buildVersion;invalidateHistory('Building run contacts…');loading=true;updateReady();

      try{

        const selected=IdentificationRuns.select(csvRows,$('historyRun').value);

        const frame=await IdentificationRuns.buildAsync(selected.rows,window.HandHistory.context(),{

          run_key:selected.run.key,name:csvName+' · '+selected.run.label+' · '+selected.run.first_index+'–'+selected.run.last_index,

          threshold:Number($('estimateThreshold').value),voxel_mm:Number($('runVoxel').value),thumb_limit_selection:'peak',thumb_only:$('runRegions').value==='thumb',

          cancelled:()=>version!==buildVersion,

          progress:(n,total)=>{$('historyStatus').textContent='Building run: '+n+' / '+total+' samples';}

        });

        const capture=selectedFileCapture?.id===selected.run.explicit?selectedFileCapture:receivedMetadata.get(selected.run.explicit);
        if(capture)frame.accumulation.received_capture=capture;
        if(version===buildVersion)await acceptRun(frame);

      }catch(e){if(version===buildVersion)invalidateHistory(e.message.startsWith('No valid ')?'CSV loaded. '+e.message+' Choose All hand regions for finger contacts, or select a run with thumb contact.':e.message)}

      finally{if(version===buildVersion){loading=false;updateReady()}}

    }

    function chooseScope(){

      buildVersion++;loading=false;

      const whole=true;$('runControls').hidden=!whole;$('rowControls').hidden=whole;

      if(whole){invalidateHistory('Choose a run, then build its point cloud.');$('historyRowInfo').textContent='Runs remain separate; no registration across grasps is assumed.';}

      else try{showCSV()}catch(e){invalidateHistory(e.message)}

    }

    $('buildRun').onclick=buildRun;

    for(const id of ['historyRun','runRegions','runVoxel'])$(id).onchange=()=>buildRun();

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

      csvRows=null;selectedFileCapture=null;$('csvControls').hidden=true;

      try{

        if(file.size>256*1024*1024)throw Error('History files must be at most 256 MB; use individual run CSVs for larger recordings.');

        const text=await file.text();if(version!==loadVersion)return;

        const isCSV=file.name.toLowerCase().endsWith('.csv');
        const payload=isCSV?null:JSON.parse(text);
        const capture=payload&&typeof payload.id==='string'&&/^\d{8}T\d{6}Z_[a-f0-9]{12}$/.test(payload.id)&&
          ['recording','complete','interrupted'].includes(payload.status)&&Number.isInteger(payload.samples);
        let csvText=text;
        if(capture){
          if(payload.status==='recording')throw Error('This run is still recording. Finish it before loading.');
          const response=await fetch(base+'/api/probing-runs/'+encodeURIComponent(payload.id),{signal:AbortSignal.timeout(15000)});
          if(version!==loadVersion)return;
          if(!response.ok)throw Error('The matching recording is unavailable on this receiver. Choose '+payload.id+'.csv from the same folder.');
          csvText=await response.text();if(version!==loadVersion)return;
          selectedFileCapture=payload;
        }
        if(isCSV||capture){

          csvRows=IdentificationHistory.records(csvText);csvName=capture?payload.id+'.csv':file.name;

          const runs=IdentificationRuns.groups(csvRows);

          if(!runs.length)throw Error('CSV contains no non-idle run samples.');

          $('historyRun').replaceChildren(...runs.map(run=>{const o=document.createElement('option');o.value=run.key;o.textContent=run.label+' · '+run.first_index+'–'+run.last_index+' · '+run.count+' samples';return o}));

          $('csvControls').hidden=false;$('historyRow').max=csvRows.length;$('historyRow').value=1;chooseScope();

          await buildRun();

        }else{

          if(payload.measurement_source==='csv_run')await acceptRun(payload);

          else{showHistory(payload,file.name);$('historyStatus').textContent='Loaded JSON coordinates and pressure.';}

        }

      }catch(error){if(version===loadVersion)invalidateHistory(file.name+': '+error.message+' Choose a valid file before estimating.')}

      finally{if(version===loadVersion){loading=false;updateReady();event.target.value=''}}

    };

    $('historyRow').onchange=()=>{try{showCSV()}catch(e){invalidateHistory(e.message)}};

    $('historyPrev').onclick=()=>{if(Number($('historyRow').value)>1){$('historyRow').value=Number($('historyRow').value)-1;$('historyRow').onchange()}};

    $('historyNext').onclick=()=>{if(csvRows&&Number($('historyRow').value)<csvRows.length){$('historyRow').value=Number($('historyRow').value)+1;$('historyRow').onchange()}};

    $('showHand').hidden=true;

    $('showCloud').onclick=()=>{if(mode==='history'&&historyValid){if(runFrame)drawRun();else{estimateView.hidden=true;$('savedCloud').click()}}};

    $('showEstimate').onclick=placeEstimate;

    $('showFitReport').onclick=()=>{const report=estimateView.querySelector('iframe');if(report.dataset.reportUrl&&report.getAttribute('src')!==report.dataset.reportUrl)report.src=report.dataset.reportUrl;estimateView.hidden=false;runView.hidden=true};

    const reasons={sensor_size_unresolved:'Both sensor sizes fit similarly; compare both models',known_placement_provisional:'Provisional fit near usual insertion; identity unconfirmed',quick_placement:'Quick approximate placement from sampled sensor surfaces.',space_constraints_only:'Hand sweep constrains space; object pose is unanchored without contacts.',both_candidates_compatible:'Both shapes fit.',neither_candidate_compatible_within_search:'Neither shape fits sufficiently within this search.',insufficient_contacts:'Not enough valid contact pressure.',insufficient_spatial_coverage:'Contacts cover too little of the object.',calibration_unverified:'Geometry needs calibration before a confident label.',score_margin_too_small:'The score difference is too small.',alignment_search_unstable:'Alignment did not settle reliably.',invalid_or_stale_frame:'No valid synchronized snapshot.',one_candidate_clearly_more_consistent:'One shape fits clearly better.'};

    function display(state){

      if(state.estimator_revision!=='small_sensor_axial_start_v20')throw Error('Restart the simulation launcher to load the updated estimator, then reload this page.');
      paintFeedback(state);
      updatePreview(state);

      running=state.status==='running';updateReady();clearTimeout(pollTimer);

      if(state.status==='running'){

        status.textContent='Estimating frozen snapshot: '+(state.input_label||mode)+'. You can keep inspecting the hand.';

        job=state.job_id;pollTimer=setTimeout(refresh,400);

      }else if(state.status==='complete'){
        $('estimatePreview').hidden=true;$('estimateFollow').checked=false;

        const result=state.result,errors=Object.entries(result.fits||{}).map(([name,fit])=>name.replace('SENSOR_','')+': '+(fit.placement_usable?'provisional':fit.compatible?(fit.quick_preview?'approximate':'valid'):'rejected')+' (max contact '+(fit.max_error_mm??fit.rmse_mm).toFixed(2)+' mm, hand '+(fit.hand_limit_max_mm??0).toFixed(2)+' mm)').join(' · ');

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

  (result.fits[name].placement_usable ? (result.fits[name].placement_quality==='partial_contact_support'?' · partial placement':' · provisional placement') : result.fits[name].compatible ? (result.fits[name].quick_preview ? ' · approximate placement' : ' · compatible') : ' · REJECTED fit');
        option.value=name;return option;}));

        if(result.candidate_comparison){
          const choice=document.createElement('option');choice.value='';choice.textContent='Choose model to compare · size unresolved';
          $('estimateCandidate').prepend(choice);$('estimateCandidate').value='';
        }
        $('estimateCandidateLabel').hidden=candidates.length<2;

        $('showEstimate').disabled=!candidates.length||!fittedFrame?.hand_snapshot;

        if(result.candidate_comparison){window.HandEstimate?.clear();$('estimatePoseWarning').hidden=true;status.textContent+=' Sensor size unresolved. Choose either fitted candidate for comparison.';}
        else if(candidates.length&&(result.fits[candidates[0][0]].compatible||result.fits[candidates[0][0]].placement_usable)&&fittedFrame?.hand_snapshot){status.textContent+=' Provisional pose shown in the frozen hand'+(fittedFrame.measurement_source==='csv_run'?' at the last contact sample.':'.');placeEstimate();}
        else if(candidates.length&&fittedFrame?.hand_snapshot){window.HandEstimate?.clear();$('estimatePoseWarning').hidden=true;status.textContent+=' No valid placement found. Rejected poses are available for explicit inspection with Estimate in hand.';}

        else{window.HandEstimate?.clear();status.textContent+=result.contacts===0&&result.hand_boundary_points>0?' Swept limits are retained for later contact or pose evidence.':candidates.length?' No reproducible hand pose; see Fit report.':' No valid STL pose; see Fit report for rejected candidates.';}

      }else if(state.status==='error'){window.HandEstimate?.clear();status.textContent='Estimate failed: '+state.error;}

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

        if(backend.estimator_revision!=='small_sensor_axial_start_v20')throw Error('Restart the simulation launcher to load the updated estimator, then reload this page.');

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
          voxel_mm:Number($('runVoxel').value),thumb_limit_selection:'peak',thumb_only:$('runRegions').value==='thumb',
          cancelled:()=>version!==buildVersion||mode==='history'||fileVersion!==loadVersion,
          progress:(n,total)=>{$('receivedRunStatus').textContent='Building received run: '+n+' / '+total+' poses';}
        });
        if(version===buildVersion&&mode!=='history'&&fileVersion===loadVersion){
          csvRows=rows;csvName='Received probe '+id;
          const groups=IdentificationRuns.groups(rows);
          $('historyRun').replaceChildren(...groups.map(run=>{const option=document.createElement('option');option.value=run.key;option.textContent=run.label;return option}));
          $('csvControls').hidden=false;$('runControls').hidden=false;$('rowControls').hidden=true;
          $('historyRow').max=rows.length;$('historyRow').value=1;
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
        if(!restoringEstimate&&mode!=='history'&&latest&&lastAutoRun!==latest.id&&$('autoReceiveRun').checked&&!loading&&!running){
          lastAutoRun=latest.id;select.value=latest.id;selectReceivedRun();await openReceivedRun(latest.id);
          if(latest.missing_source_samples||latest.skipped_samples)$('receivedRunStatus').textContent+=' Incomplete input: '+latest.missing_source_samples+' source samples missed; '+latest.skipped_samples+' invalid samples skipped.';
        }
      }catch(error){$('receivedRunStatus').textContent='Probing receiver unavailable: '+error.message;}
      finally{receiverBusy=false}
    }
    setInterval(pollReceivedRuns,1000);pollReceivedRuns();

    selectMode('live');
    // Recover feedback for a job that was started before a page reload.
    fetch(base+'/api/identification',{signal:AbortSignal.timeout(10000)})
      .then(async response=>{const state=await response.json();if(!response.ok)throw Error(state.error||'Estimator unavailable');
        if(['running','complete','error'].includes(state.status)){
          if(state.estimate_frame?.valid===true&&state.estimate_frame.hand_snapshot){
            fittedFrame=JSON.parse(JSON.stringify(state.estimate_frame));
            fittedThreshold=Number.isFinite(state.estimate_threshold)?state.estimate_threshold:200;
            historyFrame=fittedFrame;historyValid=true;
            if(fittedFrame.measurement_source==='csv_run')runFrame=fittedFrame;
            $('selectedHistory').textContent=fittedFrame.saved_name||state.input_label||'Restored estimate snapshot';
            window.HandHistory.show(fittedFrame.hand_snapshot,'Restored estimate snapshot');
            layoutMode('history');
          }
          job=state.job_id;display(state);
        }})
      .catch(error=>paintFeedback({status:'connection_error',error:error.message}))
      .finally(()=>{restoringEstimate=false;});


  }

})();
