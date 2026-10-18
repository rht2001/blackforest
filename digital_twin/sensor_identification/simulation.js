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
      #panel{width:min(350px,calc(100vw - 24px))}#profileWorkspace{left:374px}
      #onePageApp{font-size:12px;line-height:1.45}#onePageApp h1{font-size:20px;margin:0 0 14px}
      #onePageApp h2{font-size:13px;margin:14px 0 7px}#onePageApp p{margin:7px 0}
      #onePageApp button,#onePageApp select,#onePageApp input{font:inherit}
      #onePageApp button{background:#18384e;color:#dcebf5;border:1px solid #42647b;border-radius:6px;padding:8px;cursor:pointer}
      #onePageApp button:disabled{opacity:.5;cursor:not-allowed}#onePageApp button.active{background:#145875;border-color:#59c4e3}
      .data-modes{display:grid;grid-template-columns:1fr 1fr 1fr;gap:5px;margin-bottom:8px}
      .data-actions{display:flex;gap:6px;flex-wrap:wrap;margin:8px 0}.data-actions button{flex:1}
      #onePageApp select{width:100%;padding:7px;background:#10283b;color:#dcebf5;border:1px solid #42647b;border-radius:6px}
      #onePageApp input[type=number]{max-width:90px;background:#10283b;color:#dcebf5;border:1px solid #42647b;border-radius:5px;padding:6px}
      #onePageApp input[type=file]{max-width:100%;font-size:11px}#estimateButton{width:100%;background:#135b72!important;font-weight:650;margin:13px 0 4px}
      #estimateStatus{background:#10283b;border-radius:6px;padding:9px;font-size:11px}#onePageApp .hint{font-size:11px;color:#a6bac9}
      #onePageApp details{margin-top:10px;padding-top:8px}#onePageApp label{display:block;margin:7px 0}#onePageApp [hidden]{display:none!important}
      #estimateView{position:absolute;inset:0;z-index:8;background:#fff}#estimateView[hidden]{display:none}#estimateView iframe{width:100%;height:100%;border:0}
      @media(min-width:621px) and (max-width:900px){#panel{width:300px}#profileWorkspace{left:324px}}
      @media(max-width:620px){#panel{width:100%;height:50vh;max-height:50vh;padding:10px 12px}#profileWorkspace{left:0;top:50vh}#onePageApp h1{font-size:16px;margin-bottom:8px}#onePageApp h2{margin-top:8px}#onePageApp button{padding:6px}}
    </style><h1>Hand twin</h1><h2>1. Choose data</h2>
      <div class="data-modes" role="tablist" aria-label="Data mode">
        <button role="tab" id="dataManual">Manual</button><button role="tab" id="dataLive">Live</button><button role="tab" id="dataHistory">History</button>
      </div><p id="sourceInfo" class="hint"></p>
      <section id="manualData"><div class="data-actions" id="manualActions"></div><details id="manualAngles"><summary>Set finger angles</summary></details></section>
      <section id="liveData" hidden></section>
      <section id="historyData" hidden><button id="chooseHistory" type="button">Choose JSON or CSV…</button><input id="historyFiles" type="file" accept=".json,.csv,application/json,text/csv" hidden><p id="selectedHistory" class="hint">No history file selected.</p>
        <div id="csvControls" hidden><label>Use CSV as <select id="csvScope"><option value="run">Whole probing run</option><option value="row">Single row</option></select></label>
        <div id="runControls"><label>Run <select id="historyRun"></select></label><label>Contacts <select id="runRegions"><option value="thumb">Thumb only</option><option value="all">All hand regions</option></select></label><label>Point spacing (mm) <input id="runVoxel" type="number" min="0.1" max="10" step="0.1" value="1"></label><button id="buildRun">Build run point cloud</button><p class="hint">Uses probing and completed-hold contacts. Coordinates assume the object stays fixed relative to the hand base.</p></div>
        <div id="rowControls" hidden><label>CSV row <input id="historyRow" type="number" value="1" min="1"></label><div class="data-actions"><button id="historyPrev">Previous</button><button id="historyNext">Next</button></div></div><p id="historyRowInfo" class="hint"></p></div>
        <button id="downloadRun" hidden>Save run point cloud JSON</button>
        <div id="runLayers" hidden><label><input id="runShowLimits" type="checkbox" checked> Show swept hand-space limits</label><p class="hint"><span style="color:#b88aff">Purple: hand surfaces</span> · <span style="color:#58bfff">Blue: inactive taxels</span> · <span style="color:#94a3b8">Gray: pressure unknown</span>. Bright pressure colors mark active contacts. Each limit follows its recorded hand pose.</p></div>
        <p id="historyStatus" class="hint">Choose a file to preview it before estimating.</p>
      </section>
      <h2>2. Estimate from these points</h2><button id="estimateButton">Estimate</button><p id="estimateStatus" role="status" aria-live="polite">Ready. The selected data source supplies the point cloud.</p>
      <div class="data-actions"><button id="showHand">Hand view</button><button id="showCloud" disabled>Point cloud</button><button id="showEstimate" disabled>Estimate in hand</button><button id="showFitReport" disabled>Fit report</button></div><label id="estimateCandidateLabel" hidden>Fitted candidate <select id="estimateCandidate"></select></label><div id="viewActions" class="data-actions"></div>
      <p class="hint">Purple and gray: hand surface limits · red: measured contacts</p><div id="boundaryDisplay"></div><details id="estimateSettings"><summary>Estimation settings</summary><label>Contact threshold <input id="estimateThreshold" type="number" min="0" max="4095" value="200"></label><label>Position tolerance (mm) <input id="estimateTolerance" type="number" min="0.1" max="20" step="0.1" value="1.5"></label><p class="hint">Every red contact must be within the position tolerance. All known hand surfaces, including gray taxel positions, impose a separate 0.25 mm penetration limit. Hand limits use conservative local surface patches and require calibrated geometry.</p></details>
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
    runView.style.cssText='position:absolute;inset:0;z-index:7;background:#08111d';
    runView.innerHTML='<div id="runCloudScene" style="width:100%;height:100%"></div>';
    $('profileWorkspace').append(runView);let runPlot=null,runFrame=null,buildVersion=0;
    const base=location.protocol==='file:'?'http://127.0.0.1:18765':location.origin;
    let mode='manual',switching=false,historyValid=false,csvRows=null,csvName='',job=null,pollTimer=null,running=false,loading=false,historyFrame=null,loadVersion=0;
    const status=$('estimateStatus');
    let fittedFrame=null,fittedResult=null,fittedThreshold=200;
    function clearEstimate(){window.HandEstimate?.clear();fittedFrame=null;fittedResult=null;$('showEstimate').disabled=true;$('showFitReport').disabled=true;$('estimateCandidateLabel').hidden=true;}
    function placeEstimate(){
      if(!fittedFrame||!fittedResult)return;
      estimateView.hidden=true;runView.hidden=true;
      const candidate=$('estimateCandidate').value;
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
      mode=next;for(const key of ['manual','live','history']){
        $(key+'Data').hidden=key!==mode;const tab=$('data'+key[0].toUpperCase()+key.slice(1));tab.classList.toggle('active',key===mode);tab.setAttribute('aria-selected',String(key===mode));
      }
      $('sourceInfo').textContent=mode==='manual'?'Set the angles yourself. Pressure is unknown; angles alone cannot identify an object.':mode==='live'?'Current actual angles and pressure from the connected hand.': 'Historical data only. Live updates are paused.';
      updateReady();
    }
    function selectMode(next){
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
      if(!runPlot)runPlot=await HandRenderer.create('runCloudScene',traces);
      else await HandRenderer.restyle(runPlot,{'x':traces.map(t=>t.x),'y':traces.map(t=>t.y),'z':traces.map(t=>t.z),'visible':traces.map(t=>t.visible),'marker.rgb':traces.map(t=>t.marker.rgb||null),'marker.color':traces.map(t=>t.marker.color)},[0,1,2,3]);
      const shown=visible?[...points,...limits]:points.length?points:limits;
      const center=['x','y','z'].map(k=>shown.reduce((s,p)=>s+p[k]*1000,0)/shown.length);
      const radius=Math.max(...shown.map(p=>Math.hypot(p.x*1000-center[0],p.y*1000-center[1],p.z*1000-center[2])));
      runPlot._renderer.focus(center,[.5,1,.4],Math.max(30,radius*3));
    }
    $('runShowLimits').onchange=()=>{if(runFrame)drawRun()};
    async function acceptRun(frame){
      clearEstimate();runFrame=IdentificationRuns.validateFrame(frame);historyFrame=runFrame;historyValid=true;
      layoutMode('history');$('selectedHistory').textContent=frame.saved_name;
      const a=frame.accumulation;
      status.textContent=frame.points.length+' active contacts · '+(frame.boundary_points?.length||0)+' swept space limits'+(a?' from '+(a.frames_reconstructed??a.frames_used)+' probing samples.':'.')+(frame.points.length?' Press Estimate to fit with these limits.':' No contacts: the hand sweep constrains space, but does not uniquely locate the object.');
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
    $('showFitReport').onclick=()=>{estimateView.hidden=false;runView.hidden=true};
    const reasons={space_constraints_only:'Hand sweep constrains space; object pose is unanchored without contacts.',both_candidates_compatible:'Both shapes fit.',neither_candidate_compatible_within_search:'Neither shape fits sufficiently within this search.',insufficient_contacts:'Not enough valid contact pressure.',insufficient_spatial_coverage:'Contacts cover too little of the object.',calibration_unverified:'Geometry needs calibration before a confident label.',score_margin_too_small:'The score difference is too small.',alignment_search_unstable:'Alignment did not settle reliably.',invalid_or_stale_frame:'No valid synchronized snapshot.',one_candidate_clearly_more_consistent:'One shape fits clearly better.'};
    function display(state){
      if(state.estimator_revision!=='strict_hand_v1')throw Error('Restart the simulation launcher to load the updated estimator, then reload this page.');
      running=state.status==='running';updateReady();clearTimeout(pollTimer);
      if(state.status==='running'){
        status.textContent='Estimating frozen snapshot: '+(state.input_label||mode)+'. You can keep inspecting the hand.';
        job=state.job_id;pollTimer=setTimeout(refresh,1500);
      }else if(state.status==='complete'){
        const result=state.result,errors=Object.entries(result.fits||{}).map(([name,fit])=>name.replace('SENSOR_','')+': '+(fit.compatible?'valid':'rejected')+' (max contact '+(fit.max_error_mm??fit.rmse_mm).toFixed(2)+' mm, hand '+(fit.hand_limit_max_mm??0).toFixed(2)+' mm)').join(' · ');
        status.textContent='Last estimate: '+result.classification+' · '+(result.contacts===0&&result.hand_boundary_points>0?'Hand sweep restricts space; object pose has no contact anchor.':reasons[result.reason]||result.reason)+' Snapshot: '+state.input_label+(errors?' · '+errors:'');
        estimateView.querySelector('iframe').src=base+state.report_url+'?job='+state.job_id;
        estimateView.hidden=true;runView.hidden=true;$('showFitReport').disabled=false;
        fittedResult=result;
        const candidates = Object.entries(result.fits || {})
        .filter(([, fit]) =>
          Number.isFinite(fit.score) &&
          Array.isArray(fit.transform)
        )
        .sort((a, b) => {
          if (a[1].compatible !== b[1].compatible)
            return a[1].compatible ? -1 : 1;

          return a[1].score - b[1].score;
        });
        $('estimateCandidate').replaceChildren(...candidates.map(([name])=>{const option=document.createElement('option');option.textContent = name +
  (result.fits[name].compatible
    ? ' · compatible'
    : ' · REJECTED (debug only)');
        $('estimateCandidateLabel').hidden=candidates.length<2;
        $('showEstimate').disabled=!candidates.length||!fittedFrame?.hand_snapshot;
        if(candidates.length&&fittedFrame?.hand_snapshot){status.textContent+=' Provisional pose shown in the frozen hand'+(fittedFrame.measurement_source==='csv_run'?' at the last contact sample.':'.');placeEstimate();}
        else{window.HandEstimate?.clear();status.textContent+=result.contacts===0&&result.hand_boundary_points>0?' Swept limits are retained for later contact or pose evidence.':candidates.length?' No reproducible hand pose; see Fit report.':' No valid STL pose; see Fit report for rejected candidates.';}
      }else if(state.status==='error')status.textContent='Estimate failed: '+state.error;
    }
    async function refresh(){try{
      const response=await fetch(base+'/api/identification',{signal:AbortSignal.timeout(10000)}),state=await response.json();
      if(!response.ok)throw Error(state.error||'Service unavailable');
      if(job&&state.job_id!==job){clearEstimate();throw Error('Another estimate replaced this job. Run Estimate again.');}
      display(state);
    }catch(error){running=false;updateReady();status.textContent=error.message}}
    $('estimateButton').onclick=async()=>{
      if(loading||running)return;
      if(mode==='history'&&!historyValid){status.textContent='Select a valid history frame before estimating.';return}
      const threshold=Number($('estimateThreshold').value),tolerance=Number($('estimateTolerance').value);
      if(!Number.isFinite(threshold)||threshold<0||threshold>4095||!Number.isFinite(tolerance)||tolerance<.1||tolerance>20){status.textContent='Check the threshold (0–4095) and tolerance (0.1–20 mm).';return}
      window.ProfileTwin.tick();const frame=mode==='history'?historyFrame:window.getIdentificationFrame();
      if(!frame.valid){status.textContent=mode==='live'?'Live angles and pressure are missing, stale or unsynchronized.':'Load a valid history frame or choose a manual pose first.';return}
      const frozen=JSON.parse(JSON.stringify(frame));
      clearEstimate();fittedFrame=frozen;fittedThreshold=threshold;
      const count=frozen.points.filter(p=>typeof p.pressure==='number'&&p.pressure>threshold).length;
      const label=mode==='history'?'History · '+(frame.saved_name||'capture'):mode==='manual'?'Manual pose':'Live hand';
      status.textContent='Captured '+frozen.points.length+' positions · '+count+' contact points · '+(frozen.boundary_points?.length||0)+' hand limits. Starting estimate…';running=true;updateReady();
      try{
        const check=await fetch(base+'/api/identification',{signal:AbortSignal.timeout(10000)});
        const backend=await check.json();
        if(!check.ok)throw Error(backend.error||'Could not reach estimator');
        if(backend.estimator_revision!=='strict_hand_v1')throw Error('Restart the simulation launcher to load the updated estimator, then reload this page.');
        const response=await fetch(base+'/api/identification',{method:'POST',headers:{'Content-Type':'application/json'},signal:AbortSignal.timeout(15000),body:JSON.stringify({mode:'current',frame:frozen,threshold,tolerance_mm:tolerance,input_label:label})});
        const state=await response.json();if(response.status===409){clearEstimate();throw Error('An estimate is already running. Wait for it to finish, then press Estimate again.');}if(!response.ok)throw Error(state.error||'Could not estimate');job=state.job_id;display(state);
      }catch(error){running=false;updateReady();status.textContent='Estimate failed: '+error.message}
    };
    selectMode('manual');
  }
})();
