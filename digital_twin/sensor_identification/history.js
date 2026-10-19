/* Local CSV decoding only. No eval, hardware reads, or invented XYZ columns. */
(function(root) {
  'use strict';
  const angleColumns=['little','ring','middle','index','thumb_bend','thumb_rotate'].map(k=>'angle_'+k+'_raw');
  function records(text) {
    text=text.replace(/^\uFEFF/,'');
    const first=text.split(/\r?\n/,1)[0],delimiter=first.includes(';')?';':',';
    const rows=[];let row=[],pieces=[],start=0,quoted=false;
    // Slice complete fields instead of allocating one string node per character
    // in the large pressure matrices (combined run files exceed 100 MB).
    function field(end){const value=pieces.length?pieces.join('')+text.slice(start,end):text.slice(start,end);pieces=[];return value}
    for(let i=0;i<text.length;i++){
      const c=text[i];
      if(c==='"') {
        pieces.push(text.slice(start,i));
        if(quoted&&text[i+1]==='"'){pieces.push('"');i++}else quoted=!quoted;
        start=i+1;
      }
      else if(c===delimiter&&!quoted){row.push(field(i));start=i+1}
      else if((c==='\n'||c==='\r')&&!quoted){row.push(field(i));if(c==='\r'&&text[i+1]==='\n')i++;if(row.some(x=>x.trim()))rows.push(row);row=[];start=i+1}
    }
    if(quoted)throw Error('CSV contains an unfinished quoted field.');
    row.push(field(text.length));if(row.some(x=>x.trim()))rows.push(row);
    if(rows.length<2)throw Error('CSV needs a header and at least one data row.');
    const header=rows.shift().map(x=>x.trim());
    if(new Set(header).size!==header.length)throw Error('CSV contains duplicate column names.');
    if(!header.includes('angles')&&!angleColumns.every(k=>header.includes(k)))throw Error('CSV needs an angles column or all six angle_*_raw columns.');
    if(rows.some(r=>r.length!==header.length))throw Error('CSV row width does not match its header.');
    return rows.map((values,i)=>Object.fromEntries(header.map((key,j)=>[key,values[j]??'']).concat([['_row',i+1]])));
  }
  function numericArray(text) {
    if(typeof text!=='string'||!text.trim())return null;
    try {const value=JSON.parse(text);if(Array.isArray(value))return value}catch{}
    // Older NumPy exports use spaces instead of commas. Parse that grammar
    // explicitly; truncated arrays ("...") are rejected rather than guessed.
    const tokens=[];const pattern=/\[|\]|,|[-+]?(?:\d+\.?\d*|\.\d+)(?:e[-+]?\d+)?|nan|none|null/gi;
    let last=0,match;
    while((match=pattern.exec(text))){if(text.slice(last,match.index).trim())throw Error('Unsupported or truncated numeric array.');tokens.push(match[0]);last=pattern.lastIndex}
    if(text.slice(last).trim())throw Error('Unsupported or truncated numeric array.');
    let i=0;
    function value(){const token=tokens[i++];if(token==='['){const out=[];while(i<tokens.length&&tokens[i]!==']'){if(tokens[i]===','){i++;continue}out.push(value())}if(tokens[i++]!==']')throw Error('Unfinished numeric array.');return out}if(token===undefined||token===']'||token===',')throw Error('Invalid numeric array.');return /^(nan|none|null)$/i.test(token)?null:Number(token)}
    const out=value();if(i!==tokens.length||!Array.isArray(out))throw Error('Expected a numeric array.');return out;
  }
  const finite=x=>typeof x==='number'&&Number.isFinite(x);
  function actualAngles(row) {
    const angles=row.angles?.trim()?numericArray(row.angles):angleColumns.map(k=>String(row[k]??'').trim()?Number(row[k]):null);
    if(!Array.isArray(angles)||angles.length!==6||!angles.every(x=>finite(x)&&x>=0&&x<=1000))throw Error('This row does not contain six valid angles in 0..1000.');
    return angles;
  }
  function telemetry(row,regions) {
    const angles=actualAngles(row);
    const matrix=numericArray(row.tactile_matrix),tactile_regions={};
    if(matrix!==null){
      if(matrix.length!==112||matrix.some(r=>!Array.isArray(r)||r.length!==17))throw Error('tactile_matrix must be a complete 112 × 17 array; truncated exports cannot be reconstructed.');
      regions.forEach((region,column)=>{
        const values=matrix.slice(0,region.rows*region.cols).map(r=>r[column]);
        if(values.every(x=>finite(x)&&x>=0&&x<=4095))tactile_regions[region.key]=Array.from({length:region.rows},(_,r)=>values.slice(r*region.cols,(r+1)*region.cols));
      });
    }else{
      regions.forEach(region=>{
        const keys=Array.from({length:region.rows*region.cols},(_,i)=>region.key+'_'+String(i).padStart(3,'0'));
        if(!keys.every(k=>Object.hasOwn(row,k)))return;
        const values=keys.map(k=>String(row[k]??'').trim()?Number(row[k]):null);
        if(values.every(x=>finite(x)&&x>=0&&x<=4095))tactile_regions[region.key]=Array.from({length:region.rows},(_,r)=>values.slice(r*region.cols,(r+1)*region.cols));
      });
    }
    return {angles_act:angles,tactile_regions,index:row.index||row._row,state:row.state||''};
  }
  root.IdentificationHistory={records,numericArray,telemetry,actualAngles};
  if(typeof module!=='undefined')module.exports=root.IdentificationHistory;
})(typeof globalThis==='undefined'?this:globalThis);

/* Keep run reconstruction in this already-supported history endpoint. */
/* Historical contact accumulation in the hand base; never touches hardware. */
(function(root){
  'use strict';
  const H=typeof module==='undefined'?root.IdentificationHistory:module.exports;
  const K=typeof module==='undefined'?root.JointKinematics:require('../joint_kinematics.js');
  const T=typeof module==='undefined'?root.TactilePoints:require('../tactile_points.js');
  const P=typeof module==='undefined'?root.JointPressure:require('../joint_pressure.js');
  const idle=new Set(['open','starting','initialised']);
  function groups(rows){
    const out=[];let current=null,previous=null;
    rows.forEach((row,i)=>{
      const index=String(row.index??'').trim()?Number(row.index):null;
      const old=String(previous?.index??'').trim()?Number(previous.index):null;
      const gap=Number.isFinite(index)&&Number.isFinite(old)&&index!==old+1;
      const explicit=String(row.run_id||'');
      if(idle.has(row.state)){current=null;previous=row;return}
      if(!current||gap||explicit!==current.explicit){
        current={key:'segment_'+(out.length+1),explicit,label:explicit||'Run '+(out.length+1),start:i,end:i,count:0,first_index:row.index||row._row,last_index:null};
        out.push(current);
      }
      current.end=i;current.count++;current.last_index=row.index||row._row;previous=row;
    });
    return out;
  }
  const closingFingerNames=['little','ring','middle','index'];
  function fingerBend(angles,context,finger){
    const m=context.profile.control_mapping[finger],a=context.calibration.actuators[finger];
    let t=(angles[m.index]-a.raw_at_closed)/(a.raw_at_open-a.raw_at_closed);
    if(a.reversed)t=1-t;
    return Math.abs((1-t)*(a.closed_angle_rad-a.open_angle_rad));
  }
  function fingerLimitOptions(rows,context,options){
    K.validateCalibration(context.profile,context.calibration);
    if(options.finger_limit_selection==='all')return options;
    const states=new Set(options.states||['thumb_probing','probing_complete']),bends={};
    for(const row of rows){
      if(row.state&&!states.has(row.state))continue;
      if(row.slip_detected===true||String(row.slip_detected).toLowerCase()==='true')continue;
      let angles;try{angles=H.actualAngles(row)}catch(e){throw Error('CSV row '+(row._row||row.index)+': '+e.message)}
      for(const finger of closingFingerNames)bends[finger]=Math.max(bends[finger]??-Infinity,fingerBend(angles,context,finger));
    }
    return {...options,finger_limit_bends:bends};
  }
  function accumulator(context,options={}){
    const threshold=options.threshold??200,voxelMm=options.voxel_mm??1;
    if(!Number.isFinite(threshold)||threshold<0||threshold>4095)throw Error('Contact threshold must be 0–4095.');
    if(!Number.isFinite(voxelMm)||voxelMm<.1||voxelMm>10)throw Error('Voxel spacing must be 0.1–10 mm.');
    const {profile,geometry,calibration}=context;
    K.validateCalibration(profile,calibration);
    if(geometry.profile!==context.id)throw Error('Geometry and hand profile differ.');
    if(context.boundaries?.geometry_sha256&&context.boundaries.geometry_sha256!==geometry.geometry_sha256)throw Error('Hand boundary geometry differs from the taxel model.');
    const voxels=new Map(),voxel=voxelMm/1000,run=options.run_key||'run',states=new Set(options.states||['thumb_probing','probing_complete']);
    const stats={frames_total:0,frames_reconstructed:0,frames_used:0,frames_skipped_state:0,frames_skipped_slip:0,frames_without_contact:0,contact_observations:0,limit_observations:0,unique_points:0};
    let limits=new Map(),limitSpacing=Math.max(2,voxelMm)/1000;
    const poseCache=new Map(),maxLimits=4096;
    const limitKey=p=>[p.kind,...[p.x,p.y,p.z].map(v=>Math.floor(v/limitSpacing)),...p.normal.map(v=>Math.round(v*2))].join(':');
    function addLimit(point){
      const key=limitKey(point),previous=limits.get(key);
      if(previous){previous.observations++;previous.last_source_index=point.source_index}
      else limits.set(key,{...point,observations:1,first_source_index:point.source_index,last_source_index:point.source_index});
      stats.limit_observations++;
    }
    function compactLimits(){
      while(limits.size>maxLimits){
        limitSpacing*=1.25;const reduced=new Map();
        for(const p of limits.values()){
          const key=limitKey(p),old=reduced.get(key);
          if(old){old.observations+=p.observations;old.last_source_index=p.last_source_index}
          else reduced.set(key,p);
        }
        limits=reduced;
      }
    }
    function pose(angles){
      const key=angles.join(',');
      if(poseCache.has(key))return poseCache.get(key);
      const solution=K.solve(profile,angles,calibration),inverse=T.inverseRigid(solution.frames.left_base_link);
      const cells=T.cells(geometry,solution,{},threshold).map(c=>({...c,xyz:K.point(inverse,c.position_world_m),normal:T.direction(inverse,c.normal_world)}));
      const bodies=(context.boundaries?.points||[]).filter(p=>solution.frames[p.link]).map(p=>{
        const transform=K.mul(inverse,solution.frames[p.link]);
        return {...p,xyz:K.point(transform,p.point_m),normal:T.direction(transform,p.normal)};
      });
      const value={solution,cells,bodies};
      if(poseCache.size>=24)poseCache.delete(poseCache.keys().next().value);
      poseCache.set(key,value);return value;
    }
    // Keep one initial contact patch per closing finger; thumb probing continues.
    const capturedFingers=new Map(),closingFingers=new Set(closingFingerNames);
    let first=null,last=null,handSnapshot=null,thumbClosureSample=null,thumbClosureBend=-Infinity;
    function add(row){
      stats.frames_total++;
      if(row.state&&!states.has(row.state)){stats.frames_skipped_state++;return}
      if(row.slip_detected===true||String(row.slip_detected).toLowerCase()==='true'){stats.frames_skipped_slip++;return}
      let packet;
      try{packet=H.telemetry(row,P.regions)}catch(e){throw Error('CSV row '+(row._row||row.index)+': '+e.message)}
      const samples={};
      for(const [key,matrix] of Object.entries(packet.tactile_regions)){
        samples[key]={values:matrix.flat()};
      }
      const {solution,cells:surface,bodies}=pose(packet.angles_act),cells=[];
      stats.frames_reconstructed++;
      const newlyContacting=new Set();
      const keepFingerLimit=finger=>!closingFingers.has(finger)||!options.finger_limit_bends||
        fingerBend(packet.angles_act,context,finger)>=options.finger_limit_bends[finger]-1e-9;
      for(const c of surface){
        const value=samples[c.region]?.values[c.wire_index]??null;
        const [x,y,z]=c.xyz;
        if(keepFingerLimit(c.region.split('_')[0]))addLimit({x,y,z,normal:c.normal,pressure:value,
          kind:value===null?'unknown_taxel':value>threshold?'contact_surface':'inactive_taxel',
          region:c.region,taxel_id:c.id,link:c.link,source_index:packet.index});
        if(value!==null&&value>threshold&&(options.thumb_only!==true||c.region.startsWith('thumb_'))){
          const finger=c.region.split('_')[0];
          if(closingFingers.has(finger)){
            if(capturedFingers.has(finger))continue;
            newlyContacting.add(finger);
          }
          cells.push({...c,value});
        }
      }
      const thumbPressures=cells.filter(c=>c.region.startsWith('thumb_')).map(c=>c.value);
      const thumbBend=fingerBend(packet.angles_act,context,'thumb_bend');
      if(thumbPressures.length>=3&&thumbBend>=thumbClosureBend){
        thumbClosureBend=thumbBend;
        thumbClosureSample={angles_act:packet.angles_act.slice(),thumb_pressures:thumbPressures,source_index:packet.index};
      }
      for(const finger of newlyContacting)capturedFingers.set(finger,packet.index);
      for(const p of bodies){if(!keepFingerLimit(p.link.replace(/^left_/, '').split('_')[0]))continue;const [x,y,z]=p.xyz;addLimit({x,y,z,normal:p.normal,pressure:null,kind:'hand_boundary',link:p.link,surface_id:p.id,source_index:packet.index})}
      compactLimits();
      if(cells.length){stats.frames_used++;first??=packet.index;last=packet.index}
      else stats.frames_without_contact++;
      // Keep a reproducible display pose, never the unrelated current hand.
      handSnapshot={schema_version:1,profile:geometry.profile,frame:'left_base_link',units:'metres',
        geometry_status:geometry.geometry_status,geometry_sha256:geometry.geometry_sha256,
        reference_sha256:geometry.reference_sha256,valid:true,threshold,
        angles_act:packet.angles_act.slice(),kinematics_calibration:K.clone(calibration),
        joint_angles_rad:{...solution.q},received_at:null,field_received_at:{},
        points:cells.map(c=>({id:c.id,region:c.region,link:c.link,wire_index:c.wire_index,
          value:c.value,position_m:c.xyz,normal:c.normal})),centroids:[]};
      for(const cell of cells){
        const [x,y,z]=cell.xyz;
        const key=[x,y,z].map(v=>Math.floor(v/voxel)).join(':');
        const point={id:run+':voxel:'+key,x,y,z,normal:cell.normal.slice(),pressure:cell.value,region:cell.region,taxel_id:cell.id,source_index:packet.index};
        const previous=voxels.get(key);
        if(!previous)voxels.set(key,{...point,observations:1,first_source_index:packet.index,last_source_index:packet.index});
        else{
          const n=previous.observations+1,begin=previous.first_source_index;
          if(point.pressure>previous.pressure)Object.assign(previous,point);
          Object.assign(previous,{observations:n,first_source_index:begin,last_source_index:packet.index});
        }
        stats.contact_observations++;
      }
    }
    function finish(){
      stats.unique_points=voxels.size;
      if(voxels.size>20000)throw Error('Run exceeds 20,000 spatial contacts. Increase point spacing or select a shorter run.');
      if(!limits.size)throw Error('No usable probing poses in the selected run.');
      const boundaryPoints=[...limits.entries()].map(([key,p])=>({...p,id:run+':limit:'+key}));
      stats.boundary_points=boundaryPoints.length;
      stats.inactive_taxel_limits=boundaryPoints.filter(p=>p.kind==='inactive_taxel').length;
      stats.unknown_taxel_limits=boundaryPoints.filter(p=>p.kind==='unknown_taxel').length;
      stats.hand_surface_limits=boundaryPoints.filter(p=>p.kind==='hand_boundary').length;
      return {schema_version:1,valid:true,frame:'left_base_link',units:'m',geometry_status:geometry.geometry_status,
        geometry_sha256:geometry.geometry_sha256,reference_sha256:geometry.reference_sha256,
        measurement_source:'csv_run',saved_name:options.name||run,received_at:null,field_received_at:{},
        hand_snapshot:handSnapshot,thumb_closure_sample:thumbClosureSample,
        timestamp_basis:'CSV sample order; capture times and pose/tactile skew unavailable',
        accumulation:{run_key:run,threshold,voxel_mm:voxelMm,thumb_only:options.thumb_only===true,
          states:[...states],first_contact_index:first,last_contact_index:last,...stats,
          finger_limit_selection:options.finger_limit_bends?'maximum_bend_per_finger':'whole_sweep',
          finger_limit_bends_rad:options.finger_limit_bends||null,
          finger_contact_selection:'first_contact_frame_per_finger',
          finger_first_contact_indices:Object.fromEntries(capturedFingers),
          limit_spacing_mm:limitSpacing*1000,limit_max_points:maxLimits,
          limit_reduction:'Oriented spatial voxels; opposite normals and evidence roles kept separate. Spacing increases when needed to stay within the limit budget.',
          object_motion_assumption:'Object stationary relative to hand base; unverified by CSV.',
          pressure_basis:'Recorded sensor units; missing reads and pressure calibration unverified.',
          reduction:'One maximum-pressure observed contact per spatial voxel; observation count does not weight fitting.'},
        boundary_points:boundaryPoints,points:[...voxels.values()]};
    }
    return {add,finish};
  }
  function select(rows,key){const run=groups(rows).find(r=>r.key===key);if(!run)throw Error('Select one CSV run.');return {run,rows:rows.slice(run.start,run.end+1)}}
  function single(rows){if(groups(rows).length!==1)throw Error('Build one continuous run at a time; separate grasps require registration.');}
  function selectedPoses(rows,context,options){
    if(options.pose_selection===undefined||options.pose_selection==='all')return {rows,details:{pose_selection:'whole_sweep'}};
    if(options.pose_selection!==undefined&&options.pose_selection!=='peak')throw Error('Unknown thumb pose selection.');
    K.validateCalibration(context.profile,context.calibration);
    const mapping=context.profile.control_mapping.thumb_bend,a=context.calibration.actuators.thumb_bend;
    const rotation=context.profile.control_mapping.thumb_rotate;
    if(!mapping||!a||!rotation)throw Error('Thumb bend and rotation calibration are required.');
    const states=new Set(options.states||['thumb_probing','probing_complete']),groups=new Map();
    let eligible=0,skippedState=0,skippedSlip=0;
    rows.forEach((row,ordinal)=>{
      if(row.state&&!states.has(row.state)){skippedState++;return}
      if(row.slip_detected===true||String(row.slip_detected).toLowerCase()==='true'){skippedSlip++;return}
      let angles;try{angles=H.actualAngles(row)}catch(e){throw Error('CSV row '+(row._row||row.index)+': '+e.message)}
      let t=(angles[mapping.index]-a.raw_at_closed)/(a.raw_at_open-a.raw_at_closed);
      if(a.reversed)t=1-t;
      const bend=Math.abs((1-t)*(a.closed_angle_rad-a.open_angle_rad));
      const key=angles[rotation.index],previous=groups.get(key);eligible++;
      // One deepest bend for EACH recorded rotation; equal peaks use the last sample.
      if(!previous||bend>=previous.bend-1e-12)groups.set(key,{row,angles,bend,ordinal});
    });
    if(!groups.size)throw Error('No usable probing poses in the selected run.');
    const peaks=[...groups.values()].sort((a,b)=>a.ordinal-b.ordinal);
    const selected=peaks.map(p=>({source_index:p.row.index||p.row._row,
      thumb_rotation_raw:p.angles[rotation.index],thumb_bend_raw:p.angles[mapping.index],thumb_bend_rad:p.bend}));
    return {rows:peaks.map(p=>p.row),details:{pose_selection:'maximum_bend_per_thumb_rotation',frames_total:rows.length,
      frames_skipped_state:skippedState,frames_skipped_slip:skippedSlip,frames_skipped_pose:eligible-peaks.length,
      thumb_rotation_groups:peaks.length,selected_source_indices:selected.map(p=>p.source_index),selected_poses:selected,
      rotation_grouping:'Each distinct recorded thumb rotation reading; no interpolation or angle bins.',
      selection_tie_break:'last sample at equal maximum bend within the same rotation'}};
  }
  function finishSelection(frame,selection){Object.assign(frame.accumulation,selection.details);return frame}
  function build(rows,context,options={}){
    single(rows);const selection=selectedPoses(rows,context,options),a=accumulator(context,fingerLimitOptions(selection.rows,context,options));
    selection.rows.forEach(a.add);return finishSelection(a.finish(),selection);
  }
  async function buildAsync(rows,context,options={}){
    single(rows);
    if(options.cancelled?.())throw Error('Run build cancelled.');
    const selection=selectedPoses(rows,context,options),a=accumulator(context,fingerLimitOptions(selection.rows,context,options));
    for(let i=0;i<selection.rows.length;i++){
      if(options.cancelled?.())throw Error('Run build cancelled.');
      a.add(selection.rows[i]);
      if(i%20===0){options.progress?.(i+1,selection.rows.length);await new Promise(resolve=>setTimeout(resolve,0))}
    }
    return finishSelection(a.finish(),selection);
  }
  function validateFrame(frame){
    if(frame?.valid!==true||frame.frame!=='left_base_link'||frame.units!=='m'||frame.measurement_source!=='csv_run'||!Array.isArray(frame.points)||(!frame.points.length&&!frame.boundary_points?.length)||frame.points.length>20000)throw Error('Expected an accumulated CSV-run cloud in left_base_link metres.');
    const ids=new Set();
    for(const p of frame.points){
      if(!p||typeof p.id!=='string'||ids.has(p.id)||!['x','y','z','pressure'].every(k=>typeof p[k]==='number'&&Number.isFinite(p[k]))||['x','y','z'].some(k=>Math.abs(p[k])>10)||p.pressure<0||p.pressure>4095)throw Error('Invalid accumulated contact point.');
      ids.add(p.id);
    }
    if(frame.boundary_points!==undefined){
      if(!Array.isArray(frame.boundary_points)||frame.boundary_points.length>4096)throw Error('Expected up to 4096 oriented hand limits.');
      for(const p of frame.boundary_points){
        if(!p||typeof p.id!=='string'||ids.has(p.id)||!['x','y','z'].every(k=>typeof p[k]==='number'&&Number.isFinite(p[k])&&Math.abs(p[k])<=10)||!Array.isArray(p.normal)||p.normal.length!==3||!p.normal.every(Number.isFinite)||Math.abs(Math.hypot(...p.normal)-1)>.1)throw Error('Invalid oriented hand limit.');
        if(p.pressure!==null&&(typeof p.pressure!=='number'||!Number.isFinite(p.pressure)||p.pressure<0||p.pressure>4095))throw Error('Invalid limit pressure evidence.');
        ids.add(p.id);
      }
    }
    return JSON.parse(JSON.stringify(frame));
  }
  root.IdentificationRuns={groups,select,build,buildAsync,validateFrame};
  H.runs=root.IdentificationRuns;
})(typeof globalThis==='undefined'?this:globalThis);
