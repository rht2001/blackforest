/* Historical contact accumulation in the hand base; never touches hardware. */
(function(root){
  'use strict';
  const H=typeof module==='undefined'?root.IdentificationHistory:require('./history.js');
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
  function accumulator(context,options={}){
    const threshold=options.threshold??200,voxelMm=options.voxel_mm??1;
    if(!Number.isFinite(threshold)||threshold<0||threshold>4095)throw Error('Contact threshold must be 0–4095.');
    if(!Number.isFinite(voxelMm)||voxelMm<.1||voxelMm>10)throw Error('Voxel spacing must be 0.1–10 mm.');
    const {profile,geometry,calibration}=context;
    K.validateCalibration(profile,calibration);
    if(geometry.profile!==context.id)throw Error('Geometry and hand profile differ.');
    const voxels=new Map(),voxel=voxelMm/1000,run=options.run_key||'run',states=new Set(options.states||['thumb_probing','probing_complete']);
    const stats={frames_total:0,frames_used:0,frames_skipped_state:0,frames_skipped_slip:0,frames_without_contact:0,contact_observations:0,unique_points:0};
    let first=null,last=null;
    function add(row){
      stats.frames_total++;
      if(row.state&&!states.has(row.state)){stats.frames_skipped_state++;return}
      if(row.slip_detected===true||String(row.slip_detected).toLowerCase()==='true'){stats.frames_skipped_slip++;return}
      let packet;
      try{packet=H.telemetry(row,P.regions)}catch(e){throw Error('CSV row '+(row._row||row.index)+': '+e.message)}
      const samples={};
      for(const [key,matrix] of Object.entries(packet.tactile_regions)){
        if(options.thumb_only!==false&&!key.startsWith('thumb_'))continue;
        samples[key]={values:matrix.flat()};
      }
      if(!Object.values(samples).some(s=>s.values.some(v=>v>threshold))){stats.frames_without_contact++;return}
      const solution=K.solve(profile,packet.angles_act,calibration);
      const inverse=T.inverseRigid(solution.frames.left_base_link);
      const cells=T.cells(geometry,solution,samples,threshold).filter(c=>c.active);
      if(!cells.length){stats.frames_without_contact++;return}
      stats.frames_used++;first??=packet.index;last=packet.index;
      for(const cell of cells){
        const [x,y,z]=K.point(inverse,cell.position_world_m);
        const key=[x,y,z].map(v=>Math.floor(v/voxel)).join(':');
        const point={id:run+':voxel:'+key,x,y,z,pressure:cell.value,region:cell.region,taxel_id:cell.id,source_index:packet.index};
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
      if(!voxels.size)throw Error('No valid '+(options.thumb_only===false?'':'thumb ')+'contacts above '+threshold+' in the selected probing run.');
      return {schema_version:1,valid:true,frame:'left_base_link',units:'m',geometry_status:geometry.geometry_status,
        geometry_sha256:geometry.geometry_sha256,reference_sha256:geometry.reference_sha256,
        measurement_source:'csv_run',saved_name:options.name||run,received_at:null,field_received_at:{},
        timestamp_basis:'CSV sample order; capture times and pose/tactile skew unavailable',
        accumulation:{run_key:run,threshold,voxel_mm:voxelMm,thumb_only:options.thumb_only!==false,
          states:[...states],first_contact_index:first,last_contact_index:last,...stats,
          object_motion_assumption:'Object stationary relative to hand base; unverified by CSV.',
          pressure_basis:'Recorded sensor units; missing reads and pressure calibration unverified.',
          reduction:'One maximum-pressure observed contact per spatial voxel; observation count does not weight fitting.'},
        points:[...voxels.values()]};
    }
    return {add,finish};
  }
  function select(rows,key){const run=groups(rows).find(r=>r.key===key);if(!run)throw Error('Select one CSV run.');return {run,rows:rows.slice(run.start,run.end+1)}}
  function single(rows){if(groups(rows).length!==1)throw Error('Build one continuous run at a time; separate grasps require registration.');}
  function build(rows,context,options={}){single(rows);const a=accumulator(context,options);rows.forEach(a.add);return a.finish()}
  async function buildAsync(rows,context,options={}){
    single(rows);
    const a=accumulator(context,options);
    for(let i=0;i<rows.length;i++){
      if(options.cancelled?.())throw Error('Run build cancelled.');
      a.add(rows[i]);
      if(i%20===0){options.progress?.(i+1,rows.length);await new Promise(resolve=>setTimeout(resolve,0))}
    }
    return a.finish();
  }
  function validateFrame(frame){
    if(frame?.valid!==true||frame.frame!=='left_base_link'||frame.units!=='m'||frame.measurement_source!=='csv_run'||!Array.isArray(frame.points)||!frame.points.length||frame.points.length>20000)throw Error('Expected an accumulated CSV-run cloud in left_base_link metres (up to 20,000 contacts).');
    const ids=new Set();
    for(const p of frame.points){
      if(!p||typeof p.id!=='string'||ids.has(p.id)||!['x','y','z','pressure'].every(k=>typeof p[k]==='number'&&Number.isFinite(p[k]))||['x','y','z'].some(k=>Math.abs(p[k])>10)||p.pressure<0||p.pressure>4095)throw Error('Invalid accumulated contact point.');
      ids.add(p.id);
    }
    return JSON.parse(JSON.stringify(frame));
  }
  root.IdentificationRuns={groups,select,build,buildAsync,validateFrame};
  if(typeof module!=='undefined')module.exports=root.IdentificationRuns;
})(typeof globalThis==='undefined'?this:globalThis);
