#!/usr/bin/env node
/* Same offline run builder as History mode; JSON can be passed to identify_frame. */
const fs=require('node:fs'),path=require('node:path');
const H=require('./history.js'),R=require('./runs.js'),K=require('../joint_kinematics.js');
function main(){
  const args=process.argv.slice(2),input=args.shift();
  if(!input||input.startsWith('--'))throw Error('Usage: node digital_twin/sensor_identification/build_run_cloud.cjs INPUT.csv [--list] [--run run_002] [--output cloud.json] [--calibration reference.json] [--threshold 200] [--voxel-mm 1] [--all-regions]');
  const options={};
  while(args.length){const key=args.shift();if(['--list','--all-regions'].includes(key))options[key]=true;else if(['--run','--output','--calibration','--threshold','--voxel-mm'].includes(key)&&args.length)options[key]=args.shift();else throw Error('Unknown or incomplete option: '+key)}
  const rows=H.records(fs.readFileSync(input,'utf8')),groups=R.groups(rows);
  if(options['--list']){console.log(JSON.stringify(groups,null,2));return}
  const selected=options['--run']?groups.find(g=>g.key===options['--run']||g.explicit===options['--run']):groups.length===1?groups[0]:null;
  if(!selected)throw Error('Choose one run with --run; use --list to see run IDs.');
  if(options['--run']&&groups.filter(g=>g.key===options['--run']||g.explicit===options['--run']).length>1)throw Error('Run ID spans multiple discontinuous segments; select a segment key from --list.');
  const read=name=>JSON.parse(fs.readFileSync(path.join(__dirname,'..',name),'utf8'));
  const profile=read('joint_geometry_profiles_for_codex.json').primary_profiles.rh56_tactile_toolkit_left;
  const geometry=read('tactile_geometry.json');
  let calibration=K.defaults(profile);
  if(options['--calibration']){
    const saved=JSON.parse(fs.readFileSync(options['--calibration'],'utf8'));
    if(saved.geometry_sha256&&saved.geometry_sha256!==geometry.geometry_sha256)throw Error('Calibration reference has different taxel geometry.');
    if(saved.reference_sha256&&saved.reference_sha256!==geometry.reference_sha256)throw Error('Calibration reference has a different kinematic profile.');
    calibration=K.validateCalibration(profile,saved.kinematics_calibration||saved);
  }
  const frame=R.build(rows.slice(selected.start,selected.end+1),{id:geometry.profile,profile,geometry,calibration,boundaries:read('hand_boundaries.json')},{
    threshold:Number(options['--threshold']??200),voxel_mm:Number(options['--voxel-mm']??1),thumb_only:!options['--all-regions'],
    run_key:selected.explicit||selected.key,name:path.basename(input)+' · '+selected.label});
  frame.source={file:path.resolve(input),run_id:selected.explicit||selected.key,first_index:selected.first_index,last_index:selected.last_index,
    calibration:options['--calibration']?path.resolve(options['--calibration']):'Current uncalibrated profile defaults'};
  const output=path.resolve(options['--output']||'thumb_run_cloud.json');fs.mkdirSync(path.dirname(output),{recursive:true});
  fs.writeFileSync(output,JSON.stringify(frame,null,2)+'\n');
  const csv=output.replace(/\.json$/i,'')+'.points.csv';
  const fields=['id','x','y','z','pressure','region','taxel_id','source_index','observations'];
  const quote=v=>'"'+String(v??'').replaceAll('"','""')+'"';
  fs.writeFileSync(csv,fields.join(',')+'\n'+frame.points.map(p=>fields.map(k=>quote(p[k])).join(',')).join('\n')+'\n');
  const limitsCsv=output.replace(/\.json$/i,'')+'.limits.csv';
  const limitFields=['id','x','y','z','nx','ny','nz','kind','pressure','source_index','observations'];
  fs.writeFileSync(limitsCsv,limitFields.join(',')+'\n'+frame.boundary_points.map(p=>{const row={...p,nx:p.normal[0],ny:p.normal[1],nz:p.normal[2]};return limitFields.map(k=>quote(row[k])).join(',')}).join('\n')+'\n');
  console.log(JSON.stringify({output,points_csv:csv,limits_csv:limitsCsv,...frame.accumulation},null,2));
}
try{main()}catch(e){console.error(e.message);process.exitCode=1}
