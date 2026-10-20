const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path');
const root=path.join(__dirname,'../digital_twin');
const H=require(root+'/sensor_identification/history.js'),RunAPI=require(root+'/sensor_identification/runs.js');
// These regression checks explicitly exercise full-sweep reconstruction.
const R={...RunAPI,build:(rows,context,options={})=>RunAPI.build(rows,context,{pose_selection:'all',...options}),buildAsync:(rows,context,options={})=>RunAPI.buildAsync(rows,context,{pose_selection:'all',...options})};
const K=require(root+'/joint_kinematics.js'),T=require(root+'/tactile_points.js'),P=require(root+'/joint_pressure.js');
const geometry=JSON.parse(fs.readFileSync(root+'/tactile_geometry.json','utf8'));
const profile=JSON.parse(fs.readFileSync(root+'/joint_geometry_profiles_for_codex.json','utf8')).primary_profiles.rh56_tactile_toolkit_left;
const context={id:geometry.profile,profile,geometry,calibration:K.defaults(profile)};
function row(index,angle=1000,extra={}){
 const matrix=Array.from({length:112},()=>Array(17).fill(0));matrix[0][15]=700;matrix[1][15]=500;
 return {index:String(index),_row:index,angles:JSON.stringify([1000,1000,1000,1000,angle,500]),tactile_matrix:JSON.stringify(matrix),state:'thumb_probing',slip_detected:'False',...extra};
}
const pose1=row(1),pose2=row(2,400),held=row(3,400);
const cloud=R.build([pose1,pose2],context,{voxel_mm:.1});
const withHold=R.build([pose1,pose2,held],context,{voxel_mm:.1});
assert.equal(cloud.points.length,4,'A moving thumb must create positions from each pose');
assert.equal(withHold.points.length,cloud.points.length,'Repeated hold must not gain fit weight');
assert.equal(withHold.accumulation.contact_observations,6);
assert.equal(new Set(cloud.points.map(p=>p.id)).size,cloud.points.length);
const expected=T.cells(geometry,K.solve(profile,[1000,1000,1000,1000,1000,500],context.calibration),{thumb_pad:{values:[700,500,...Array(94).fill(0)]}},200).find(c=>c.id==='thumb_pad:0');
const inv=T.inverseRigid(K.solve(profile,[1000,1000,1000,1000,1000,500],context.calibration).frames.left_base_link);
assert.deepEqual([cloud.points[0].x,cloud.points[0].y,cloud.points[0].z],K.point(inv,expected.position_world_m));
assert.equal(cloud.units,'m');assert.equal(cloud.received_at,null);
assert.throws(()=>R.build([row(1),row(9)],context),/one continuous run/);
assert.throws(()=>R.build([row(1,1000,{run_id:'a'}),row(2,1000,{run_id:'b'})],context),/one continuous run/);
assert.equal(R.groups([row(1),row(2,1000,{state:'open'}),row(3)]).length,2);
assert.throws(()=>R.build([row(1,1000,{slip_detected:'True'})],context),/No usable/);
assert.throws(()=>R.build([row(1,1000,{state:'resetting'})],context),/No usable/);
assert.equal(R.build([row(1,1000,{state:'probing_complete'})],context).points.length,2);
assert.throws(()=>R.build([row(1,1000,{angles:'[1,2]'})],context),/CSV row.*six valid/);
assert.equal(R.build([row(1,1000,{tactile_matrix:''})],context).points.length,0);
const flat={index:'1',state:'thumb_probing'};
['little','ring','middle','index','thumb_bend','thumb_rotate'].forEach((k,i)=>flat['angle_'+k+'_raw']=[1000,1000,1000,1000,1000,500][i]);
P.regions.filter(r=>r.key.startsWith('thumb_')).forEach(r=>{for(let i=0;i<r.rows*r.cols;i++)flat[r.key+'_'+String(i).padStart(3,'0')]='0'});
flat.thumb_pad_000='700';flat.thumb_pad_001='500';
assert.deepEqual(R.build([flat],context).points,R.build([pose1],context).points);
const csv=Object.keys(flat).join(';')+'\n'+Object.values(flat).join(';')+'\n';
assert.equal(H.records(csv).length,1);assert.equal(H.telemetry(H.records(csv)[0],P.regions).tactile_regions.thumb_pad[0][0],700);
assert.throws(()=>H.records('angles;state\n[1 2 3 4 5 6];probe;extra'),/row width/);
assert.throws(()=>R.validateFrame({...cloud,points:[cloud.points[0],cloud.points[0]]}),/Invalid/);
assert.equal(R.validateFrame(cloud).points.length,4);
(async()=>{
 const asyncCloud=await R.buildAsync([pose1,pose2],context,{voxel_mm:.1});assert.deepEqual(asyncCloud,cloud);
 await assert.rejects(R.buildAsync([pose1],context,{cancelled:()=>true}),/cancelled/);
 console.log('PASS moving-pose reconstruction, voxel hold reduction, run boundaries, pressure/pose validation, flat/raw CSV parity and asynchronous cancellation');
})().catch(e=>{console.error(e);process.exitCode=1});

assert.equal(cloud.hand_snapshot.angles_act[4],400);
assert.equal(cloud.hand_snapshot.units,"metres");
assert.ok(cloud.hand_snapshot.points.length);

// The history UI keeps one deepest thumb bend at each recorded rotation.
const thumbPeaks=R.build([row(1,900),row(2,400),row(3,700)],context,{thumb_limit_selection:'peak'});
assert.equal(thumbPeaks.accumulation.thumb_limit_selection,'maximum_bend_per_thumb_rotation');
assert.ok(thumbPeaks.points.length>0);
assert.ok(thumbPeaks.points.every(p=>String(p.source_index)==='2'));
assert.equal(thumbPeaks.accumulation.frames_reconstructed,3,'Other fingers retain their full row history');

// Pre-roll pressure is not the closing endpoint; reopening must not become the default pose.
const endMatrix=JSON.stringify(Array.from({length:112},()=>Array(17).fill(700)));
const endRows=[990,400,405,990].map((angle,i)=>({index:String(i+1),_row:i+1,
 angles:JSON.stringify([angle,angle,angle,angle,650,600]),tactile_matrix:endMatrix,state:'thumb_probing',slip_detected:'False'}));
const endpoint=R.build(endRows,context,{thumb_limit_selection:'peak'});
assert.equal(endpoint.accumulation.finger_contact_selection,'last_closed_contact_per_finger');
assert.deepEqual(endpoint.hand_snapshot.angles_act.slice(0,4),[405,405,405,405]);
assert.equal(endpoint.hand_snapshot.pose_source_row,3);
assert.deepEqual(Object.values(endpoint.accumulation.finger_selected_pose_indices),['3','3','3','3']);
assert.ok(endpoint.points.filter(p=>['little','ring','middle','index'].includes(p.region.split('_')[0])).every(p=>p.source_index==='3'));
const endCells=T.cells(geometry,K.solve(profile,[405,405,405,405,650,600],context.calibration),{},200);
for(const point of endpoint.points.filter(p=>p.region.startsWith('index_'))){
 const cell=endCells.find(c=>c.id===point.taxel_id);
 const inverse=T.inverseRigid(K.solve(profile,[405,405,405,405,650,600],context.calibration).frames.left_base_link);
 assert.deepEqual([point.x,point.y,point.z],K.point(inverse,cell.position_world_m));
}
