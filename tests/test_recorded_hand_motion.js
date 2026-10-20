const assert=require('node:assert/strict'),fs=require('fs');
const root=require('path').join(__dirname,'../digital_twin');
const H=require(root+'/sensor_identification/history.js'),K=require(root+'/joint_kinematics.js');
const profile=JSON.parse(fs.readFileSync(root+'/joint_geometry_profiles_for_codex.json')).primary_profiles.rh56_tactile_toolkit_left;
const geometry=JSON.parse(fs.readFileSync(root+'/tactile_geometry.json'));
const context={id:geometry.profile,profile,geometry,calibration:K.defaults(profile)};
const matrix=Array.from({length:112},()=>Array(17).fill(0));
const rows=[400,600,800].map((rotation,i)=>({index:String(i+1),_row:i+1,state:'thumb_probing',slip_detected:'False',angles:JSON.stringify([1000,1000,1000,1000,500,rotation]),tactile_matrix:JSON.stringify(matrix)}));
const frame=H.runs.build(rows,context,{});
assert.equal(frame.points.length,0);assert.equal(frame.hand_snapshot.motion_samples.length,3);
assert.ok(frame.hand_snapshot.motion_samples.every(p=>p.link_prefix==='left_thumb_'));
assert.deepEqual(frame.hand_snapshot.motion_samples.map(p=>p.source_index),['1','2','3']);
console.log('PASS no-contact measured thumb poses retained with source indices');

