/* Read saved contact exports as data; never reinterpret them as live telemetry. */
(function(root){
'use strict';
const vector=(v,n)=>Array.isArray(v)&&v.length===n&&v.every(x=>typeof x==='number'&&Number.isFinite(x));
function prepare(payload,context){
 const fail=message=>{throw Error(message)};
 if(!payload||payload.schema_version!==1||payload.frame!=='left_base_link'||payload.units!=='metres')fail('Expected a version 1 hand contact JSON in left_base_link metres.');
 if(!Array.isArray(payload.points)||payload.points.length>1062)fail('Expected up to 1,062 saved contact points.');
 const seen=new Set();
 for(const p of payload.points){
  if(!p||typeof p.id!=='string'||p.id.length>160||seen.has(p.id)||!vector(p.position_m,3)||p.position_m.some(x=>Math.abs(x)>10)||typeof p.value!=='number'||!Number.isFinite(p.value)||p.value<0||p.value>4095)fail('Invalid or duplicate contact point.');
  seen.add(p.id);
 }
 if(payload.centroids!==undefined&&(!Array.isArray(payload.centroids)||payload.centroids.length>17||payload.centroids.some(c=>!c||!vector(c.position_m,3)||c.position_m.some(x=>Math.abs(x)>10))))fail('Invalid contact centres.');
 const cloud=JSON.parse(JSON.stringify(payload));
 let solution=null,reason='';
 try{
  if(payload.valid!==true)throw Error('This export has no valid saved hand pose.');
  if(payload.profile!==context.id)throw Error('The hand profile differs from this viewer.');
  if(payload.reference_sha256!==context.referenceHash||payload.geometry_sha256!==context.geometryHash)throw Error('The saved geometry differs from this viewer.');
  if(!vector(payload.angles_act,6)||payload.angles_act.some(x=>x<0||x>1000)||!payload.kinematics_calibration)throw Error('The saved angles or calibration are missing.');
  const calibration=root.JointKinematics.validateCalibration(context.profile,cloud.kinematics_calibration);
  solution=root.JointKinematics.solve(context.profile,cloud.angles_act,calibration);
  if(payload.joint_angles_rad&&Object.entries(payload.joint_angles_rad).some(([key,value])=>typeof value!=='number'||!Number.isFinite(value)||!Number.isFinite(solution.q[key])||Math.abs(solution.q[key]-value)>1e-6))throw Error('The saved joint angles cannot be reproduced by this viewer.');
 }catch(error){reason=error.message;solution=null}
 return {cloud,solution,handAvailable:!!solution,handReason:reason};
}
root.SavedContacts={prepare};
if(typeof module!=='undefined')module.exports=root.SavedContacts;
})(typeof globalThis!=='undefined'?globalThis:this);
