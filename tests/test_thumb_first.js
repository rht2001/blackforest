const assert=require('node:assert/strict'),path=require('node:path'),fs=require('node:fs');
const root=path.resolve(__dirname,'..');
const H=require(path.join(root,'digital_twin/sensor_identification/history.js'));
const K=require(path.join(root,'digital_twin/joint_kinematics.js'));
const read=n=>JSON.parse(fs.readFileSync(path.join(root,'digital_twin',n),'utf8'));
const profile=read('joint_geometry_profiles_for_codex.json').primary_profiles.rh56_tactile_toolkit_left;
const context={id:'rh56_tactile_toolkit_left',profile,calibration:K.defaults(profile),geometry:read('tactile_geometry.json'),boundaries:read('hand_boundaries.json')};
function row(index,bend,contact=true,extra={}){
 const m=Array.from({length:112},()=>Array(17).fill(0));
 if(contact)for(let i=0;i<6;i++)m[i][15]=700;
 return {index:String(index),state:'thumb_probing',angles:JSON.stringify([1000,1000,1000,1000,bend,500]),tactile_matrix:JSON.stringify(m),...extra};
}
const frame=H.runs.build([row(1,900),row(2,100),row(3,0,false),row(4,0,true,{slip_detected:'true'}),row(5,0,true,{state:'resetting'})],context,{pose_selection:'all'});
assert.equal(frame.thumb_closure_sample.source_index,'2');
assert.equal(frame.thumb_closure_sample.angles_act[4],100);
assert(frame.thumb_closure_sample.thumb_pressures.length>=3);
assert.equal(frame.hand_snapshot.angles_act[4],0);
assert.deepEqual(H.runs.validateFrame(frame).thumb_closure_sample,frame.thumb_closure_sample);
assert.equal(H.runs.build([row(1,0,false)],context).thumb_closure_sample,null);
const sandbox={};require('node:vm').runInNewContext(fs.readFileSync(path.join(root,'digital_twin/sensor_identification/simulation.js'),'utf8').split('/* One page:')[0],sandbox);const F=sandbox.EstimateFeedback;
const html=F.render({status:'running',first_guess:{candidate:'SENSOR_A',closure_fraction:.9},progress:{stage:'alignment',candidate:'SENSOR_A'}});
assert(html.includes('First guess: VEGABAR 29'));
assert(html.includes('90% closed with contact'));
console.log('PASS thumb contact/pose pairing, slip/state exclusions, no-contact suppression, saved-cloud roundtrip, running first-guess feedback');

