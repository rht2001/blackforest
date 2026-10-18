/* URDF FK in metres/radians. No rendering, networking or hardware commands. */
(function(root){
'use strict';
const clone=v=>JSON.parse(JSON.stringify(v));
const finite=v=>typeof v==='number'&&Number.isFinite(v);
const vec=(v,n)=>Array.isArray(v)&&v.length===n&&v.every(finite);
const identity=()=>[1,0,0,0,0,1,0,0,0,0,1,0,0,0,0,1];
// Row-major matrices, column vectors; composition applies the right operand first.
function mul(a,b){return Array.from({length:16},(_,i)=>{let s=0;for(let k=0;k<4;k++)s+=a[4*Math.floor(i/4)+k]*b[4*k+i%4];return s})}
function axis(v,q){const n=Math.hypot(...v);if(n<1e-12)throw Error('Zero rotation axis');const [x,y,z]=v.map(t=>t/n),c=Math.cos(q),s=Math.sin(q),t=1-c;return [t*x*x+c,t*x*y-s*z,t*x*z+s*y,0,t*x*y+s*z,t*y*y+c,t*y*z-s*x,0,t*x*z-s*y,t*y*z+s*x,t*z*z+c,0,0,0,0,1]}
function rigid(xyz,rpy){const m=mul(mul(axis([0,0,1],rpy[2]),axis([0,1,0],rpy[1])),axis([1,0,0],rpy[0]));[m[3],m[7],m[11]]=xyz;return m}
const point=(m,p)=>[0,1,2].map(r=>m[r*4]*p[0]+m[r*4+1]*p[1]+m[r*4+2]*p[2]+m[r*4+3]);
function order(items,ready,dependency){const pending=items.slice(),out=[];while(pending.length){const i=pending.findIndex(x=>dependency(x).every(d=>ready.has(d)));if(i<0)throw Error('Missing parent or cyclic dependency');const x=pending.splice(i,1)[0];out.push(x);ready.add(x.child||x.name)}return out}
function validateProfile(p){
 if(!p||!Array.isArray(p.joints)||!p.root_link)throw Error('Invalid profile');
 const names=new Set(),children=new Set([p.root_link]);
 for(const j of p.joints){if(names.has(j.name)||children.has(j.child))throw Error('Duplicate joint or child link');names.add(j.name);children.add(j.child);if(!['fixed','revolute','continuous'].includes(j.type)||!vec(j.origin_xyz_m,3)||!vec(j.origin_rpy_rad,3))throw Error('Invalid joint transform');if(j.type!=='fixed'&&(!vec(j.axis_xyz,3)||Math.hypot(...j.axis_xyz)<1e-12))throw Error('Invalid axis')}
 order(p.joints,new Set([p.root_link]),j=>[j.parent]);
 const maps=Object.values(p.control_mapping);if(maps.length!==6||new Set(maps.map(m=>m.index)).size!==6||new Set(maps.map(m=>m.driver_joint)).size!==6||maps.some(m=>!Number.isInteger(m.index)||m.index<0||m.index>5||!names.has(m.driver_joint)))throw Error('Expected six unique driver channels');
 const drivers=new Set(maps.map(m=>m.driver_joint));
 if(p.joints.some(j=>j.type!=='fixed'&&!drivers.has(j.name)&&!j.mimic_parent))throw Error('Undriven articulated joint');
 if(p.joints.some(j=>j.mimic_parent&&drivers.has(j.name)))throw Error('Driver cannot also mimic');
 order(p.joints.filter(j=>j.mimic_parent).map(j=>({...j,child:null})),new Set(drivers),j=>[j.mimic_parent]);
 return p;
}
function defaults(p){validateProfile(p);return {base_alignment:{translation_m:[0,0,0],rpy_rad:[0,0,0]},enable_candidate_live:false,actuators:Object.fromEntries(Object.entries(p.control_mapping).map(([k,m])=>[k,{raw_at_open:m.raw_at_open,raw_at_closed:m.raw_at_closed,open_angle_rad:m.angle_at_open_rad??m.proposed_open_angle_rad,closed_angle_rad:m.angle_at_closed_rad??m.proposed_closed_angle_rad,offset_rad:0,reversed:false}])),joints:Object.fromEntries(p.joints.map(j=>[j.name,{origin_xyz_m:j.origin_xyz_m.slice(),origin_rpy_rad:j.origin_rpy_rad.slice(),axis_xyz:j.axis_xyz?.slice()??null,zero_offset_rad:0,...(j.mimic_parent?{multiplier:j.mimic_multiplier,offset_rad:j.mimic_offset_rad??0}:{})}]))}}
function keys(v,allowed){if(!v||typeof v!=='object'||Array.isArray(v)||Object.keys(v).some(k=>!allowed.includes(k)))throw Error('Unknown calibration field')}
function validateCalibration(p,c){
 keys(c,['base_alignment','enable_candidate_live','actuators','joints']);keys(c.base_alignment,['translation_m','rpy_rad']);
 if(!vec(c.base_alignment.translation_m,3)||!vec(c.base_alignment.rpy_rad,3)||typeof c.enable_candidate_live!=='boolean')throw Error('Invalid base alignment');
 keys(c.actuators,Object.keys(p.control_mapping));if(Object.keys(c.actuators).length!==6)throw Error('Missing actuator calibration');
 for(const a of Object.values(c.actuators)){keys(a,['raw_at_open','raw_at_closed','open_angle_rad','closed_angle_rad','offset_rad','reversed']);if(![a.raw_at_open,a.raw_at_closed,a.open_angle_rad,a.closed_angle_rad,a.offset_rad].every(finite)||a.raw_at_open<0||a.raw_at_open>1000||a.raw_at_closed<0||a.raw_at_closed>1000||a.raw_at_open===a.raw_at_closed||typeof a.reversed!=='boolean')throw Error('Invalid actuator endpoints')}
 keys(c.joints,p.joints.map(j=>j.name));if(Object.keys(c.joints).length!==p.joints.length)throw Error('Missing joint calibration');
 for(const j of p.joints){const o=c.joints[j.name];keys(o,['origin_xyz_m','origin_rpy_rad','axis_xyz','zero_offset_rad',...(j.mimic_parent?['multiplier','offset_rad']:[])]);if(!vec(o.origin_xyz_m,3)||!vec(o.origin_rpy_rad,3)||!finite(o.zero_offset_rad)||j.type!=='fixed'&&(!vec(o.axis_xyz,3)||Math.hypot(...o.axis_xyz)<1e-12)||j.type==='fixed'&&o.axis_xyz!==null||j.mimic_parent&&![o.multiplier,o.offset_rad].every(finite))throw Error('Invalid joint calibration')}
 const numbers=[];function collect(o){if(typeof o==='number')numbers.push(o);else if(o&&typeof o==='object')Object.values(o).forEach(collect)}collect(c);
 if(numbers.some(x=>!finite(x)||Math.abs(x)>10000))throw Error('Calibration magnitude exceeds visual editor range');
 return c;
}
function solve(p,raw,c=defaults(p)){
 validateCalibration(p,c);if(!vec(raw,6)||raw.some(x=>x<0||x>1000))throw Error('Expected six ANGLE_ACT values in [0,1000]');
 const q={};for(const [k,m] of Object.entries(p.control_mapping)){const a=c.actuators[k];let t=(raw[m.index]-a.raw_at_closed)/(a.raw_at_open-a.raw_at_closed);if(a.reversed)t=1-t;q[m.driver_joint]=a.closed_angle_rad+t*(a.open_angle_rad-a.closed_angle_rad)+a.offset_rad+c.joints[m.driver_joint].zero_offset_rad}
 const mimics=order(p.joints.filter(j=>j.mimic_parent).map(j=>({...j,child:null})),new Set(Object.keys(q)),j=>[j.mimic_parent]);
 for(const j of mimics){const o=c.joints[j.name];q[j.name]=o.multiplier*q[j.mimic_parent]+o.offset_rad+o.zero_offset_rad}
 const frames={[p.root_link]:rigid(c.base_alignment.translation_m,c.base_alignment.rpy_rad)},warnings=[];
 for(const j of order(p.joints,new Set([p.root_link]),j=>[j.parent])){const o=c.joints[j.name];let t=rigid(o.origin_xyz_m,o.origin_rpy_rad);if(j.type!=='fixed'){t=mul(t,axis(o.axis_xyz,q[j.name]));if(q[j.name]<j.lower_rad-1e-8||q[j.name]>j.upper_rad+1e-8)warnings.push(j.name+' exceeds source limits: '+q[j.name].toFixed(4)+' rad')}frames[j.child]=mul(frames[j.parent],t)}
 return {q,frames,warnings};
}
function freshActual(msg,now=Date.now()/1000){const t=msg?.field_received_at?.angles_act;return !!msg&&finite(t)&&now-t>=-1&&now-t<=4&&vec(msg.angles_act,6)&&msg.angles_act.every(x=>x>=0&&x<=1000)}
function canDrive(p,c,msg,now){return freshActual(msg,now)&&(!Object.values(p.control_mapping).some(m=>m.mapping_status!=='source_defined')||c.enable_candidate_live)}
function saveBundle(referenceHash,profiles,calibrations){return {schema_version:1,reference_sha256:referenceHash,status:'UNVALIDATED_VISUAL_CALIBRATION',profiles:clone(calibrations)}}
function loadBundle(bundle,hash,profiles){keys(bundle,['schema_version','reference_sha256','status','profiles']);if(bundle.schema_version!==1||bundle.reference_sha256!==hash)throw Error('Calibration reference version mismatch');keys(bundle.profiles,Object.keys(profiles));if(Object.keys(bundle.profiles).length!==Object.keys(profiles).length)throw Error('Missing profile');for(const [id,p]of Object.entries(profiles))validateCalibration(p,bundle.profiles[id]);return clone(bundle.profiles)}
root.JointKinematics={clone,identity,mul,axis,rigid,point,validateProfile,defaults,validateCalibration,solve,freshActual,canDrive,saveBundle,loadBundle};
if(typeof module!=='undefined')module.exports=root.JointKinematics;
})(typeof globalThis!=='undefined'?globalThis:this);
