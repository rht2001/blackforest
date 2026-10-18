const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
global.JointKinematics=require('../digital_twin/joint_kinematics.js');
const K=JointKinematics,T=require('../digital_twin/tactile_points.js');
const B=require('../digital_twin/sensor_identification/boundaries.js');
const {capture}=require('../digital_twin/sensor_identification/measurements.js');
const html=fs.readFileSync(path.join(__dirname,'../digital_twin/hand_twin_6dof.html'),'utf8');
const d=JSON.parse(html.match(/<script id="jointProfileData" type="application\/json">(.*?)<\/script>/s)[1]);
const b=JSON.parse(html.match(/<script id="handBoundaryData" type="application\/json">(.*?)<\/script>/s)[1]);
const p=d.reference.primary_profiles.rh56_tactile_toolkit_left;
const open=K.solve(p,Array(6).fill(1000),K.defaults(p));
const closed=K.solve(p,Array(6).fill(0),K.defaults(p));
const a=B.transform(b,open,K,T),c=B.transform(b,closed,K,T);
assert.equal(a.length,379);
const vec=p=>[p.x,p.y,p.z],dist=(p,q)=>Math.hypot(...p.map((x,i)=>x-q[i]));
const taxels=T.cells(d.tactile_geometry,open,{}).map(c=>K.point(T.inverseRigid(open.frames.left_base_link),c.position_world_m));
for(let i=0;i<a.length;i++){
 assert.ok(Math.abs(Math.hypot(...a[i].normal)-1)<1e-8);
 for(let j=0;j<i;j++)assert.ok(dist(vec(a[i]),vec(a[j]))>=.005-1e-8);
 assert.ok(taxels.every(t=>dist(vec(a[i]),t)>=.003-1e-8));
 if(a[i].link==='left_base_link')assert.ok(dist(vec(a[i]),vec(c[i]))<1e-9);
}
assert.ok(a.some((p,i)=>p.link.startsWith('left_index')&&dist(vec(p),vec(c[i]))>.01));
const viewer={getContactCloud:()=>({valid:true,frame:'left_base_link',units:'metres'}),getTaxelPositions:()=>[],getHandBoundaryPoints:()=>a,ProfileTwin:{snapshot:()=>({live:true})}};
const f=capture(viewer);assert.equal(f.boundary_points.length,379);assert.equal(f.points.length,0);
f.boundary_points[0].normal[0]=999;assert.notEqual(a[0].normal[0],999);
viewer.getContactCloud=()=>({valid:false});assert.equal(capture(viewer).boundary_points.length,0);
console.log('PASS 379 palmar boundaries, spacing/taxel halo, articulated FK, unit normals, separate frozen evidence');
