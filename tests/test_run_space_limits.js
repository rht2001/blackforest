const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const root=path.join(__dirname,'../digital_twin'),RunAPI=require(root+'/sensor_identification/runs.js');
// These regression checks explicitly exercise full-sweep reconstruction.
const R={...RunAPI,build:(rows,context,options={})=>RunAPI.build(rows,context,{pose_selection:'all',...options}),buildAsync:(rows,context,options={})=>RunAPI.buildAsync(rows,context,{pose_selection:'all',...options})};
const K=require(root+'/joint_kinematics.js'),T=require(root+'/tactile_points.js');
const read=n=>JSON.parse(fs.readFileSync(path.join(root,n),'utf8'));
const geometry=read('tactile_geometry.json'),profile=read('joint_geometry_profiles_for_codex.json').primary_profiles.rh56_tactile_toolkit_left;
const context={id:geometry.profile,profile,geometry,calibration:K.defaults(profile),boundaries:read('hand_boundaries.json')};
function row(index,bend=1000,matrix=Array.from({length:112},()=>Array(17).fill(0))){return {index:String(index),angles:JSON.stringify([1000,1000,1000,1000,bend,500]),tactile_matrix:matrix===null?'':JSON.stringify(matrix),state:'thumb_probing',slip_detected:'False'}}
const zero=R.build([row(1)],context);
assert.equal(zero.points.length,0);assert(zero.boundary_points.length>0);
assert.equal(zero.accumulation.frames_without_contact,1);assert.equal(zero.accumulation.frames_reconstructed,1);
assert(zero.boundary_points.some(p=>p.kind==='inactive_taxel'&&p.pressure===0));
assert(zero.boundary_points.some(p=>p.kind==='hand_boundary'&&p.pressure===null));
assert.equal(R.validateFrame(zero).points.length,0);
const missing=R.build([row(1,1000,null)],context);
assert(missing.boundary_points.some(p=>p.kind==='unknown_taxel'&&p.pressure===null));
assert(!missing.boundary_points.some(p=>p.kind==='inactive_taxel'));
const sweep=R.build([row(1),row(2,400)],context);
assert(sweep.boundary_points.length>zero.boundary_points.length,'Second no-contact pose must add swept limits');
const p1=zero.boundary_points.find(p=>p.taxel_id==='thumb_pad:0');
assert(sweep.boundary_points.some(p=>p.source_index==='1'&&p.taxel_id==='thumb_pad:0'));
assert(sweep.boundary_points.some(p=>p.source_index==='2'&&p.kind==='inactive_taxel'&&p.region.startsWith('thumb_')));
const closed=K.solve(profile,[1000,1000,1000,1000,400,500],context.calibration);
const expected=T.cells(geometry,closed,{},200).find(c=>c.id==='thumb_pad:0');
const inv=T.inverseRigid(closed.frames.left_base_link),xyz=K.point(inv,expected.position_world_m);
assert(sweep.boundary_points.some(p=>p.source_index==='2'&&Math.hypot(p.x-xyz[0],p.y-xyz[1],p.z-xyz[2])<.003));
assert.equal(new Set(sweep.boundary_points.map(p=>p.id)).size,sweep.boundary_points.length);
assert(sweep.boundary_points.every(p=>Math.abs(Math.hypot(...p.normal)-1)<1e-6));
assert(sweep.boundary_points.length<=4096);
assert.throws(()=>R.validateFrame({...zero,boundary_points:[{...zero.boundary_points[0],normal:[0,0,0]}]}),/Invalid oriented/);
assert.equal(sweep.hand_snapshot.angles_act[4],400,'Display pose must follow no-contact samples, too');
// Identical positions with opposite normals are distinct limiting half-spaces.
const custom={...context,boundaries:{points:[]},geometry:{...geometry,regions:[{...geometry.regions[0],cells:[{wire_index:0,row:0,col:0,point_m:[0,0,0],normal:[0,0,1]},{wire_index:1,row:0,col:1,point_m:[0,0,0],normal:[0,0,-1]}]}]}};
assert.equal(R.build([row(1)],custom).boundary_points.length,2);
console.log('PASS no-contact sweeps, inactive/unknown distinction, per-pose FK, body limits, opposite normals and constraint-only frame validation');
