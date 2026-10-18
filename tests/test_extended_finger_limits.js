const assert=require('node:assert/strict'),fs=require('node:fs'),path=require('node:path');
const root=path.resolve(__dirname,'../digital_twin');
const R=require(root+'/sensor_identification/runs.js'),K=require(root+'/joint_kinematics.js');
const read=name=>JSON.parse(fs.readFileSync(path.join(root,name),'utf8'));
const geometry=read('tactile_geometry.json'),profile=read('joint_geometry_profiles_for_codex.json').primary_profiles.rh56_tactile_toolkit_left;
const context={id:geometry.profile,profile,calibration:K.defaults(profile),
 geometry:{...geometry,regions:geometry.regions.map(r=>({...r,cells:r.cells.slice(0,3)}))},
 boundaries:{...read('hand_boundaries.json'),points:read('hand_boundaries.json').points.filter((p,i,a)=>a.findIndex(q=>q.link===p.link)===i)}};
function row(index,angles,extra={}){
 const matrix=Array.from({length:112},()=>Array(17).fill(0));
 matrix[0][0]=700;matrix[0][15]=700;
 return {index:String(index),state:'thumb_probing',slip_detected:'False',angles:JSON.stringify(angles),tactile_matrix:JSON.stringify(matrix),...extra};
}
const rows=[row(1,[1000,1000,1000,1000,1000,500]),row(2,[100,900,800,700,700,500]),
 row(3,[600,100,200,300,400,600]),row(4,[500,600,100,200,300,700]),
 row(5,[0,0,0,0,0,500],{slip_detected:'True'}),row(6,[0,0,0,0,0,500],{state:'resetting'})];
const original=JSON.stringify(rows),reduced=R.build(rows,context),sweep=R.build(rows,context,{finger_limit_selection:'all'});
const finger=p=>p.region?.split('_')[0]||p.link.replace(/^left_/,'').split('_')[0];
for(const [name,index] of Object.entries({little:'2',ring:'3',middle:'4',index:'4'})){
 const points=reduced.boundary_points.filter(p=>finger(p)===name);
 assert(points.length>0,name);
 assert(points.every(p=>p.source_index===index),name+' must retain only its deepest pose');
 assert(points.some(p=>p.kind==='hand_boundary'),name+' body limits');
 assert(points.some(p=>p.taxel_id),name+' taxel limits');
}
assert(reduced.boundary_points.length<sweep.boundary_points.length);
assert.deepEqual(reduced.points,sweep.points,'Measured contact evidence stays unchanged');
const fixed=p=>['thumb','palm','base'].includes(finger(p));
const canonical=a=>a.filter(fixed).map(({id,...p})=>p);
assert.deepEqual(canonical(reduced.boundary_points),canonical(sweep.boundary_points));
assert.equal(JSON.stringify(rows),original,'Source rows stay unchanged');
const reverse={...context,calibration:K.clone(context.calibration)};
reverse.calibration.actuators.little.reversed=true;
assert(R.build(rows,reverse).boundary_points.filter(p=>finger(p)==='little').every(p=>p.source_index==='1'));
assert.equal(reduced.accumulation.finger_limit_selection,'maximum_bend_per_finger');
assert.equal(R.validateFrame(reduced).boundary_points.length,reduced.boundary_points.length);
(async()=>{
 assert.deepEqual(await R.buildAsync(rows,context),reduced);
 console.log(`PASS independent finger poses, body/taxel pruning, contacts/thumb/palm preserved, reversed calibration, excluded slips/states, source preservation and async parity (${sweep.boundary_points.length} -> ${reduced.boundary_points.length} limits)`);
})().catch(e=>{console.error(e);process.exitCode=1});
