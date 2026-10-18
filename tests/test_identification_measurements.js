const assert = require('node:assert/strict');
const {capture} = require('../digital_twin/sensor_identification/measurements.js');
const cloud = {valid:true, frame:'left_base_link', units:'metres', geometry_status:'UNVALIDATED',
  received_at:1000, field_received_at:{angles_act:1000,tactile_regions:1000},
  points:[{id:'index:0',position_m:[.01,.02,.03],value:800}]};
const cells = [{id:'index:0',x:.01,y:.02,z:.03,pressure:800},
               {id:'index:1',x:.02,y:.03,z:.04,pressure:0},
               {id:'index:2',x:.02,y:.03,z:.04,pressure:null}];
const viewer = {getContactCloud:()=>cloud, getTaxelPositions:()=>cells,
  ProfileTwin:{snapshot:()=>({live:true,demo:false,replay:null})}};
let frame = capture(viewer);
assert.equal(frame.units,'m');assert.equal(frame.frame,'left_base_link');
assert.equal(frame.points.length,3);assert.equal(frame.points[1].pressure,0);
assert.equal(frame.points[2].pressure,null);assert.equal(frame.points[0].x,.01);
frame.points[0].x=100;assert.equal(cells[0].x,.01);
viewer.ProfileTwin.snapshot=()=>({live:false,demo:false,replay:{name:'saved.json'}});
frame = capture(viewer);
assert.equal(frame.measurement_source,'saved_capture');assert.equal(frame.saved_name,'saved.json');
assert.equal(frame.points.length,1);assert.equal(frame.received_at,1000);
assert.equal(frame.points[0].pressure,800);assert.equal(frame.points[0].x,.01);
cloud.valid=false;assert.equal(capture(viewer).points.length,0);assert.equal(capture(viewer).valid,false);
cloud.valid=true;cloud.units='mm';assert.equal(capture(viewer).valid,false);
cloud.units='metres';cloud.frame='unknown';assert.equal(capture(viewer).valid,false);
assert.equal(capture({}).valid,false);
cloud.frame='left_base_link';cloud.valid=false;
viewer.ProfileTwin.snapshot=()=>({live:false,demo:false,replay:null});
frame=capture(viewer);assert.equal(frame.valid,true);assert.equal(frame.measurement_source,'manual_pose');
assert.ok(frame.points.every(p=>p.pressure===null));
viewer.ProfileTwin.snapshot=()=>({live:true,demo:false,replay:null});
assert.equal(capture(viewer).valid,false);
cloud.valid=true;viewer.ProfileTwin.snapshot=()=>({live:false,replay:{name:'recorded.csv'}});
viewer.HandHistory={points:()=>[{id:'known',x:0,y:0,z:0,pressure:700},{id:'unknown',x:1,y:0,z:0,pressure:null}]};
frame=capture(viewer);assert.equal(frame.points.length,2);assert.equal(frame.points[1].pressure,null);
console.log('PASS current metre-frame adapter, zero/missing readings, saved-capture isolation and invalid frames');
