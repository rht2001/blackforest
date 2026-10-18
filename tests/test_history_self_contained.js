const fs=require('node:fs'),vm=require('node:vm'),assert=require('node:assert/strict'),path=require('node:path');
const root=path.join(__dirname,'../digital_twin');
const context={console,setTimeout};vm.createContext(context);
// Reproduce an older server that serves history.js but 404s runs.js.
for(const file of ['joint_kinematics.js','joint_pressure.js','tactile_points.js','sensor_identification/history.js'])vm.runInContext(fs.readFileSync(path.join(root,file),'utf8'),context,{filename:file});
assert.equal(typeof context.IdentificationHistory.records,'function');
assert.equal(typeof context.IdentificationRuns.buildAsync,'function');
assert.equal(context.IdentificationRuns,context.IdentificationHistory.runs);
assert(!fs.readFileSync(path.join(root,'hand_twin_6dof.html'),'utf8').includes('<script src="sensor_identification/runs.js">'));
console.log('PASS run builder available from history.js alone, without the new server route');
