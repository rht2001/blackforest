const fs=require('node:fs'),path=require('node:path'),K=require('../digital_twin/joint_kinematics');
const reference=JSON.parse(fs.readFileSync(path.join(__dirname,'../digital_twin/joint_geometry_profiles_for_codex.json')));
const profiles={rh56_tactile_toolkit_left:reference.primary_profiles.rh56_tactile_toolkit_left};
const report={units:'metres/radians',frame:'Each profile own root frame; NO cross-model registration or measured hardware data',profiles:{}};
for(const [id,p]of Object.entries(profiles)){
 const poses={open:Array(6).fill(1000),half:Array(6).fill(500),closed:Array(6).fill(0)};
 for(const [channel,m]of Object.entries(p.control_mapping)){poses['sweep_'+channel]=Array(6).fill(1000);poses['sweep_'+channel][m.index]=0}
 report.profiles[id]=Object.fromEntries(Object.entries(poses).map(([name,raw])=>[name,{raw,...K.solve(p,raw)}]));
}
if(require.main===module)process.stdout.write(JSON.stringify(report,null,2));
module.exports=report;
