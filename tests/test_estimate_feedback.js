const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../digital_twin/sensor_identification/simulation.js'),'utf8');
const context={module:{exports:{}},Date};vm.runInNewContext(source.split('/* One page:')[0],context);
const {render,elapsed}=context.module.exports;
assert.equal(elapsed(10,71),'1m 01s');
const running=render({status:'running',progress:{stage:'alignment',candidate:'SENSOR_A'}});
assert.ok(running.includes('VEGABAR 29'));assert.ok(running.includes('Matching surfaces'));
const transform=[[1,0,0,10],[0,1,0,20],[0,0,1,30],[0,0,0,1]];
for(const [compatible,status] of [[false,'Rejected fit'],[true,'Provisional fit']]){
 const html=render({status:'complete',result:{fits:{SENSOR_A:{compatible,transform}}}},'',0,{models:[{name:'<bad>',source_center_mm:[0,0,0]}]});
 assert.ok(html.includes(status));assert.ok(html.includes('10.00 mm'));assert.ok(html.includes('30.00 mm'));
 assert.ok(html.includes('&lt;bad&gt;'));assert.ok(!html.includes('<bad>'));
}
assert.ok(render({status:'connection_error'}).includes('Retry'));
assert.ok(render({status:'complete',result:{fits:{}}}).includes('No position found'));
assert.doesNotThrow(()=>render({status:'complete',result:{fits:{SENSOR_A:{}}}}));
console.log('PASS compact status, pose values, provisional/rejected distinction, escaping and retry');
// Both frontend guards must track the estimator protocol on disk.
const backendSource=fs.readFileSync(path.join(__dirname,'../digital_twin/sensor_identification/web_service.py'),'utf8');
const estimatorRevision=backendSource.match(/ESTIMATOR_REVISION = '([^']+)'/)[1];
const frontendRevisions=[...source.matchAll(/(?:state|backend)\.estimator_revision!=='([^']+)'/g)].map(match=>match[1]);
assert.deepEqual(frontendRevisions,[estimatorRevision,estimatorRevision]);
console.log('PASS frontend estimator checks match backend revision');
