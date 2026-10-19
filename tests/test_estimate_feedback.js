const assert=require('node:assert/strict');
const fs=require('node:fs'),path=require('node:path'),vm=require('node:vm');
const source=fs.readFileSync(path.join(__dirname,'../digital_twin/sensor_identification/simulation.js'),'utf8');
const context={module:{exports:{}},Date};vm.runInNewContext(source.split('/* One page:')[0],context);
const {render,elapsed}=context.module.exports;
assert.equal(elapsed(10,71),'1m 01s');
const running=render({status:'running',started_at:10,input_label:'<bad>',progress:{stage:'alignment',candidate:'SENSOR_A',completed:24,total:168}},'',71);
assert.ok(running.includes('1m 01s'));assert.ok(running.includes('24 / 168'));assert.ok(running.includes('&lt;bad&gt;'));assert.ok(!running.includes('<bad>'));
const fit={compatible:false,rmse_mm:1.2,max_error_mm:3.4,inlier_fraction:.75,hand_limit_max_mm:.8,contact_tolerance_mm:1.5,hand_tolerance_mm:.25,rejection_reasons:['contact_outside_tolerance','hand_penetration']};
const rejected=render({status:'complete',result:{classification:'AMBIGUOUS',contacts:12,fits:{SENSOR_A:fit}}});
for(const text of ['Neither model fits','Rejected','3.40 mm','9 / 12','crosses the hand limits','too far from the surface'])assert.ok(rejected.includes(text),text);
const provisional=render({status:'complete',result:{classification:'AMBIGUOUS',contacts:12,fits:{SENSOR_A:{...fit,compatible:true,rejection_reasons:[]}}}});
assert.ok(provisional.includes('Provisional fit'));assert.ok(provisional.includes('identity is not confirmed'));
const noContact=render({status:'complete',result:{classification:'AMBIGUOUS',contacts:0,hand_boundary_points:55,fits:{}}},'Hand sweep constrains space.');
assert.ok(noContact.includes('No anchored pose'));assert.ok(noContact.includes('55 hand limits'));
assert.ok(render({status:'connection_error',error:'<script>alert(1)</script>'}).includes('&lt;script&gt;'));
assert.doesNotThrow(()=>render({status:'complete',result:{fits:{SENSOR_A:{}}}}));
console.log('PASS running progress, elapsed time, fit diagnostics, ambiguity, no-contact feedback, missing metrics and HTML escaping');

const quickRejected=render({status:'complete',result:{quick_preview:true,fits:{SENSOR_A:{...fit,quick_preview:true}}}});
assert.ok(quickRejected.includes('Neither model fits'));
assert.ok(quickRejected.includes('Rejected'));
assert.ok(!quickRejected.includes('fit-good'));

// Both frontend guards must track the estimator protocol on disk.
const backendSource=fs.readFileSync(path.join(__dirname,'../digital_twin/sensor_identification/web_service.py'),'utf8');
const estimatorRevision=backendSource.match(/ESTIMATOR_REVISION = '([^']+)'/)[1];
const frontendRevisions=[...source.matchAll(/(?:state|backend)\.estimator_revision!=='([^']+)'/g)].map(match=>match[1]);
assert.deepEqual(frontendRevisions,[estimatorRevision,estimatorRevision]);
console.log('PASS frontend estimator checks match backend revision');

const conflicting=render({status:'complete',result:{classification:'AMBIGUOUS',geometry_conflict:{contact_count:5},
 contacts:12,fits:{SENSOR_A:{...fit,hand_limit_max_is_lower_bound:true,hand_validation_complete:false,
 hand_constraints_checked:2,hand_collision_count:2,rejection_reasons:['input_geometry_conflict']}}}});
for(const text of ['Contact / hand-model conflict','at least','Checked witness violations','contact-fit preview','sensor identity is undetermined'])assert.ok(conflicting.includes(text),text);
assert.ok(!conflicting.includes('Max hand penetration'));
assert.ok(!conflicting.includes('fit-good'));
console.log('PASS conflicting-evidence previews are rejected and partial hand checks are labelled as lower bounds');
