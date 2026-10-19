const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm'), assert = require('node:assert/strict');
const source = fs.readFileSync(path.join(__dirname, '../digital_twin/sensor_identification/simulation.js'), 'utf8');
const context = {module: {exports: {}}, Date};
vm.runInNewContext(source.split('/* One page:')[0], context);
const html = context.module.exports.render({status: 'complete', result: {
  placement_candidate: 'SENSOR_A', placement_reason: 'smaller_candidate_after_larger_rejected',
  fits: {SENSOR_B: {hand_collision_count: 3, hand_constraints_checked: 80}}
}});
assert.ok(html.includes('VEGABAR 29'));
assert.ok(html.includes('Rejected fit'));
const comparator = source.match(/\.sort\(\(a, b\) => \{([\s\S]*?)\}\);/)[1];
const result = {placement_candidate: 'SENSOR_B'};
const compare = new Function('result', 'a', 'b', comparator).bind(null, result);
const candidates = [['SENSOR_A', {compatible: true, score: .1}], ['SENSOR_B', {compatible: true, score: 2}]];
assert.equal(candidates.sort(compare)[0][0], 'SENSOR_B');
result.placement_candidate = 'SENSOR_A';
assert.equal(candidates.sort(compare)[0][0], 'SENSOR_A');
console.log('PASS larger-first viewer selection and collision feedback');

const reduced = context.module.exports.render({status:'complete',result:{contacts:80,fits:{SENSOR_A:{
  compatible:true,inlier_fraction:.5,input_contact_points:80,scored_contact_points:8,
  input_hand_points:20,hand_constraints_checked:1,hand_collision_count:0,point_spacing_mm:2
}}}});
assert.ok(reduced.includes('VEGABAR 29'));
assert.ok(reduced.includes('Provisional fit'));
console.log('PASS representative counts and retained-contact inlier denominator');
