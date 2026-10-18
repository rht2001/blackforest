const fs = require('node:fs'), path = require('node:path'), vm = require('node:vm'), assert = require('node:assert/strict');
const source = fs.readFileSync(path.join(__dirname, '../digital_twin/sensor_identification/simulation.js'), 'utf8');
const context = {module: {exports: {}}, Date};
vm.runInNewContext(source.split('/* One page:')[0], context);
const html = context.module.exports.render({status: 'complete', result: {
  placement_candidate: 'SENSOR_A', placement_reason: 'smaller_candidate_after_larger_rejected',
  fits: {SENSOR_B: {hand_collision_count: 3, hand_constraints_checked: 80}}
}});
assert.ok(html.includes('3 / 80'));
assert.ok(html.includes('after the larger sensor failed the fit limits'));
const comparator = source.match(/\.sort\(\(a, b\) => \{([\s\S]*?)\}\);/)[1];
const result = {placement_candidate: 'SENSOR_B'};
const compare = new Function('result', 'a', 'b', comparator).bind(null, result);
const candidates = [['SENSOR_A', {compatible: true, score: .1}], ['SENSOR_B', {compatible: true, score: 2}]];
assert.equal(candidates.sort(compare)[0][0], 'SENSOR_B');
result.placement_candidate = 'SENSOR_A';
assert.equal(candidates.sort(compare)[0][0], 'SENSOR_A');
console.log('PASS larger-first viewer selection and collision feedback');
