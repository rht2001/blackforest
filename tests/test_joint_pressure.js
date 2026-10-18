const assert=require('node:assert/strict'),P=require('../digital_twin/joint_pressure');
assert.equal(P.regions.length,17);assert.equal(new Set(P.regions.map(r=>r.link)).size,17);
assert.equal(P.regions.find(r=>r.key==='thumb_tip').link,'left_thumb_force_sensor_4');
assert.equal(P.regions.find(r=>r.key==='index_pad').link,'left_index_force_sensor_1');
for(const [v,color]of [[0,'#00aa00'],[799,'#00aa00'],[800,'#cccc00'],[1999,'#cccc00'],[2000,'#ff8800'],[3199,'#ff8800'],[3200,'#ff0000'],[4095,'#ff0000']])assert.equal(P.color(v),color);
const packet={field_received_at:{tactile_regions:100},tactile_regions:{}};
P.regions.forEach((r,i)=>{packet.tactile_regions[r.key]=Array.from({length:r.rows},()=>Array(r.cols).fill(i*200))});
let values=P.sample(packet,true,101);assert.equal(Object.values(values).filter(v=>v.peak!==null).length,17);assert.equal(values.index_tip.peak,1800);assert.equal(values.palm.color,'#ff0000');
packet.tactile_regions.index_tip[1][1]=4095;assert.equal(P.sample(packet,true,101).index_tip.peak,4095);
for(const missing of [P.sample(packet,true,105),P.sample(packet,false,101),P.sample({...packet,field_received_at:{tactile_regions:110}},true,101),P.sample({...packet,field_received_at:{}},true,101)])assert.ok(Object.values(missing).every(v=>v.peak===null&&v.color===P.missing));
for(const bad of [null,[[1]],Array.from({length:3},()=>Array(3).fill(NaN)),Array.from({length:3},()=>Array(3).fill(-1)),Array.from({length:3},()=>Array(3).fill('300'))]){packet.tactile_regions.index_tip=bad;assert.equal(P.sample(packet,true,101).index_tip.color,P.missing)}
const demo=P.sample(P.demo(1),true);assert.equal(Object.values(demo).filter(v=>v.peak!==null).length,17);
console.log('PASS pressure mapping, 17 distinct sensor links, thresholds, peaks, stale/missing/invalid data and demo');
