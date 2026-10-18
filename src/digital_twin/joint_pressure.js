/* Region pressure -> matching Toolkit sensor body; calibrated sensor counts. */
(function(root){
'use strict';
const regions=[];
for(const finger of ['little','ring','middle','index'])for(const [zone,number,rows,cols]of [['tip',3,3,3],['nail',2,12,8],['pad',1,10,8]])regions.push({key:finger+'_'+zone,link:'left_'+finger+'_force_sensor_'+number,rows,cols});
for(const [zone,number,rows,cols]of [['tip',4,3,3],['nail',3,12,8],['mid',2,3,3],['pad',1,12,8]])regions.push({key:'thumb_'+zone,link:'left_thumb_force_sensor_'+number,rows,cols});
regions.push({key:'palm',link:'left_palm_force_sensor',rows:8,cols:14});
const missing='#536171',color=value=>value<800?'#00aa00':value<2000?'#cccc00':value<3200?'#ff8800':'#ff0000';
function fresh(packet,now){const t=packet?.field_received_at?.tactile_regions;return typeof t==='number'&&Number.isFinite(t)&&now-t>=-1&&now-t<=4}
function sample(packet,enabled,now=Date.now()/1000){const available=enabled&&fresh(packet,now),result={};for(const region of regions){const matrix=available?packet.tactile_regions?.[region.key]:null;const valid=Array.isArray(matrix)&&matrix.length===region.rows&&matrix.every(row=>Array.isArray(row)&&row.length===region.cols&&row.every(x=>typeof x==='number'&&Number.isFinite(x)&&x>=0&&x<=4095));const values=valid?matrix.flat():[],peak=valid?Math.max(...values):null;result[region.key]={link:region.link,peak,values:valid?values:null,color:valid?color(peak):missing}}return result}
function demo(seconds){const matrices={};regions.forEach((r,i)=>{const values=Array.from({length:r.rows*r.cols},(_,index)=>{const [row,col]=r.key==='palm'?[r.rows-1-index%r.rows,Math.floor(index/r.rows)]:[Math.floor(index/r.cols),index%r.cols],u=(col+.5)/r.cols,v=(row+.5)/r.rows,cx=.5+.2*Math.sin(seconds+i),cy=.5+.15*Math.cos(seconds*.8+i);return Math.round(4095*Math.exp(-((u-cx)**2+(v-cy)**2)/.045))});matrices[r.key]=Array.from({length:r.rows},(_,row)=>values.slice(row*r.cols,(row+1)*r.cols))});return {simulated:true,field_received_at:{tactile_regions:Date.now()/1000},tactile_regions:matrices}}
root.JointPressure={regions,missing,color,sample,demo};if(typeof module!=='undefined')module.exports=root.JointPressure;
})(typeof globalThis!=='undefined'?globalThis:this);
