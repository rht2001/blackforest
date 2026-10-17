/* Independent grasp objects. Mesh-local metres -> hand-base metres -> display. */
(function(root){
'use strict';
const K=root.JointKinematics;
const clone=value=>JSON.parse(JSON.stringify(value));
const defaults=id=>({id,position_m:[id==='vegabar_29'?-.12:.13,0,.12],rotation_rad:[Math.PI/2,0,0],visible:true});
function valid(state,id){return !!state&&state.id===id&&typeof state.visible==='boolean'&&['position_m','rotation_rad'].every(key=>Array.isArray(state[key])&&state[key].length===3&&state[key].every(x=>typeof x==='number'&&Number.isFinite(x)&&Math.abs(x)<=(key==='position_m'?1:2*Math.PI)))}
function points(model,state,toDisplay=p=>p){
 const transform=K.rigid(state.position_m,state.rotation_rad),out=[];
 for(let i=0;i<model.vertices.length;i+=3)out.push(toDisplay(K.point(transform,model.vertices.slice(i,i+3))));
 return out;
}
function attach({models,traces,plot,toDisplay}){
 const $=id=>document.getElementById(id),select=$('sensorObjectSelect'),states=models.map(m=>defaults(m.id));
 const storageKey='rh56-sensor-objects:'+models.map(m=>m.source_sha256).join(':');
 try{const saved=JSON.parse(localStorage.getItem(storageKey)||'null');if(Array.isArray(saved)&&saved.length===states.length&&saved.every((s,i)=>valid(s,models[i].id)))saved.forEach((s,i)=>states[i]=s)}catch{}
 const indices=models.map(model=>{const index=traces.length;traces.push({name:model.name,type:'mesh3d',color:model.color,x:[],y:[],z:[],i:model.indices.filter((_,i)=>i%3===0),j:model.indices.filter((_,i)=>i%3===1),k:model.indices.filter((_,i)=>i%3===2)});const option=document.createElement('option');option.value=model.id;option.textContent=model.name;select.appendChild(option);return index});
 let queued=false;
 const pending=new Set();
 const selected=()=>models.findIndex(m=>m.id===select.value);
 const fields=['X','Y','Z','RX','RY','RZ'];
 function save(){try{localStorage.setItem(storageKey,JSON.stringify(states))}catch{}}
 function controls(){const state=states[selected()];fields.forEach((key,k)=>{$('sensorObject'+key).value=Number((k<3?state.position_m[k]*1000:state.rotation_rad[k-3]*180/Math.PI).toFixed(2))});$('sensorObjectVisible').checked=state.visible;select.style.borderColor=models[selected()].color}
 function render(i){const trace=traces[indices[i]],xyz=points(models[i],states[i],toDisplay);trace.x=xyz.map(p=>p[0]);trace.y=xyz.map(p=>p[1]);trace.z=xyz.map(p=>p[2]);trace.visible=states[i].visible;if(plot._renderer)HandRenderer.restyle(plot,{x:[trace.x],y:[trace.y],z:[trace.z],visible:[trace.visible]},[indices[i]])}
 function schedule(i){pending.add(i);if(queued)return;queued=true;requestAnimationFrame(()=>{queued=false;for(const index of pending)render(index);pending.clear()})}
 select.onchange=controls;
 fields.forEach((key,k)=>{$('sensorObject'+key).addEventListener('input',()=>{const input=$('sensorObject'+key),v=input.valueAsNumber;if(!Number.isFinite(v)||!input.checkValidity())return;const state=states[selected()];if(k<3)state.position_m[k]=v/1000;else state.rotation_rad[k-3]=v*Math.PI/180;save();schedule(selected())})});
 $('sensorObjectVisible').onchange=()=>{states[selected()].visible=$('sensorObjectVisible').checked;save();schedule(selected())};
 $('sensorObjectReset').onclick=()=>{states[selected()]=defaults(models[selected()].id);controls();save();schedule(selected())};
 $('sensorObjectFocus').onclick=()=>plot._renderer?.focus(toDisplay(states[selected()].position_m),[.2,-1,.25],300);
 models.forEach((_,i)=>render(i));controls();
 function connectDrag(){
  const canvas=plot.querySelector('canvas');let drag=null;
  const origin=toDisplay([0,0,0]),basis=[[1,0,0],[0,1,0],[0,0,1]].map(p=>toDisplay(p).map((v,k)=>(v-origin[k])/1000));
  canvas.addEventListener('pointerdown',e=>{if(!e.shiftKey||e.button!==0||!states[selected()].visible)return;e.preventDefault();e.stopImmediatePropagation();drag={index:selected(),x:e.clientX,y:e.clientY};canvas.setPointerCapture(e.pointerId)},true);
  canvas.addEventListener('pointermove',e=>{if(!drag)return;e.preventDefault();e.stopImmediatePropagation();const state=states[drag.index],delta=plot._renderer.screenDelta(toDisplay(state.position_m),e.clientX-drag.x,e.clientY-drag.y);state.position_m=state.position_m.map((v,k)=>Math.max(-1,Math.min(1,v+basis[k].reduce((s,x,j)=>s+x*delta[j],0)/1000)));drag.x=e.clientX;drag.y=e.clientY;controls();save();schedule(drag.index)},true);
  const finish=e=>{if(!drag)return;e.stopImmediatePropagation();drag=null};canvas.addEventListener('pointerup',finish,true);canvas.addEventListener('pointercancel',finish,true);canvas.addEventListener('lostpointercapture',()=>{drag=null});
 }
 return {connectDrag,snapshot:()=>clone(states)};
}
root.SensorObjects={defaults,valid,points,attach};if(typeof module!=='undefined')module.exports=root.SensorObjects;
})(typeof globalThis!=='undefined'?globalThis:this);
