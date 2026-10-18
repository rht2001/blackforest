(function toolkitControls(){
'use strict';
const names=['Little finger','Ring finger','Middle finger','Index finger','Thumb bending','Thumb rotation'];
const keys=['little','ring','middle','index','thumb','thumb_rotation'];
const palette=['#60bcec','#63cfae','#c0a4f7','#5fe5fb','#f5bd75','#f8d08c'];
const positions=Array(6).fill(1000);
const controls=document.getElementById('controls');
let live=false,pending=false,animating=false,renderedOnce=false,lastRender=0,received=0,lastPacket=null,requestedLive=false;
const srcData={},buffers={},partPose={};let queued=false,demo=false;let sensorPacket=null;
for(let i=0;i<6;i++){
  const div=document.createElement('section');div.className='channel';div.innerHTML=`<div class="channel-head"><span class="channel-name" style="color:${palette[i]}">${names[i]}</span><span class="channel-info" id="info${i}">Position ${positions[i]}</span></div><div class="channel-main"><input type="range" id="s${i}" min="0" max="1000" value="1000"><input type="number" id="n${i}" min="0" max="1000" value="1000"></div><div class="tactile"><span id="touch${i}"></span></div><div class="channel-info" id="sensorInfo${i}" style="margin-top:5px">Force: – · Tactile max: –</div>`;controls.appendChild(div);
  for(const prefix of ['s','n'])document.getElementById(prefix+i).addEventListener(prefix==='s'?'input':'change',e=>{if(live)return;demo=false;window.ProfileTwin?.demo(false);document.getElementById('animateDemo').textContent='Motion demo';setPos(i,e.target.value)});
}
function setPos(i,v){const n=Number(v);if(!Number.isFinite(n))return;const next=Math.max(0,Math.min(1000,Math.round(n)));if(positions[i]===next)return;positions[i]=next;document.getElementById('s'+i).value=positions[i];document.getElementById('n'+i).value=positions[i];document.getElementById('info'+i).textContent='Position '+positions[i];schedule();}
function setAll(value){if(live)return;for(let i=0;i<6;i++)setPos(i,value)}
const clamp=(v,a,b)=>Math.max(a,Math.min(b,v));

function schedule(){if(!live)window.ProfileTwin?.manual(positions)}
function fresh(msg,key){
 const stamp=msg.field_received_at?.[key];
 return !!stamp && Date.now()/1000-stamp<=4;
}
function setLive(l){live=l;window.ProfileTwin?.mode(l,positions);document.getElementById('modeManual').classList.toggle('active',!l);document.getElementById('modeLive').classList.toggle('active',l);
 if(!l){document.getElementById('dataSource').textContent='Data source: no fresh hand data';document.getElementById('actuatorPosition').textContent='Actuator positions: –';document.getElementById('fieldMeans').textContent='Region means: –';}
 demo=false;document.getElementById('animateDemo').textContent='Motion demo';
 for(let i=0;i<6;i++){document.getElementById('s'+i).disabled=l;document.getElementById('n'+i).disabled=l;}
 document.getElementById('openAll').disabled=l;document.getElementById('closeAll').disabled=l;
 if(l){document.getElementById('liveStatus').textContent='Connecting to local Python bridge …';poll();}
 else {sensorPacket=null;for(let i=0;i<6;i++){document.getElementById('sensorInfo'+i).textContent='Force: – · Tactile max: –';document.getElementById('touch'+i).style.width='0%';}document.getElementById('palmInfo').textContent='Palm · Tactile max: –';schedule();document.getElementById('statusDot').className='dot';document.getElementById('liveStatus').textContent='Manual preview · no hardware connection';}
}
async function poll(){if(!live||requestedLive)return;requestedLive=true;
 const dot=document.getElementById('statusDot');const status=document.getElementById('liveStatus');const root=document.getElementById('bridgeUrl').value.trim().replace(/\/$/,'');
 try{const response=await fetch(root+'/api/state',{cache:'no-store',signal:AbortSignal.timeout(1250)});if(!response.ok)throw new Error('HTTP '+response.status);const msg=await response.json();if(!live)return;lastPacket=msg;sensorPacket=msg;window.ProfileTwin?.receive(msg);schedule();
  const age=msg.received_at?Date.now()/1000-msg.received_at:999;
  document.getElementById('dataSource').textContent=age<=4?'Data source: '+(msg.source||'Python-Bridge')+' · '+(msg.simulated?'SIMULATION':'Telemetry'):'Data source: no fresh hand data';
  document.getElementById('actuatorPosition').textContent='Actuator positions: '+(fresh(msg,'position_act')&&Array.isArray(msg.position_act)?msg.position_act.join(' · '):'–');
  document.getElementById('fieldMeans').textContent=fresh(msg,'tactile_means')?Object.entries(msg.tactile_means||{}).map(([key,v])=>key+': '+Number(v).toFixed(1)).join(String.fromCharCode(10)):'Region means: –';
  if(age>4){dot.className='dot bad';status.textContent='Connected · no fresh data';}
  else {dot.className='dot good';status.textContent='Live · last update age: '+Math.max(0,age).toFixed(1)+' s';}
  const keys=['angles_act'];
  const source=JointKinematics.freshActual(msg)?'angles_act':undefined;const arr=msg[source];if(source)status.textContent+=' · Actual readings';
  if(arr&&age<=4){arr.forEach((v,i)=>{if(Number.isFinite(Number(v)))setPos(i,v)});}
  const force=fresh(msg,'force_act')&&Array.isArray(msg.force_act)?msg.force_act:[];const touch=fresh(msg,'tactile')?msg.tactile||{}:{};
  for(let i=0;i<6;i++){const t=touch[i===5?'thumb':keysForFinger[i]];
   const raw=typeof t==='number'?t:t?.max,val=raw==null?NaN:Number(raw);document.getElementById('touch'+i).style.width=(Number.isFinite(val)?clamp(val/4000*100,0,100):0)+'%';document.getElementById('touch'+i).style.background=Number.isFinite(val)?pressureColor(val):'#293541';
   const forceText=force[i]==null?'–':Number(force[i]).toFixed(0)+' g';
   document.getElementById('sensorInfo'+i).textContent='Force: '+forceText+' · Tactile max: '+(Number.isFinite(val)?val.toFixed(0):'–');
  }
  document.getElementById('palmInfo').textContent='Palm · Tactile max: '+(touch.palm?.max ?? touch.palm ?? '–');
  document.getElementById('lastPacket').textContent='Last data received: '+(msg.received_at?new Date(msg.received_at*1000).toLocaleTimeString('en-GB'):'none yet')+' · '+(msg.seq??0)+' packets';
 }catch(e){window.ProfileTwin?.disconnect();sensorPacket=null;clearPressureReadouts();schedule();dot.className='dot bad';status.textContent='No connection: '+e.message;document.getElementById('dataSource').textContent='Data source: Connection lost';document.getElementById('actuatorPosition').textContent='Actuator positions: –';document.getElementById('fieldMeans').textContent='Region means: –';}
 finally{requestedLive=false;}
}
function clearPressureReadouts(){
 for(let i=0;i<6;i++){document.getElementById('sensorInfo'+i).textContent='Force: – · Tactile max: –';document.getElementById('touch'+i).style.width='0%';}
 document.getElementById('palmInfo').textContent='Palm · Tactile max: –';
}
const keysForFinger=['little','ring','middle','index','thumb'];
document.getElementById('modeManual').onclick=()=>setLive(false);document.getElementById('modeLive').onclick=()=>setLive(true);
document.getElementById('checkBridge').onclick=()=>{setLive(true);poll();};
document.getElementById('liveField').onchange=()=>{if(live)poll();};
document.getElementById('openAll').onclick=()=>setAll(1000);document.getElementById('closeAll').onclick=()=>setAll(0);
document.getElementById('quickOpen').onclick=()=>{setLive(false);setAll(1000)};document.getElementById('quickClose').onclick=()=>{setLive(false);setAll(0)};
document.getElementById('animateDemo').onclick=()=>{const next=!demo;setLive(false);demo=next;window.ProfileTwin?.demo(demo);document.getElementById('animateDemo').textContent=demo?'Stop demo':'Motion demo';};
setInterval(()=>{if(demo){const p=Math.round(500+500*Math.cos(performance.now()/1600));setAll(p)}},60);

document.getElementById('resetView').onclick=()=>document.getElementById('profileCamera').click();
window.setHandAngles=(arr)=>{if(!Array.isArray(arr)||arr.length!==6)throw Error('Need six positions');arr.forEach((p,i)=>setPos(i,p));};
window.getHandAngles=()=>positions.slice();
setInterval(()=>{if(live)poll()},300);
})();
