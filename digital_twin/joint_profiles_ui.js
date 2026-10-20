/* Read-only live motion and pressure. Uses the existing poller's packets. */
(function profileExtension(){
'use strict';
const K=JointKinematics,P=JointPressure,T=TactilePoints,$=id=>document.getElementById(id),data=JSON.parse($('jointProfileData').textContent),id='rh56_tactile_toolkit_left',profile=data.reference.primary_profiles[id],geometry=data.tactile_geometry;
let calibration=K.defaults(profile),live=false,packet=null,disconnected=false,demo=false,raw=Array(6).fill(1000),view=null,manualQueued=false;
// Preserve previously saved Toolkit geometry without exposing editing controls.
try{const saved=JSON.parse(localStorage.getItem('rh56-joint-calibration:'+data.reference_sha256)||'null');if(saved?.reference_sha256===data.reference_sha256&&saved.profiles?.[id])calibration=K.clone(K.validateCalibration(profile,saved.profiles[id]))}catch{}
calibration.enable_candidate_live=true;
const assets=data.assets[id],traces=assets.map(a=>{const n=a.vertices.length/3;return {type:'mesh3d',color:a.sensor?'#263846':'#a4b6ca',x:Array(n).fill(0),y:Array(n).fill(0),z:Array(n).fill(0),i:a.indices.filter((_,i)=>i%3===0),j:a.indices.filter((_,i)=>i%3===1),k:a.indices.filter((_,i)=>i%3===2)}});
const pointIndex=traces.length,centroidIndex=pointIndex+1;
traces.push({name:'Taxels',type:'scatter3d',mode:'markers',x:[],y:[],z:[],marker:{size:3.4,rgb:[]}}, {name:'Contact centres',type:'scatter3d',mode:'markers',color:'#247dff',x:[],y:[],z:[],marker:{size:8,rgb:[]}});
function meshPoints(a,solution){const m=K.mul(solution.frames[a.link],K.rigid(a.origin_xyz_m,a.origin_rpy_rad)),out=[];for(let i=0;i<a.vertices.length;i+=3)out.push(K.point(m,a.vertices.slice(i,i+3).map((v,k)=>v*a.scale[k])));return out}
const displayRotation=K.rigid([0,0,0],[0,-Math.PI/2,0]),bounds=[[Infinity,Infinity,Infinity],[-Infinity,-Infinity,-Infinity]],open=K.solve(profile,Array(6).fill(1000),K.defaults(profile));
for(const a of assets)for(const p of meshPoints(a,open)){const q=K.point(displayRotation,p);q.forEach((x,i)=>{bounds[0][i]=Math.min(bounds[0][i],x);bounds[1][i]=Math.max(bounds[1][i],x)})}
const center=bounds[0].map((x,i)=>(x+bounds[1][i])/2);
const meshOrigins=assets.map(a=>K.rigid(a.origin_xyz_m,a.origin_rpy_rad)),meshFrames=[];
function updateMeshes(solution){
 const changed=[];
 assets.forEach((a,index)=>{
  const frame=solution.frames[a.link],old=meshFrames[index];
  if(old&&frame.every((v,k)=>Math.abs(v-old[k])<1e-12))return;
  meshFrames[index]=frame.slice();changed.push(index);
  const m=K.mul(displayRotation,K.mul(frame,meshOrigins[index])),v=a.vertices,t=traces[index],scale=a.scale;
  for(let j=0,i=0;i<v.length;i+=3,j++){
   const x=v[i]*scale[0],y=v[i+1]*scale[1],z=v[i+2]*scale[2];
   t.x[j]=(m[0]*x+m[1]*y+m[2]*z+m[3]-center[0])*1000;
   t.y[j]=(m[4]*x+m[5]*y+m[6]*z+m[7]-center[1])*1000+15;
   t.z[j]=(m[8]*x+m[9]*y+m[10]*z+m[11]-center[2])*1000+130;
  }
 });return changed;
}
function pose(values){view.solution=K.solve(profile,values,calibration);view.raw=values.slice();return updateMeshes(view.solution)}
let pressures=P.sample(null,false),signature=null,overlaySignature=null,renderedCells=[],lastInput=null,taxelPositions=[],poseSignature=null,positionSignature=null,publishedSignature=null,publishing=false,lastPublished=0;
const rgb=hex=>[1,3,5].map(i=>parseInt(hex.slice(i,i+2),16)/255);
function present(point){return K.point(displayRotation,point).map((x,i)=>(x-center[i])*1000+[0,15,130][i])}
function currentCloud(){return T.cloud(geometry,view.solution,P.sample(lastInput,demo||(live&&!disconnected)),lastInput,demo||(live&&!disconnected),$('activationThreshold').value,calibration)}
function overlay(input){
 renderedCells=T.cells(geometry,view.solution,pressures,$('activationThreshold').value);
 // Reuse the actual model cells. Export base-link metres, before camera/lift transforms.
 if(poseSignature!==signature){const fromWorld=T.inverseRigid(view.solution.frames.left_base_link);taxelPositions=renderedCells.map(cell=>{const [x,y,z]=K.point(fromWorld,cell.position_world_m);return {id:cell.id,x,y,z,pressure:cell.value}});poseSignature=signature}
 else renderedCells.forEach((cell,i)=>{taxelPositions[i].pressure=cell.value});
 positionSignature=overlaySignature;
 const dots=traces[pointIndex],centres=traces[centroidIndex];dots.x=[];dots.y=[];dots.z=[];dots.marker.rgb=[];centres.x=[];centres.y=[];centres.z=[];centres.marker.rgb=[];
 for(const cell of renderedCells){const p=present(cell.position_world_m.map((v,i)=>v+.00035*cell.normal_world[i]));['x','y','z'].forEach((axis,i)=>dots[axis].push(p[i]));dots.marker.rgb.push(rgb(cell.active?'#ff766b':'#8dd3ee'))}
 const cloud=currentCloud();if($('showContactCentres').checked)for(const c of cloud.centroids){const p=present(K.point(view.solution.frames.left_base_link,c.position_m.map((v,i)=>v+.0008*c.normal[i])));['x','y','z'].forEach((axis,i)=>centres[axis].push(p[i]));centres.marker.rgb.push(rgb('#247dff'))}
 $('contactSummary').textContent=cloud.valid?cloud.points.length+' activated points · '+cloud.centroids.length+' contact centres':'Contact export awaits fresh, synchronized angles and pressure';
}
function update(){if(!view)return;
 const desired=live?(!disconnected&&K.freshActual(packet)?packet.angles_act:view.raw):raw;
 const next=JSON.stringify(desired),moved=next!==signature;let meshChanged=[];if(moved){signature=next;meshChanged=pose(desired)}
 lastInput=demo?{...P.demo(performance.now()/1000),angles_act:view.raw.slice(),received_at:Date.now()/1000,field_received_at:{angles_act:Date.now()/1000,tactile_regions:Date.now()/1000}}:packet;
 pressures=P.sample(lastInput,demo||(live&&!disconnected));
 const nextOverlay=JSON.stringify([desired,Object.values(pressures).map(p=>p.values),$('activationThreshold').value,$('showContactCentres').checked,T.synchronized(lastInput,Date.now()/1000),live,disconnected,demo]);
 const overlayChanged=nextOverlay!==overlaySignature;if(overlayChanged){overlaySignature=nextOverlay;overlay(lastInput)}
 if(view.ready&&(moved||overlayChanged)){const indices=[...meshChanged,pointIndex,centroidIndex];HandRenderer.restyle(view.plot,{x:indices.map(i=>traces[i].x),y:indices.map(i=>traces[i].y),z:indices.map(i=>traces[i].z)},indices)}
 const valid=Object.values(pressures).filter(p=>p.peak!==null).length;
 $('pressureStatus').textContent=(demo?'Demo pressure · ':live?'Live pressure · ':'Preview · ')+valid+'/17 fields · '+renderedCells.filter(c=>c.value!==null).length+'/1,062 readings';
 for(const r of P.regions){const p=pressures[r.key],el=$('pressure_'+r.key);el.textContent=r.key.replaceAll('_',' ')+': '+(p.peak===null?'—':p.peak);el.style.borderLeftColor=p.color}
 $('profileStatus0').textContent=demo?'Demo · simulated motion and pressure':!live?'Manual preview':disconnected?'Disconnected · pose held':!K.freshActual(packet)?'Waiting for fresh actual angles · pose held':(packet.simulated?'Simulation':'Live hand')+' · actual angles and pressure';
 $('profileAngles0').textContent=view.solution.warnings.join('\n');
}
window.ProfileTwin={manual(values){if(!live){raw=values.slice();if(!manualQueued){manualQueued=true;requestAnimationFrame(()=>{manualQueued=false;update()})}}},mode(enabled,values){live=enabled;demo=false;disconnected=false;if(!enabled){packet=null;raw=values.slice()}update()},demo(enabled){demo=enabled;update()},receive(msg){packet=K.clone(msg);disconnected=false;if(live)update()},disconnect(){disconnected=true;update()},tick:update,snapshot(){return {live,demo,pressure:K.clone(pressures),taxels:K.clone(renderedCells),models:view?{[id]:{raw:view.raw.slice(),q:K.clone(view.solution.q),frames:K.clone(view.solution.frames)}}:{}}}};
window.getContactCloud=()=>currentCloud();window.getContactPoints=()=>currentCloud().points;
window.getTaxelPositions=()=>taxelPositions.map(point=>({...point}));
// A single in-flight request; latest displayed pose wins. Refresh held poses once/second.
async function shareTaxels(){
 if(publishing||!taxelPositions.length||publishedSignature===positionSignature&&performance.now()-lastPublished<1000)return;
 const sentSignature=positionSignature,root=$('bridgeUrl').value.trim().replace(/\/$/,'');publishing=true;
 try{const response=await fetch(root+'/api/taxels',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(taxelPositions),signal:AbortSignal.timeout(1250)});if(response.ok){publishedSignature=sentSignature;lastPublished=performance.now()}}catch{}finally{publishing=false}
}
setInterval(shareTaxels,200);
$('activationThreshold').oninput=update;$('showContactCentres').onchange=update;
$('exportContactCloud').onclick=()=>{const cloud=currentCloud(),url=URL.createObjectURL(new Blob([JSON.stringify(cloud,null,2)],{type:'application/json'})),a=document.createElement('a');a.href=url;a.download='hand_contact_cloud.json';a.click();setTimeout(()=>URL.revokeObjectURL(url),1000)};
function fit(){const low=[Infinity,Infinity,Infinity],high=[-Infinity,-Infinity,-Infinity];for(const trace of traces){if(trace.visible===false)continue;for(let i=0;i<trace.x.length;i++)for(let k=0;k<3;k++){const value=trace[['x','y','z'][k]][i];low[k]=Math.min(low[k],value);high[k]=Math.max(high[k],value)}}const target=low.map((v,k)=>(v+high[k])/2),distance=Math.max(440,.55*Math.hypot(...high.map((v,k)=>v-low[k]))/Math.sin(Math.PI/7.5));view?.plot._renderer?.focus(target,[.2,-1,.25],distance)}
$('resetView').onclick=fit;
$('panel').insertBefore($('profileControls'),$('panel').children[2]);document.body.classList.add('profile-mode');
for(const id of ['sensorEditor','showSensors','exportContacts','indexAlpha','thumbBend','thumbRot','ratio']){let el=$(id);if(!el)continue;if(id!=='sensorEditor')el=el.closest('details')||el.closest('.metrics');el?.remove()}
for(const p of $('panel').querySelectorAll('p'))if(p.textContent.startsWith('Original STEP geometry'))p.remove();
$('liveField').previousElementSibling.remove();$('liveField').hidden=true;
for(const r of P.regions){const el=document.createElement('div');el.id='pressure_'+r.key;$('pressureRegions').appendChild(el)}
view={plot:$('profileScene0'),raw:raw.slice(),ready:false};pose(raw);update();
const objects=SensorObjects.attach({models:data.sensor_objects,traces,plot:view.plot,toDisplay:p=>present(K.point(view.solution.frames.left_base_link,p))});
window.getSensorObjects=()=>objects.snapshot();
HandRenderer['create']('profileScene0',traces).then(()=>{view.ready=true;objects.connectDrag();fit();$('modeLive').click()}).catch(e=>{$('profileStatus0').textContent='Rendering failed: '+e.message});
setInterval(update,200);
})();
