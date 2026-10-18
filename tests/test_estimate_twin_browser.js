const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),{pathToFileURL}=require('node:url');
const {chromium}=require('playwright');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('http://127.0.0.1:18765/**',route=>route.fulfill({json:{received_at:0,field_received_at:{},angles_act:null,tactile_regions:{}}}));
  let reject=false,request,oldBackend=false,posts=0;
  const matrix=[[0,-1,0,22],[1,0,0,-13],[0,0,1,55],[0,0,0,1]];
  await page.route('http://127.0.0.1:18765/api/identification',route=>{
   if(route.request().method()==='GET')return route.fulfill({json:{status:'idle',estimator_revision:oldBackend?null:'sampled_preview_v3'}});
   posts++;request=route.request().postDataJSON();
   return route.fulfill({json:{status:'complete',estimator_revision:'sampled_preview_v3',job_id:'test',input_label:'test capture',report_url:'/identification/report',result:{classification:'AMBIGUOUS',reason:'calibration_unverified',fits:{SENSOR_A:{compatible:!reject,score:.1,rmse_mm:.1,transform:matrix},SENSOR_B:{compatible:!reject,score:.2,rmse_mm:.2,transform:matrix}}}}});
  });
  await page.goto(pathToFileURL(path.resolve(__dirname,'../digital_twin/hand_twin_6dof.html')).href);
  await page.waitForFunction(()=>document.querySelector('#onePageApp')&&window.HandEstimate,null,{timeout:60000});
  await page.evaluate(()=>{
   HandHistory.telemetry({...JointPressure.demo(0),angles_act:[1000,1000,1000,1000,450,500],index:'1'},'test.csv');
   document.querySelector('#savedSelect').dispatchEvent(new Event('change'));
  });
  await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#profileScene0').data.some(t=>t.name==='Estimated SENSOR_A'&&t.visible));
  assert.equal(request.frame.units,'m');assert.ok(request.frame.hand_snapshot.angles_act);
  const pose=await page.evaluate(()=>{
   const plot=document.querySelector('#profileScene0'),traces=plot.data;
   const t=traces.find(t=>t.name==='Estimated SENSOR_A'),axes=['X','Y','Z'].map(a=>traces.find(t=>t.name==='Global '+a));
   return {vertex:[t.x[0],t.y[0],t.z[0]],origin:axes.map(t=>[t.x[0],t.y[0],t.z[0]])[0],
    basis:axes.map(t=>[t.x[1]-t.x[0],t.y[1]-t.y[0],t.z[1]-t.z[0]]),
    local:JSON.parse(document.querySelector('#jointProfileData').textContent).sensor_objects[0].vertices.slice(0,3),
    sourceCenter:JSON.parse(document.querySelector('#jointProfileData').textContent).sensor_objects[0].source_center_mm,
    red:traces.find(t=>t.name==='Frozen estimate measurements').marker.rgb.filter(c=>c[0]===1).length,
    replay:ProfileTwin.snapshot().replay,reportHidden:document.querySelector('#estimateView').hidden};
  });
  const local=pose.local.map((v,k)=>v*1000+pose.sourceCenter[k]),world=matrix.slice(0,3).map(row=>row[3]+row.slice(0,3).reduce((s,v,k)=>s+v*local[k],0));
  const expected=pose.origin.map((v,k)=>v+world.reduce((s,x,j)=>s+x*pose.basis[j][k]/40,0));
  expected.forEach((v,k)=>assert.ok(Math.abs(v-pose.vertex[k])<1e-6,`${k}: ${v} != ${pose.vertex[k]}`));
  assert.ok(pose.red>0);assert.ok(pose.replay);assert.ok(pose.reportHidden);
  await page.selectOption('#estimateCandidate','SENSOR_B');
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Estimated SENSOR_B').visible),true);
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Estimated SENSOR_A').visible),false);
  await page.selectOption('#estimateCandidate','SENSOR_A');
  const verification=process.env.HAND_VERIFICATION_DIR||path.join(__dirname,'../verification');
  fs.mkdirSync(verification,{recursive:true});
  await page.screenshot({path:path.join(verification,'estimate-in-twin.png')});
  reject=true;await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#estimateStatus').textContent.includes('No valid STL pose'));
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.filter(t=>t.name?.startsWith('Estimated ')&&t.visible).length),0);
  assert.equal(await page.isDisabled('#showEstimate'),true);
  await page.click('#dataManual');
  reject=false;
  const accumulated=await page.evaluate(()=>{
   const matrix=Array.from({length:112},()=>Array(17).fill(0));matrix[0][15]=700;matrix[1][15]=500;
   const rows=[1,2].map((index)=>({index:String(index),angles:JSON.stringify([1000,1000,1000,1000,index===1?1000:400,500]),tactile_matrix:JSON.stringify(matrix),state:'thumb_probing',slip_detected:'False'}));
   return IdentificationRuns.build(rows,HandHistory.context(),{voxel_mm:.1});
  });
  await page.setInputFiles('#historyFiles',{name:'run.json',mimeType:'application/json',buffer:Buffer.from(JSON.stringify(accumulated))});
  await page.waitForFunction(()=>document.querySelector('#historyStatus').textContent.includes('Saved run coordinates'));
  await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#profileScene0').data.some(t=>t.name==='Estimated SENSOR_A'&&t.visible));
  assert.equal(request.frame.hand_snapshot.angles_act[4],400);
  assert.ok(request.frame.boundary_points.length>1062);
  assert.equal(request.frame.points.length,4);
  oldBackend=true;const previousPosts=posts;await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#estimateStatus').textContent.includes('Restart the simulation launcher'));
  assert.equal(posts,previousPosts,'An old backend must not run the permissive estimator');
  assert.deepEqual(errors,[]);
  console.log('PASS frozen hand overlay, STL-mm to base-metres transform, red contacts, rejected-pose removal and mode changes; no browser errors');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
