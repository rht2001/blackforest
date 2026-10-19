const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),{pathToFileURL}=require('node:url');
const {chromium}=require('playwright');
const revision=fs.readFileSync(path.join(__dirname,'../digital_twin/sensor_identification/web_service.py'),'utf8').match(/ESTIMATOR_REVISION = '([^']+)'/)[1];
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  let active=false,step=1,done=false,request;
  const pose=n=>({candidate:n===3?'SENSOR_B':'SENSOR_A',stage:n===1?'standard_placement':n===2?'moving_to_thumb':'refining',sequence:n,elapsed_seconds:n,transform:[[1,0,0,10*n],[0,1,0,20],[0,0,1,100],[0,0,0,1]]});
  const state=()=>active?{status:done?'complete':'running',job_id:'live-test',estimator_revision:revision,started_at:Date.now()/1000,input_label:'Preview test',progress:{stage:pose(step).stage,candidate:pose(step).candidate},pose_history:Array.from({length:step},(_,i)=>pose(i+1)),...(done?{result:{classification:'AMBIGUOUS',reason:'quick_placement',fits:{SENSOR_A:{compatible:false,score:1,rmse_mm:5,transform:pose(2).transform}}}}:{})}:{status:'idle',estimator_revision:revision};
  await page.route('http://127.0.0.1:18765/**',route=>route.fulfill({json:{received_at:0,field_received_at:{},angles_act:null,tactile_regions:{}}}));
  await page.route('http://127.0.0.1:18765/api/identification',route=>{
   if(route.request().method()==='POST'){request=route.request().postDataJSON();active=true;}
   return route.fulfill({json:state()});
  });
  await page.goto(pathToFileURL(path.resolve(__dirname,'../digital_twin/hand_twin_6dof.html')).href);
  await page.waitForFunction(()=>document.querySelector('#onePageApp')&&window.HandEstimate,null,{timeout:60000});
  await page.evaluate(()=>{
   HandHistory.telemetry({...JointPressure.demo(0),angles_act:[1000,1000,1000,1000,450,500],index:'1'},'test.csv');
   document.querySelector('#savedSelect').dispatchEvent(new Event('change'));
   const original=HandEstimate.show;window.previewCalls=[];
   HandEstimate.show=function(...args){previewCalls.push({candidate:args[0],transform:args[1].transform,options:args[4]});return original.apply(this,args)};
  });
  await page.click('#estimateButton');
  await page.waitForFunction(()=>window.previewCalls.length>0);
  assert.equal(await page.isVisible('#estimatePreview'),true);
  assert.equal(await page.evaluate(()=>previewCalls.at(-1).options.preserveCamera),true);
  assert.equal(await page.evaluate(()=>previewCalls.at(-1).transform[0][3]),10);
  step=2;
  await page.waitForFunction(()=>window.previewCalls.at(-1)?.transform[0][3]===20);
  await page.evaluate(()=>{const slider=document.querySelector('#estimatePoseStep');slider.value='0';slider.dispatchEvent(new Event('input'))});
  assert.equal(await page.isChecked('#estimateFollow'),false);
  assert.equal(await page.evaluate(()=>previewCalls.at(-1).transform[0][3]),10);
  step=3;
  await page.waitForFunction(()=>document.querySelector('#estimatePoseStep').max==='2');
  assert.equal(await page.evaluate(()=>previewCalls.at(-1).transform[0][3]),10);
  await page.check('#estimateFollow');
  await page.waitForFunction(()=>previewCalls.at(-1).candidate==='SENSOR_B');
  done=true;
  await page.waitForFunction(()=>document.querySelector('#estimateStatus').textContent.includes('No valid placement found.'));
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.some(t=>/^Estimated SENSOR_/.test(t.name)&&t.visible)),false);
  await page.evaluate(()=>{const slider=document.querySelector('#estimatePoseStep');slider.value='1';slider.dispatchEvent(new Event('input'))});
  assert.equal(await page.evaluate(()=>previewCalls.at(-1).transform[0][3]),20);
  assert.ok((await page.textContent('#estimatePreviewStatus')).includes('trial pose'));
  assert.ok(request.frame.hand_snapshot);
  const out=process.env.HAND_VERIFICATION_DIR;if(out){fs.mkdirSync(out,{recursive:true});await page.screenshot({path:path.join(out,'live-stl-preview.png')})}
  await page.click('#dataManual');
  assert.equal(await page.isVisible('#estimatePreview'),false);
  assert.deepEqual(errors,[]);
  console.log('PASS live STL motion, candidate changes, camera preservation, pause, scrubbing, rejected completion and mode cleanup');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
