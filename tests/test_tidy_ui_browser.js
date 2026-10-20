const fs=require('node:fs'),assert=require('node:assert/strict');
const path=require('node:path');const root=path.resolve(__dirname,'..');
const {chromium}=require(process.env.PLAYWRIGHT_PACKAGE||'playwright');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  const revision=fs.readFileSync(root+'/digital_twin/sensor_identification/web_service.py','utf8').match(/ESTIMATOR_REVISION = '([^']+)'/)[1];
  let request;
  await page.route('**/api/probing-runs',r=>r.fulfill({json:{runs:[],active:null,auto_on_close:false}}));
  await page.route('**/api/identification',r=>{
   if(r.request().method()==='GET')return r.fulfill({json:{status:'idle',estimator_revision:revision}});
   request=r.request().postDataJSON();return r.fulfill({json:{status:'complete',job_id:'tidy-test',report_url:'/identification/report',estimator_revision:revision,result:{classification:'AMBIGUOUS',reason:'quick_placement',fits:{SENSOR_A:{compatible:true,score:.1,rmse_mm:.1,transform:[[1,0,0,0],[0,1,0,0],[0,0,1,80],[0,0,0,1]]}}}}});
  });
  await page.addInitScript(()=>{window.geometryUploads=0;const upload=WebGL2RenderingContext.prototype.bufferData;WebGL2RenderingContext.prototype.bufferData=function(...args){geometryUploads++;return upload.apply(this,args)}});
  await page.goto(process.env.TEST_RECEIVER_URL,{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>document.querySelector('#historyTime')&&window.HandHistory?.runOverlay,null,{timeout:60000});
  assert.equal(await page.locator('[role=tab]').count(),2);
  assert.equal(await page.locator('#dataLive').getAttribute('aria-selected'),'true');
  await page.uncheck('#followTelemetry');
  assert.equal(await page.locator('#s0').isEnabled(),true);
  const liveDots=await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Taxels').x.length);
  assert.ok(liveDots>1000);
  await page.evaluate(()=>{window.geometryUploads=0});
  await page.uncheck('#handModelVisible');
  assert.equal(await page.evaluate(()=>geometryUploads),0,'Hiding the hand must only redraw, without uploading CAD buffers');
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data[0].visible),false);
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Taxels').visible!==false),true);
  await page.check('#handModelVisible');
  await page.check('#followTelemetry');
  await page.click('#dataHistory');
  const csv=await page.evaluate(()=>{
   const matrix=Array.from({length:112},()=>Array(17).fill(700));
   return 'index;angles;tactile_matrix;state;run_id;slip_detected\n'+[850,850,650,400].map((bend,i)=>[101+i,JSON.stringify([900,900,900,900,bend,600]),JSON.stringify(matrix),'thumb_probing','chosen','False'].join(';')).join('\n');
  });
  await page.setInputFiles('#historyFiles',{name:'chosen.csv',mimeType:'text/csv',buffer:Buffer.from(csv)});
  await page.waitForFunction(()=>!document.querySelector('#estimateButton').disabled&&document.querySelector('#selectedHistory').textContent.includes('chosen.csv'));
  assert.equal(await page.locator('#csvScope').count(),0);
  assert.equal(await page.locator('#historyTimeline').isVisible(),true);
  assert.equal(await page.locator('#historyTime').getAttribute('max'),'3','Repeated six-angle poses must share a slider step');

  const baseline=await page.evaluate(()=>{
   const t=document.querySelector('#profileScene0').data.find(t=>t.name==='Taxels');return {x:t.x.slice(),rgb:t.marker.rgb.slice()};
  });
  await page.locator('#historyTime').fill('1');await page.locator('#historyTime').dispatchEvent('input');
  await page.waitForFunction(()=>ProfileTwin.snapshot().models.rh56_tactile_toolkit_left.raw[4]===850);
  await page.click('#timelineNext');
  await page.waitForFunction(()=>ProfileTwin.snapshot().models.rh56_tactile_toolkit_left.raw[4]===650);
  assert.ok((await page.locator('#historyTimeLabel').textContent()).includes('row 3 / 4'),'The label must retain the original CSV row');
  await page.click('#timelinePrev');
  await page.waitForFunction(()=>ProfileTwin.snapshot().models.rh56_tactile_toolkit_left.raw[4]===850);

  await page.evaluate(()=>{geometryUploads=0;const slider=document.querySelector('#historyTime');for(const value of [2,3,2,1]){slider.value=value;slider.dispatchEvent(new Event('input'))}if(geometryUploads!==0)throw Error('Slider input must defer CAD uploads until the next animation frame')});
  await page.waitForFunction(()=>ProfileTwin.snapshot().models.rh56_tactile_toolkit_left.raw[4]===850);
  const after=await page.evaluate(()=>{const t=document.querySelector('#profileScene0').data.find(t=>t.name==='Taxels');return {x:t.x.slice(),rgb:t.marker.rgb.slice()}});
  assert.deepEqual(after,baseline,'The accumulated contacts and colors stay fixed while scrubbing');
  await page.uncheck('#handModelVisible');
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data[0].visible),false);
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Taxels').visible),true);
  await page.locator('#historyTime').fill('2');await page.locator('#historyTime').dispatchEvent('input');
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data[0].visible),false,'Hidden model stays hidden after row change');
  await page.check('#handModelVisible');
  await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#profileScene0').data.some(t=>t.name==='Estimated SENSOR_A'&&t.visible));
  assert.equal(request.frame.accumulation.thumb_limit_selection,'maximum_bend_per_thumb_rotation');
  assert.equal(request.frame.accumulation.frames_reconstructed,4,'Slider compaction must not remove any row from the run estimator');
  assert.ok(request.frame.points.filter(p=>p.region.startsWith('thumb_')).every(p=>String(p.source_index)==='104'));
  await page.uncheck('#handModelVisible');
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data[0].visible),false);
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Frozen estimate measurements').visible),true);
  await page.check('#handModelVisible');
  await page.click('#dataHistory');
  if(process.env.HAND_VERIFICATION_DIR)await page.screenshot({path:path.join(process.env.HAND_VERIFICATION_DIR,'tidy_ui_history.png')});
  assert.deepEqual(errors,[]);
  console.log('PASS two modes, shared manual/live controls, model visibility in live/history/estimate, stable run colors, thumb peak filtering, and timeline preserving the full estimator input');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
