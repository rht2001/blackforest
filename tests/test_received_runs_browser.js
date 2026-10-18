/* End-to-end passive capture; the Python harness owns an ephemeral HTTP server. */
const assert=require('node:assert/strict'),fs=require('node:fs');
const {chromium}=require(process.env.PLAYWRIGHT_PACKAGE||'playwright');
(async()=>{
 const base=process.env.TEST_RECEIVER_URL;
 assert.ok(base,'Run through test_received_runs_browser.py');
 const browser=await chromium.launch({channel:process.env.TEST_BROWSER_CHANNEL||(process.platform==='win32'?'msedge':undefined),headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1550,height:1050},acceptDownloads:true}),errors=[];
  page.on('pageerror',error=>errors.push(error.message));
  await page.goto(base,{waitUntil:'domcontentloaded'});
  await page.waitForSelector('#startReceivedCapture',{timeout:60000});
  await page.waitForFunction(()=>window.HandHistory?.context()?.geometry,null,{timeout:60000});
  await page.click('#startReceivedCapture');
  await page.waitForFunction(()=>!document.getElementById('finishReceivedCapture').disabled);
  const post=async payload=>{const response=await page.request.post(base+'/api/state',{data:payload});assert.equal(response.status(),200);};
  // No probing states: only readouts from a hand moved by someone else's script.
  for(let index=1;index<=4;index++)await post({source_id:'remote-observer',sample_index:index,
    angles_act:[900,900,900,900,index%2?650:450,index<3?500:650],
    tactile_regions:{thumb_tip:[[800,650,0],[0,0,0],[0,0,0]]}});
  await page.waitForFunction(()=>document.getElementById('receivedRunStatus').textContent.includes('4 received samples'));
  await page.click('#finishReceivedCapture');
  await page.waitForFunction(()=>document.getElementById('receivedRunStatus').textContent.includes('Captured run ready'),null,{timeout:60000});
  assert.equal(await page.locator('#dataHistory').getAttribute('aria-selected'),'true');
  await page.waitForFunction(()=>document.querySelector('#runCloudScene')._renderer,null,{timeout:60000});
  const axes=()=>page.evaluate(()=>document.querySelector('#runCloudScene').data.filter(t=>/^Global [XYZ]$/.test(t.name)));
  const frameAxes=await axes();assert.equal(frameAxes.length,3);
  frameAxes.forEach((trace,axis)=>{
    assert.equal(trace.visible,true);assert.equal(trace.mode,'lines');
    assert.deepEqual(['x','y','z'].map(k=>trace[k][0]),[0,0,0]);
    assert.deepEqual(['x','y','z'].map(k=>trace[k][1]),[0,1,2].map(k=>k===axis?40:0));
  });
  await page.uncheck('#runShowAxes');assert.ok((await axes()).every(t=>!t.visible));
  await page.check('#runShowAxes');assert.ok((await axes()).every(t=>t.visible));
  await page.uncheck('#runShowLimits');assert.ok((await axes()).every(t=>t.visible));
  await page.check('#runShowLimits');
  assert.equal(await page.locator('#estimateButton').isEnabled(),true);
  const downloadPromise=page.waitForEvent('download');await page.click('#downloadRun');
  const download=await downloadPromise,cloud=JSON.parse(fs.readFileSync(await download.path(),'utf8'));
  assert.equal(cloud.accumulation.received_capture.capture_mode,'manual');
  assert.equal(cloud.accumulation.received_capture.samples,4);
  assert.ok(cloud.points.length>=4);assert.equal(cloud.frame,'left_base_link');
  await page.click('#dataLive');
  const csvPromise=page.waitForEvent('download');await page.click('#downloadReceivedCSV');
  const csv=await csvPromise;assert.ok(fs.readFileSync(await csv.path(),'utf8').includes('reported_state'));
  await page.reload({waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>document.getElementById('receivedRunStatus')?.textContent.includes('Captured run ready'),null,{timeout:60000});
  await page.click('#dataLive');
  // A second, state-driven run should open automatically without manual controls.
  await post({source_id:'state-sender',sample_index:1,state:'thumb_probing',angles_act:[900,900,900,900,450,500],
    tactile_regions:{thumb_tip:[[800,650,0],[0,0,0],[0,0,0]]}});
  await post({source_id:'state-sender',sample_index:2,state:'cylinder_confirmed'});
  await page.waitForFunction(()=>document.querySelector('#receivedRunSelect option')?.textContent.includes('1 samples'),null,{timeout:30000});
  await page.waitForFunction(()=>document.getElementById('receivedRunStatus').textContent.includes('1 received samples'),null,{timeout:60000});
  await page.click('#dataLive');
  const motion=async(index,primary)=>post({source_id:'motion-observer',sample_index:index,
    angles_act:[...primary,450,650],tactile_regions:{thumb_tip:[[800,650,0],[0,0,0],[0,0,0]]}});
  await motion(1,[950,950,950,950]);
  await motion(2,[700,700,900,900]);
  await page.waitForFunction(()=>document.getElementById('closureCaptureStatus').textContent.includes('Recording finger-motion'));
  await motion(3,[950,950,950,950]);await motion(4,[950,950,950,950]);await motion(5,[950,950,950,950]);
  await page.waitForFunction(()=>document.getElementById('receivedRunStatus').textContent.includes('5 received samples'),null,{timeout:60000});
  const latest=await(await page.request.get(base+'/api/probing-runs')).json();
  assert.equal(latest.runs[0].capture_mode,'closure');assert.equal(latest.runs[0].reason,'hand_reopened');
  await page.click('#dataLive');
  await page.uncheck('#captureOnClose');
  await page.waitForFunction(()=>document.getElementById('closureCaptureStatus').textContent.includes('off'));
  await motion(6,[950,950,950,950]);await motion(7,[700,700,700,700]);
  assert.equal((await(await page.request.get(base+'/api/probing-runs')).json()).active,null);
  if(process.env.TEST_RECEIVER_SCREENSHOT)await page.screenshot({path:process.env.TEST_RECEIVER_SCREENSHOT,fullPage:true});
  assert.deepEqual(errors,[]);
  console.log('PASS manual/state/motion capture, automatic reconstruction, CSV/cloud downloads, reload recovery, pre-roll, reopening finish and disabling motion detection.');
 }finally{await browser.close();}
})().catch(error=>{console.error(error);process.exitCode=1});
