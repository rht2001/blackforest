const assert=require('node:assert/strict'),fs=require('node:fs');
const {chromium}=require(process.env.PLAYWRIGHT_PACKAGE||'playwright');
(async()=>{
 const base=process.env.TEST_RECEIVER_URL;
 assert.ok(base,'Run through the Python harness');
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1550,height:1050},acceptDownloads:true}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  let incoming=false,captureFetches=0,releaseCapture,startedCapture,request;
  const captureStarted=new Promise(resolve=>startedCapture=resolve);
  await page.route(base+'/api/probing-runs',route=>route.fulfill({json:{runs:incoming?[{id:'incoming',status:'complete',samples:2}]:[],active:null,auto_on_close:false}}));
  let csv;
  await page.route(base+'/api/probing-runs/incoming',async route=>{
   captureFetches++;startedCapture();
   await new Promise(resolve=>releaseCapture=resolve);
   await route.fulfill({body:csv.replaceAll('chosen-run','incoming'),contentType:'text/csv'});
  });
  await page.route(base+'/api/identification',route=>{
   if(route.request().method()==='GET')return route.fulfill({json:{status:'idle',estimator_revision:'thumb_first_v7'}});
   request=route.request().postDataJSON();
   return route.fulfill({json:{status:'complete',job_id:'source-test',estimator_revision:'thumb_first_v7',report_url:'/identification/report',result:{classification:'AMBIGUOUS',reason:'quick_placement',quick_preview:true,contacts:request.frame.points.length,fits:{}}}});
  });
  await page.goto(base,{waitUntil:'domcontentloaded'});
  await page.waitForFunction(()=>window.HandHistory?.context()?.geometry&&document.querySelector('#receivedRuns'),null,{timeout:60000});
  assert.equal(await page.locator('#receivedRuns').isVisible(),true);
  await page.click('#dataHistory');
  assert.equal(await page.locator('#receivedRuns').isVisible(),false);
  csv=await page.evaluate(()=>{
   const matrix=Array.from({length:112},()=>Array(17).fill(700));
   const angles=[900,900,900,900,650,600];
   return 'index;angles;tactile_matrix;state;run_id;slip_detected\n'+[101,102].map(index=>[index,JSON.stringify(angles),JSON.stringify(matrix),'thumb_probing','chosen-run','False'].join(';')).join('\n');
  });
  await page.setInputFiles('#historyFiles',{name:'chosen.csv',mimeType:'text/csv',buffer:Buffer.from(csv)});
  await page.waitForFunction(()=>document.querySelector('#selectedHistory').textContent.includes('chosen.csv')&&!document.querySelector('#estimateButton').disabled);
  const downloaded=page.waitForEvent('download');await page.click('#downloadRun');
  const download=await downloaded,expected=JSON.parse(fs.readFileSync(await download.path(),'utf8'));
  assert.ok(expected.points.length>0);
  incoming=true;
  await page.waitForFunction(()=>document.querySelector('#receivedRunSelect option[value="incoming"]'));
  assert.equal(captureFetches,0,'History must not fetch an automatically completed capture');
  assert.ok((await page.locator('#selectedHistory').textContent()).includes('chosen.csv'));
  await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#estimateStatus').textContent.includes('Estimate complete'));
  assert.deepEqual(request.frame,expected,'The estimate must receive the chosen CSV cloud exactly');
  // A capture fetch started in Live must not replace this file after switching back.
  await page.click('#dataLive');
  assert.equal(await page.locator('#receivedRuns').isVisible(),true);
  await captureStarted;
  await page.click('#dataHistory');
  releaseCapture();
  await page.waitForResponse(r=>r.url()===base+'/api/probing-runs/incoming');
  await page.evaluate(()=>new Promise(resolve=>requestAnimationFrame(()=>requestAnimationFrame(resolve))));
  await page.waitForFunction(()=>!document.querySelector('#estimateButton').disabled);
  await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#estimateStatus').textContent.includes('Estimate complete'));
  assert.deepEqual(request.frame,expected,'An in-flight received capture must not replace the chosen CSV');
  await page.selectOption('#csvScope','row');
  await page.waitForFunction(()=>!document.querySelector('#estimateButton').disabled);
  await page.click('#estimateButton');
  await page.waitForFunction(()=>document.querySelector('#estimateStatus').textContent.includes('Estimate complete'));
  assert.deepEqual(request.frame.hand_snapshot.angles_act,[900,900,900,900,650,600]);
  assert.ok(request.frame.points.some(p=>p.pressure===700));
  assert.ok(!request.frame.saved_name.includes('incoming'));
  await page.setInputFiles('#historyFiles',{name:'invalid.csv',mimeType:'text/csv',buffer:Buffer.from('index;state\n1;thumb_probing')});
  await page.waitForFunction(()=>document.querySelector('#historyStatus').textContent.includes('Choose a valid file'));
  assert.equal(await page.locator('#estimateButton').isDisabled(),true,'Invalid CSV must not fall back to live or previous points');
  assert.equal(await page.locator('#receivedRuns').isVisible(),false);
  assert.deepEqual(errors,[]);
  console.log('PASS History visibility, exact chosen-CSV estimation, receiver polling isolation, in-flight capture cancellation, single-row CSV source and invalid-file protection');
 }finally{await browser.close()}
})().catch(error=>{console.error(error);process.exitCode=1});
