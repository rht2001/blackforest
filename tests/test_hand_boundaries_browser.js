const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),{pathToFileURL}=require('node:url');
const {chromium}=require(process.env.PLAYWRIGHT_PACKAGE||'playwright');
(async()=>{
 const browser=await chromium.launch({channel:'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('http://127.0.0.1:18765/**',route=>route.fulfill({json:{received_at:0,field_received_at:{},angles_act:null,tactile_regions:{}}}));
  await page.goto(pathToFileURL(path.resolve(__dirname,'../digital_twin/hand_twin_6dof.html')).href);
  await page.waitForFunction(()=>document.querySelector('#onePageApp')&&document.querySelector('#profileScene0')._renderer,null,{timeout:60000});
  await page.click('#dataManual');
  const initial=await page.evaluate(()=>({frame:getIdentificationFrame(),trace:document.querySelector('#profileScene0').data.find(t=>t.name==='Hand boundary limits').x.length}));
  assert.equal(initial.frame.boundary_points.length,379);assert.equal(initial.trace,379);
  assert.ok(initial.frame.points.every(p=>p.pressure===null));
  await page.evaluate(()=>{ProfileTwin.manual(Array(6).fill(0));ProfileTwin.tick()});
  await page.waitForFunction(()=>getHandBoundaryPoints().some(p=>p.link==='left_index_1'&&p.z!==0));
  const closed=await page.evaluate(()=>getHandBoundaryPoints());
  assert.notDeepEqual(closed,initial.frame.boundary_points);
  // Saved CSV reconstruction uses the recorded pose, not the preceding pose.
  await page.evaluate(()=>HandHistory.telemetry({...JointPressure.demo(0),angles_act:Array(6).fill(1000),index:'1'},'test.csv'));
  const history=await page.evaluate(()=>getIdentificationFrame());
  assert.equal(history.measurement_source,'saved_capture');
  assert.deepEqual(history.boundary_points,initial.frame.boundary_points);
  assert.ok(history.points.every(p=>p.normal?.length===3));
  await page.evaluate(()=>{const c=document.querySelector('#showHandBoundaries');c.checked=false;c.onchange()});
  assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Hand boundary limits').visible),false);
  assert.equal((await page.evaluate(()=>getIdentificationFrame())).boundary_points.length,379);
  await page.evaluate(()=>{const c=document.querySelector('#showHandBoundaries');c.checked=true;c.onchange()});
  await page.click('#dataManual');await page.evaluate(()=>ProfileTwin.manual(Array(6).fill(1000)));
  await page.waitForTimeout(300);
  fs.mkdirSync(path.join(__dirname,'../verification'),{recursive:true});
  await page.screenshot({path:path.join(__dirname,'../verification/hand-boundary-points.png')});
  assert.deepEqual(errors,[]);
  console.log('PASS rendered boundary points, manual FK, saved pose/normals, visibility independent from fit evidence; no browser errors');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
