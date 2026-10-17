/* Browser/WebGL integration with fake bridge data. Never connects to a hand. */
const fs=require('node:fs'),path=require('node:path'),os=require('node:os'),assert=require('node:assert/strict'),{pathToFileURL}=require('node:url');
const P=require('../digital_twin/joint_pressure');
const {chromium}=require(process.env.PLAYWRIGHT_PACKAGE||'playwright');
(async()=>{const browser=await chromium.launch({channel:process.env.TEST_BROWSER_CHANNEL||(process.platform==='win32'?'msedge':undefined),headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});try{
 const page=await browser.newPage({viewport:{width:1500,height:1000},acceptDownloads:true}),errors=[],methods=[];
 // Keep telemetry age deterministic; stale/skew cases below set explicit ages.
 // Timers and rendering continue normally even on a slow software GPU.
 const telemetryTime=Date.now()/1000;await page.clock.setFixedTime(new Date(telemetryTime*1000));
 page.on('pageerror',e=>errors.push(e.message));page.on('request',r=>{if(r.url().startsWith('http'))methods.push({method:r.method(),url:r.url()})});
 const taxelUrl='http://127.0.0.1:18765/api/taxels';
 await page.route(taxelUrl,route=>route.fulfill({json:{ok:true},headers:{'Access-Control-Allow-Origin':'*'}}));
 const firstShare=page.waitForRequest(r=>r.url()===taxelUrl&&r.method()==='POST');
 let actualAge=0,pressureAge=0,offline=false,raw=[950,950,950,950,1000,1000],seq=0;
 const regions=Object.fromEntries(P.regions.map((r,i)=>{const matrix=Array.from({length:r.rows},()=>Array(r.cols).fill(0));matrix[Math.floor(r.rows/2)][Math.floor(r.cols/2)]=[100,1000,2500,3600][i%4];return[r.key,matrix]}));
 regions.little_tip=[[100,1000,2500],[0,0,0],[0,0,3500]];
 await page.route('http://127.0.0.1:18765/api/state',route=>{if(offline)return route.abort('connectionrefused');const now=telemetryTime;return route.fulfill({json:{seq:++seq,angles_act:raw,angles_set:Array(6).fill(0),force_act:[10,20,30,40,50,60],tactile_regions:regions,simulated:true,received_at:now,field_received_at:{angles_act:now-actualAge,angles_set:now,force_act:now,tactile_regions:now-pressureAge}}})});
 await page.goto(pathToFileURL(path.resolve(__dirname,'../digital_twin/hand_twin_6dof.html')).href);
 await page.waitForFunction(()=>window.ProfileTwin?.snapshot().models.rh56_tactile_toolkit_left.raw[0]===950,null,{timeout:60000});
 const snap=()=>page.evaluate(()=>window.ProfileTwin.snapshot());
 let state=await snap();assert.equal(state.live,true);assert.equal(state.taxels.length,1062);assert.equal(await page.locator('canvas').count(),1);assert.equal(await page.locator('#calibrationPanel,#candidateLive,#calJoint').count(),0);
 const shared=(await firstShare).postDataJSON();assert.equal(shared.length,1062);assert.deepEqual(Object.keys(shared[0]).sort(),['id','pressure','x','y','z']);
 const positions=await page.evaluate(()=>window.getTaxelPositions());assert.equal(positions.length,1062);
 const expected=await page.evaluate(()=>{const s=window.ProfileTwin.snapshot(),base=TactilePoints.inverseRigid(s.models.rh56_tactile_toolkit_left.frames.left_base_link);return s.taxels.map(c=>JointKinematics.point(base,c.position_world_m))});
 positions.forEach((p,i)=>assert.deepEqual([p.x,p.y,p.z],expected[i]));
 assert.deepEqual(positions.slice(0,3).map(p=>p.pressure),[100,1000,2500]);
 await page.evaluate(()=>{const points=window.getTaxelPositions();points[0].x=999;points.pop()});assert.deepEqual(await page.evaluate(()=>window.getTaxelPositions()),positions);
 assert.deepEqual(state.taxels.filter(c=>c.region==='little_tip').slice(0,3).map(c=>c.value),[100,1000,2500]);
 const markers=await page.evaluate(()=>{const t=document.getElementById('profileScene0').data.find(t=>t.name==='Taxels');return {count:t.x.length,colors:t.marker.rgb.slice(0,3)}});
 assert.equal(markers.count,1062);assert.deepEqual(markers.colors,[[0,170/255,0],[204/255,204/255,0],[1,136/255,0]]);
 let cloud=await page.evaluate(()=>window.getContactCloud());assert.equal(cloud.valid,true);assert.equal(cloud.frame,'left_base_link');assert.equal(cloud.units,'metres');assert.ok(cloud.points.length>0);assert.equal(cloud.valid_cells,1062);
 const centres=await page.evaluate(()=>document.getElementById('profileScene0').data.find(t=>t.name==='Contact centres'));
 assert.equal(centres.x.length,cloud.centroids.length);assert.ok(centres.marker.rgb.every(c=>c[2]===1));
 const out=process.env.HAND_VIEWER_ARTIFACT_DIR||os.tmpdir();fs.mkdirSync(out,{recursive:true});await page.screenshot({path:path.join(out,'hand_taxel_preview.png')});
 const downloaded=page.waitForEvent('download');await page.click('#exportContactCloud');const dl=await downloaded,exported=JSON.parse(fs.readFileSync(await dl.path(),'utf8'));assert.equal(exported.geometry_status,'APPROXIMATE_UNCALIBRATED');assert.equal(exported.points[0].position_m.length,3);assert.ok(exported.angles_act);assert.ok(exported.geometry_sha256);
 await page.fill('#activationThreshold','4095');await page.waitForFunction(()=>window.getContactPoints().length===0);await page.fill('#activationThreshold','200');await page.waitForFunction(()=>window.getContactPoints().length>0);
 const before=(await snap()).taxels.find(c=>c.region==='index_tip').position_world_m;
 raw=[100,200,300,400,500,600];await page.waitForFunction(()=>window.ProfileTwin.snapshot().models.rh56_tactile_toolkit_left.raw[0]===100);assert.notDeepEqual((await snap()).taxels.find(c=>c.region==='index_tip').position_world_m,before);
 const movedPositions=await page.evaluate(()=>window.getTaxelPositions());assert.notDeepEqual(movedPositions,positions);
 const movingShare=await page.waitForRequest(r=>r.url()===taxelUrl&&r.method()==='POST'&&r.postDataJSON()[0].x===movedPositions[0].x);
 assert.deepEqual(movingShare.postDataJSON(),movedPositions);
 // A single cell changes while its region maximum and pose remain unchanged.
 regions.little_tip[0][0]=3200;
 // Capture data and color atomically: a slower software renderer can cross
 // the intentional freshness timeout between separate browser evaluations.
 const stationary=await page.waitForFunction(()=>{const s=window.ProfileTwin.snapshot(),color=document.getElementById('profileScene0').data.find(t=>t.name==='Taxels').marker.rgb[0];return s.taxels[0].value===3200&&color[0]===1&&color[1]===0?{peak:s.pressure.little_tip.peak,color}:null});
 const observed=await stationary.jsonValue();assert.equal(observed.peak,3500);assert.deepEqual(observed.color,[1,0,0]);
 const pressureShare=await page.waitForRequest(r=>r.url()===taxelUrl&&r.method()==='POST'&&r.postDataJSON()[0].pressure===3200);
 const pressurePoint=pressureShare.postDataJSON()[0];assert.equal(pressurePoint.x,movedPositions[0].x);assert.equal(pressurePoint.y,movedPositions[0].y);assert.equal(pressurePoint.z,movedPositions[0].z);
 pressureAge=10;await page.waitForFunction(()=>window.ProfileTwin.snapshot().taxels[0].value===null);assert.equal((await page.evaluate(()=>window.getContactPoints())).length,0);
 assert.equal(await page.evaluate(()=>window.getTaxelPositions()[0].pressure),null);
 pressureAge=0;delete regions.index_tip;await page.waitForFunction(()=>window.ProfileTwin.snapshot().taxels[0].value===3200);state=await snap();assert.equal(state.taxels.filter(c=>c.region==='index_tip'&&c.value===null).length,9);assert.equal((await page.evaluate(()=>window.getContactCloud())).valid_cells,1053);
 actualAge=1;await page.waitForFunction(()=>!window.getContactCloud().valid);assert.equal((await page.evaluate(()=>window.getContactPoints())).length,0);
 actualAge=10;raw=Array(6).fill(0);await page.waitForFunction(()=>document.getElementById('profileStatus0').textContent.includes('pose held'));assert.equal((await snap()).models.rh56_tactile_toolkit_left.raw[0],100);
 offline=true;await page.waitForFunction(()=>document.getElementById('profileStatus0').textContent.includes('Disconnected'));assert.ok((await snap()).taxels.every(c=>c.value===null));
 await page.click('#animateDemo');await page.waitForFunction(()=>window.ProfileTwin.snapshot().demo&&window.getContactCloud().valid);assert.equal((await page.evaluate(()=>window.getContactCloud())).simulated,true);assert.match(await page.locator('#profileStatus0').textContent(),/simulated/);await page.click('#animateDemo');assert.ok((await snap()).taxels.every(c=>c.value===null));
 assert.deepEqual(errors,[]);assert.ok(methods.every(r=>r.method==='GET'||r.method==='POST'&&r.url===taxelUrl));console.log('PASS 1062 taxels, colors, motion, contacts, stale/skew/disconnect/demo and shared cached XYZ; only telemetry GETs and taxel-cache POSTs');
}finally{await browser.close()}})().catch(e=>{console.error(e);process.exitCode=1});
