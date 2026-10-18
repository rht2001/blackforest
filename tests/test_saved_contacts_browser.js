/* Saved capture replay, exact points, offline use, live isolation and file errors. */
const fs=require('node:fs'),path=require('node:path'),assert=require('node:assert/strict'),{pathToFileURL}=require('node:url');
const {chromium}=require(process.env.PLAYWRIGHT_PACKAGE||'playwright');
(async()=>{
 const browser=await chromium.launch({channel:process.env.TEST_BROWSER_CHANNEL||'msedge',headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[],posts=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('http://127.0.0.1:18765/**',route=>{
   if(route.request().method()==='POST'){posts.push(route.request().postDataJSON());return route.fulfill({json:{ok:true}})}
   return route.fulfill({json:{angles_act:null,tactile_regions:{},received_at:0,field_received_at:{}}});
  });
  await page.goto(pathToFileURL(path.resolve(__dirname,'../digital_twin/hand_twin_6dof.html')).href);
  await page.waitForFunction(()=>document.querySelector('#profileScene0')._renderer?.focus&&window.ProfileTwin,null,{timeout:60000});
  await page.click('#animateDemo');
  await page.waitForFunction(()=>window.getContactCloud()?.valid);
  let cloud=await page.evaluate(()=>window.getContactCloud());
  if(process.env.HAND_SAVED_SAMPLE)cloud=JSON.parse(fs.readFileSync(process.env.HAND_SAVED_SAMPLE,'utf8'));
  cloud.received_at=1000; // Historical data must never expire as if it were live.
  const file=(name,data)=>({name,mimeType:'application/json',buffer:Buffer.from(JSON.stringify(data))});
  const localBefore=await page.evaluate(()=>JSON.stringify(localStorage));
  await page.setInputFiles('#savedFiles',[file('grasp-one.json',cloud),file('grasp-two.json',{...cloud,points:cloud.points.slice(0,2)})]);
  await page.waitForFunction(()=>window.ProfileTwin.snapshot().replay?.name==='grasp-two.json');
  assert.equal(await page.locator('#savedSelect option').count(),2);
  assert.equal((await page.evaluate(()=>window.getContactPoints())).length,2);
  await page.selectOption('#savedSelect','0');
  let snapshot=await page.evaluate(()=>window.ProfileTwin.snapshot());
  assert.equal(snapshot.replay.mode,'hand');assert.deepEqual(snapshot.models.rh56_tactile_toolkit_left.raw,cloud.angles_act);
  for(const [key,value]of Object.entries(cloud.joint_angles_rad))assert.ok(Math.abs(snapshot.models.rh56_tactile_toolkit_left.q[key]-value)<1e-6);
  const shown=()=>page.evaluate(()=>{const traces=document.querySelector('#profileScene0').data;return {meshes:traces.filter(t=>t.type==='mesh3d'&&t.visible!==false).length,points:traces.find(t=>t.name==='Taxels'),others:traces.find(t=>t.name==='Other taxels (pressure not stored)'),axes:traces.filter(t=>/^Global [XYZ]$/.test(t.name)),objects:traces.filter(t=>/^VEGABAR/.test(t.name)&&t.visible!==false).length}});
  let scene=await shown();assert.equal(scene.points.x.length,cloud.points.length);assert.ok(scene.meshes>0);assert.equal(scene.objects,0);assert.equal(scene.others.visible,false);assert.equal(scene.axes.filter(t=>t.visible).length,3);
  assert.deepEqual(await page.evaluate(()=>window.getTaxelPositions()),[]);
  assert.deepEqual(await page.evaluate(()=>window.getContactCloud()),cloud);
  const exact=await page.evaluate(()=>{const cloud=window.getContactCloud(),K=JointKinematics,data=JSON.parse(document.querySelector('#jointProfileData').textContent),profile=data.reference.primary_profiles.rh56_tactile_toolkit_left,assets=data.assets.rh56_tactile_toolkit_left,rotation=K.rigid([0,0,0],[0,-Math.PI/2,0]),open=K.solve(profile,Array(6).fill(1000),K.defaults(profile)),low=[Infinity,Infinity,Infinity],high=[-Infinity,-Infinity,-Infinity];
   for(const a of assets){const m=K.mul(open.frames[a.link],K.rigid(a.origin_xyz_m,a.origin_rpy_rad));for(let i=0;i<a.vertices.length;i+=3){const q=K.point(rotation,K.point(m,a.vertices.slice(i,i+3).map((v,k)=>v*a.scale[k])));q.forEach((v,k)=>{low[k]=Math.min(low[k],v);high[k]=Math.max(high[k],v)})}}
   const center=low.map((v,k)=>(v+high[k])/2),base=window.ProfileTwin.snapshot().models.rh56_tactile_toolkit_left.frames.left_base_link;
   return cloud.points.map(p=>K.point(rotation,K.point(base,p.position_m)).map((v,k)=>(v-center[k])*1000+[0,15,130][k]));
  });
  exact.forEach((p,i)=>p.forEach((v,k)=>assert.ok(Math.abs(v-scene.points[['x','y','z'][k]][i])<1e-8)));
  const handAxes=scene.axes;
  await page.click('#savedCloud');scene=await shown();assert.equal(scene.meshes,0);assert.equal(scene.points.x.length,cloud.points.length);
  assert.equal(scene.others.visible,true);assert.equal(scene.others.x.length,1062-cloud.points.length);assert.deepEqual(scene.axes,handAxes);
  for(const axis of scene.axes){assert.equal(axis.mode,'lines');assert.ok(Math.abs(Math.hypot(...['x','y','z'].map(k=>axis[k][1]-axis[k][0]))-40)<1e-8);assert.deepEqual(['x','y','z'].map(k=>axis[k][0]),['x','y','z'].map(k=>scene.axes[0][k][0]))}
  const rebuilt=await page.evaluate(()=>{const data=JSON.parse(document.querySelector('#jointProfileData').textContent),K=JointKinematics,cloud=window.getContactCloud(),solution=K.solve(data.reference.primary_profiles.rh56_tactile_toolkit_left,cloud.angles_act,cloud.kinematics_calibration),ids=new Set(cloud.points.map(p=>p.id));return TactilePoints.cells(data.tactile_geometry,solution,{},cloud.threshold).filter(c=>!ids.has(c.id))});
  assert.equal(rebuilt.length,scene.others.x.length);assert.ok(rebuilt.every(c=>c.value===null));
  const deltaCheck=await page.evaluate(()=>{const traces=document.querySelector('#profileScene0').data,other=traces.find(t=>t.name==='Other taxels (pressure not stored)'),data=JSON.parse(document.querySelector('#jointProfileData').textContent),K=JointKinematics,cloud=window.getContactCloud(),solution=K.solve(data.reference.primary_profiles.rh56_tactile_toolkit_left,cloud.angles_act,cloud.kinematics_calibration),ids=new Set(cloud.points.map(p=>p.id)),cells=TactilePoints.cells(data.tactile_geometry,solution,{},cloud.threshold).filter(c=>!ids.has(c.id)),rotation=K.rigid([0,0,0],[0,-Math.PI/2,0]),origin=K.point(rotation,K.point(solution.frames.left_base_link,[0,0,0])),axisOrigin=traces.find(t=>t.name==='Global X');return cells.map((c,i)=>({expected:K.point(rotation,c.position_world_m).map((v,k)=>(v-origin[k])*1000),actual:['x','y','z'].map(k=>other[k][i]-axisOrigin[k][0])}))});
  deltaCheck.forEach(p=>p.expected.forEach((v,k)=>assert.ok(Math.abs(v-p.actual[k])<1e-8)));
  await page.uncheck('#showOtherTaxels');assert.equal((await shown()).others.visible,false);await page.check('#showOtherTaxels');
  await page.uncheck('#showSavedAxes');assert.ok((await shown()).axes.every(t=>!t.visible));await page.check('#showSavedAxes');
  await page.uncheck('#showContactCentres');assert.equal(await page.evaluate(()=>document.querySelector('#profileScene0').data.find(t=>t.name==='Contact centres').x.length),0);
  const canvas=page.locator('#profileScene0 canvas'),box=await canvas.boundingBox(),objectBefore=await page.evaluate(()=>window.getSensorObjects());
  await page.keyboard.down('Shift');await page.mouse.move(box.x+box.width*.5,box.y+box.height*.5);await page.mouse.down();await page.mouse.move(box.x+box.width*.5+30,box.y+box.height*.5+20);await page.mouse.up();await page.keyboard.up('Shift');
  assert.deepEqual(await page.evaluate(()=>window.getSensorObjects()),objectBefore);assert.equal((await shown()).objects,0);
  const afterPosts=posts.length;await page.waitForTimeout(1200);assert.equal(posts.length,afterPosts);
  assert.deepEqual(await page.evaluate(()=>window.getContactCloud()),cloud);
  assert.equal(await page.evaluate(()=>JSON.stringify(localStorage)),localBefore);
  await page.locator('#panel').evaluate(el=>el.scrollTop=0);
  if(process.env.HAND_VIEWER_ARTIFACT_DIR){fs.mkdirSync(process.env.HAND_VIEWER_ARTIFACT_DIR,{recursive:true});await page.screenshot({path:path.join(process.env.HAND_VIEWER_ARTIFACT_DIR,'saved_cloud.png')});await page.click('#savedHand');await page.screenshot({path:path.join(process.env.HAND_VIEWER_ARTIFACT_DIR,'saved_hand.png')})}
  await page.setInputFiles('#savedFiles',file('bad.json',{...cloud,points:[{id:'bad',value:20,position_m:[1,2,'bad']}]}));
  assert.match(await page.locator('#savedInfo').textContent(),/Invalid/);assert.equal((await page.evaluate(()=>window.getContactPoints())).length,cloud.points.length);
  await page.setInputFiles('#savedFiles',file('different-geometry.json',{...cloud,geometry_sha256:'other'}));
  assert.equal(await page.locator('#savedHand').isDisabled(),true);assert.equal((await shown()).meshes,0);assert.equal((await shown()).others.visible,false);assert.equal(await page.locator('#showOtherTaxels').isDisabled(),true);
  await page.setInputFiles('#savedFiles',file('empty.json',{...cloud,points:[],centroids:[]}));await page.click('#savedCloud');assert.equal((await shown()).points.x.length,0);assert.equal((await shown()).others.x.length,1062);
  await page.click('#savedReturn');assert.equal((await page.evaluate(()=>window.ProfileTwin.snapshot())).replay,null);assert.equal((await page.evaluate(()=>window.ProfileTwin.snapshot())).live,true);
  assert.ok((await shown()).meshes>0);assert.equal((await shown()).others.visible,false);assert.ok((await shown()).axes.every(t=>!t.visible));await page.waitForFunction(()=>window.getTaxelPositions().length===1062);
  assert.equal(await page.locator('#sensorObjectsPanel').isVisible(),true);
  assert.deepEqual(errors,[]);
  console.log('PASS saved hand pose, all 1062 active/unknown-pressure taxels, independently verified geometry, shared 40 mm XYZ axes, visibility controls, history and live isolation');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
