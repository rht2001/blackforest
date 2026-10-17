/* Both actual STL objects, independent transforms, persistence, dragging and telemetry isolation. */
const fs=require('node:fs'),os=require('node:os'),path=require('node:path'),assert=require('node:assert/strict'),{pathToFileURL}=require('node:url');
const {chromium}=require(process.env.PLAYWRIGHT_PACKAGE||'playwright');
(async()=>{
 const browser=await chromium.launch({channel:process.env.TEST_BROWSER_CHANNEL||(process.platform==='win32'?'msedge':undefined),headless:true,args:['--use-angle=swiftshader','--enable-unsafe-swiftshader']});
 try{
  const page=await browser.newPage({viewport:{width:1500,height:1000}}),errors=[],writes=[];
  page.on('pageerror',e=>errors.push(e.message));
  await page.route('http://127.0.0.1:18765/**',route=>{
   const request=route.request();if(request.method()==='POST'){writes.push(request.url());return route.fulfill({json:{ok:true}})}
   return route.fulfill({json:{received_at:0,field_received_at:{},angles_act:null,tactile_regions:{}}});
  });
  await page.goto(pathToFileURL(path.resolve(__dirname,'../digital_twin/hand_twin_6dof.html')).href);
  await page.waitForFunction(()=>window.getSensorObjects?.().length===2&&document.querySelector('#profileScene0')._renderer?.screenDelta,null,{timeout:60000});
  await page.click('#modeManual');
  const states=()=>page.evaluate(()=>window.getSensorObjects());
  const mesh=()=>page.evaluate(()=>document.querySelector('#profileScene0').data.filter(t=>t.name==='VEGABAR 29'||t.name==='VEGABAR 38').map(t=>({name:t.name,x:t.x.slice(),y:t.y.slice(),z:t.z.slice(),visible:t.visible,triangles:t.i.length})));
  const initial=await states(),meshes=await mesh(),taxels=await page.evaluate(()=>window.getTaxelPositions());
  assert.deepEqual(meshes.map(m=>m.triangles),[53138,87580]);assert.ok(meshes.every(m=>m.visible));
  await page.fill('#sensorObjectX','-180');await page.waitForFunction(()=>window.getSensorObjects()[0].position_m[0]===-.18);
  await page.waitForFunction(()=>{const t=document.querySelector('#profileScene0').data.find(t=>t.name==='VEGABAR 29');return t.x.length===25052});
  // Wait for the queued GPU update and compare it to the declared base -> display transform.
  await page.evaluate(()=>new Promise(requestAnimationFrame));
  const moved=await mesh();assert.notDeepEqual(moved[0],meshes[0]);assert.deepEqual(moved[1],meshes[1]);
  assert.deepEqual(await page.evaluate(()=>window.getTaxelPositions()),taxels);
  await page.selectOption('#sensorObjectSelect','vegabar_38');
  await page.fill('#sensorObjectRZ','45');await page.fill('#sensorObjectY','30');
  await page.evaluate(()=>new Promise(requestAnimationFrame));
  const rotated=await mesh();assert.notDeepEqual(rotated[1],meshes[1]);assert.deepEqual(rotated[0],moved[0]);
  const geometryCheck=await page.evaluate(()=>{
   const data=JSON.parse(document.getElementById('jointProfileData').textContent),K=JointKinematics,state=window.getSensorObjects()[1],model=data.sensor_objects[1],trace=document.querySelector('#profileScene0').data.find(t=>t.name==='VEGABAR 38');
   const ref=data.reference.primary_profiles.rh56_tactile_toolkit_left,s=K.solve(ref,Array(6).fill(1000)),display=K.rigid([0,0,0],[0,-Math.PI/2,0]);
   const local=K.point(K.rigid(state.position_m,state.rotation_rad),model.vertices.slice(0,3));
   const delta=K.point(display,K.point(s.frames.left_base_link,local));
   // Translation cancels when comparing two transformed mesh vertices.
   const local2=K.point(K.rigid(state.position_m,state.rotation_rad),model.vertices.slice(3,6)),delta2=K.point(display,K.point(s.frames.left_base_link,local2));
   return {actual:[trace.x[1]-trace.x[0],trace.y[1]-trace.y[0],trace.z[1]-trace.z[0]],expected:delta2.map((v,i)=>(v-delta[i])*1000)};
  });
  geometryCheck.actual.forEach((v,i)=>assert.ok(Math.abs(v-geometryCheck.expected[i])<1e-8));
  await page.uncheck('#sensorObjectVisible');await page.evaluate(()=>new Promise(requestAnimationFrame));assert.equal((await mesh())[1].visible,false);
  await page.check('#sensorObjectVisible');await page.evaluate(()=>new Promise(requestAnimationFrame));
  const saved=await states();await page.reload();await page.waitForFunction(()=>document.querySelector('#profileScene0')._renderer?.screenDelta,null,{timeout:60000});assert.deepEqual(await states(),saved);
  await page.selectOption('#sensorObjectSelect','vegabar_38');await page.click('#sensorObjectFocus');
  const beforeDrag=await states(),canvas=page.locator('#profileScene0 canvas'),box=await canvas.boundingBox();
  await page.keyboard.down('Shift');await page.mouse.move(box.x+box.width*.55,box.y+box.height*.55);await page.mouse.down();await page.mouse.move(box.x+box.width*.55+65,box.y+box.height*.55+35,{steps:5});await page.mouse.up();await page.keyboard.up('Shift');
  const dragged=await states();assert.notDeepEqual(dragged[1].position_m,beforeDrag[1].position_m);assert.deepEqual(dragged[0],beforeDrag[0]);
  // Normal dragging rotates only the camera.
  await page.mouse.move(box.x+box.width*.55,box.y+box.height*.55);await page.mouse.down();await page.mouse.move(box.x+box.width*.55+40,box.y+box.height*.55+10,{steps:3});await page.mouse.up();assert.deepEqual(await states(),dragged);
  await page.click('#sensorObjectReset');assert.deepEqual((await states())[1],initial[1]);await page.evaluate(()=>new Promise(requestAnimationFrame));await page.click('#modeManual');
  const stationary=await mesh();await page.fill('#n0','200');await page.locator('#n0').dispatchEvent('change');await page.evaluate(()=>new Promise(requestAnimationFrame));
  assert.deepEqual(await mesh(),stationary);
  await page.click('#resetView');await page.locator('#panel').evaluate(el=>el.scrollTop=0);const out=process.env.HAND_VIEWER_ARTIFACT_DIR||os.tmpdir();fs.mkdirSync(out,{recursive:true});await page.screenshot({path:path.join(out,'hand_sensor_objects.png')});
  assert.deepEqual(errors,[]);assert.ok(writes.every(url=>url.endsWith('/api/taxels')));
  console.log('PASS both source STLs, independent XYZ/rotation, visibility, persistence, shift-drag, reset and unchanged hand/taxel pipeline');
 }finally{await browser.close()}
})().catch(e=>{console.error(e);process.exitCode=1});
