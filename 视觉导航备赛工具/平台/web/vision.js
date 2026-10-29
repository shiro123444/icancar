// Digital track overlay for the reused racecar console.
(async function(){
 const svg=document.getElementById('visionMap'), ns='http://www.w3.org/2000/svg';
 function el(name,attrs,parent=svg){const e=document.createElementNS(ns,name);for(const [k,v] of Object.entries(attrs))e.setAttribute(k,v);parent.appendChild(e);return e}
 try{
  const t=await (await fetch('/api/track')).json();
  t.boundaries.forEach(p=>el('path',{d:p.d,fill:'none',stroke:'#8c969c','stroke-width':3,'stroke-linejoin':'round'}));
  const i=t.intersection;el('rect',{x:i.x,y:i.y,width:i.width,height:i.height,fill:'var(--bg-card)',stroke:'#8c969c','stroke-width':2,'stroke-dasharray':'6 6'});
  const c=t.crosswalk;for(let n=0;n<c.count;n++)el('rect',{x:c.x+n*(c.stripe_width+c.gap),y:c.y,width:c.stripe_width,height:c.stripe_height,fill:'#7d888d'});
  const l=t.start_line;el('line',{x1:l.x1,y1:l.y1,x2:l.x2,y2:l.y2,stroke:'#3964fe','stroke-width':3});
  t.barriers.forEach(b=>el('line',{x1:b.x1,y1:b.y1,x2:b.x2,y2:b.y2,stroke:'#48a6b8','stroke-width':7,'stroke-linecap':'round'}));
  t.landmarks.forEach(m=>{el('circle',{cx:m.x,cy:m.y,r:10,fill:'#dce8ff'});const text=el('text',{x:m.x,y:m.y+4,'text-anchor':'middle','font-size':11,fill:'#294bc9'});text.textContent=m.id});
  if(t.navigation){
   el('polyline',{points:t.navigation.points.map(p=>p.join(',')).join(' '),fill:'none',stroke:'#517ddd','stroke-width':2,'stroke-dasharray':'5 6',opacity:.45});
   for(let n=15;n<t.navigation.points.length;n+=45){const p=t.navigation.points[n],q=t.navigation.points[(n+1)%t.navigation.points.length],a=Math.atan2(q[1]-p[1],q[0]-p[0])*180/Math.PI;el('path',{d:'M 6 0 L -5 -4 L -3 0 L -5 4 Z',fill:'#517ddd',transform:`translate(${p[0]} ${p[1]}) rotate(${a})`})}
  }
  const trace=el('polyline',{points:'',fill:'none',stroke:'#dd8629','stroke-width':2,opacity:.65});
  function marker(color){const g=el('g',{display:'none'});el('circle',{cx:0,cy:0,r:12,fill:'var(--bg-card)',stroke:color,'stroke-width':3},g);el('path',{d:'M 10 0 L -6 -6 L -3 0 L -6 6 Z',fill:color},g);return g}
  const wheel=marker('#29a6b8'),fused=marker('#dd8629');
  window.addEventListener('robot-localization',e=>{
   const s=e.detail||{},ok=s.fused&&s.odometry;
   if(ok){for(const [g,p] of [[wheel,s.odometry],[fused,s.fused]]){g.setAttribute('display','');g.setAttribute('transform',`translate(${p.x} ${p.y}) rotate(${p.heading_deg})`);g.setAttribute('opacity',s.state==='tracking'?1:.4)}trace.setAttribute('points',(s.path||[]).map(p=>p.join(',')).join(' '));document.getElementById('visionPosition').textContent=`${s.fused.x.toFixed(1)}, ${s.fused.y.toFixed(1)} · ${s.fused.heading_deg.toFixed(0)}°`}
   else{wheel.setAttribute('display','none');fused.setAttribute('display','none');trace.setAttribute('points','');document.getElementById('visionPosition').textContent='位置 —'}
   document.getElementById('visionLocState').textContent=s.state==='replay'?'历史估计':s.state==='tracking'?`地图匹配 ${Math.round((s.confidence||0)*100)}%`:s.state==='last_known'?'上次位置':'待定位';
   const dir={forward:'正向',reverse:'反向',uncertain:'朝向待确认'};
   document.getElementById('visionSection').textContent=s.section_label?`${s.section_label} · ${dir[s.direction]||''}${s.branch_conflict?' · 分支待确认':''}`:'路段 —';
  });
  window.addEventListener('robot-offline',()=>{wheel.setAttribute('opacity',.4);fused.setAttribute('opacity',.4);const label=document.getElementById('visionLocState');if(label.textContent.startsWith('地图匹配'))label.textContent='上次位置'});
  document.getElementById('visionReset').onclick=async()=>{try{const r=await fetch('/api/localization/anchor',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({landmark:1})}),s=await r.json();if(!r.ok)throw Error(s.error||'定位未就绪');window.dispatchEvent(new CustomEvent('robot-localization',{detail:s}))}catch(e){document.getElementById('visionLocState').textContent=e.message}};
  document.getElementById('visionReplay').onclick=async()=>{try{const r=await fetch('/api/localization/last-trial',{cache:'no-store'});if(!r.ok)throw Error();const s=await r.json();window.dispatchEvent(new CustomEvent('robot-localization',{detail:s}))}catch(e){document.getElementById('visionLocState').textContent='回放待就绪'}};
  try{const s=await(await fetch('/api/track-scan')).json(),p=document.getElementById('trackPhotoScan');p.setAttribute('viewBox',`0 0 ${s.image_size[0]} ${s.image_size[1]}`);for(const line of s.yellow_contours)el('polygon',{points:line.map(v=>v.join(',')).join(' '),fill:'none',stroke:'#d6a828','stroke-width':2},p)}catch(e){}
 }catch(e){console.error('赛道加载失败',e)}
})();

// Hardware telemetry remains independent of the simulated map coordinates.
(async function(){
 const status=document.getElementById('robotBridgeStatus'),odom=document.getElementById('robotOdom'),camera=document.getElementById('robotCamera'),img=document.getElementById('robotImage');
 let polling=false,lastSuccess=0;
 function showMission(m){
   const set=(id,value)=>document.getElementById(id).textContent=value;
   const phases={idle:'待准备',armed:'准备中',running:'行驶中',yield:'停车礼让',finished:'两圈完成',fault:'已停止'};
   const mark=x=>x===true?'已识别':x===false?'未识别':'—';
   set('autoPhase',m?(phases[m.phase]||'未知'):'等待连接');
   set('autoLaps',m?`圈数 ${m.laps} / 2`:'圈数 —');
   set('autoTime',m?`用时 ${Number(m.elapsed_seconds).toFixed(2)} 秒`:'用时 —');
   set('autoStops',m?`停车 ${m.crosswalk_stops} 次`:'停车 —');
   set('autoSpeak',m?(m.announcement_ok?'播报指令已处理':'播报待命'):'播报 —');
   set('autoLine',m?`起点线 ${mark(m.line)}`:'起点线 —');
   set('autoCrosswalk',m?`人行道 ${mark(m.crosswalk)}`:'人行道 —');
   set('percLine',m?`起点线 ${mark(m.line)}`:'起点线 —');
   set('percCrosswalk',m?`人行道 ${mark(m.crosswalk)}`:'人行道 —');
   set('percBadge',m?'实时识别':'等待连接');
   set('autoCalibration',m?(m.calibrated?'已标定':'待标定'):'标定 —');
   document.getElementById('autoNotice').style.display=m && m.phase==='yield'?'block':'none';
   document.getElementById('camNotice').style.display=m && m.phase==='yield'?'block':'none';
   document.getElementById('percNotice').style.display=m && m.phase==='yield'?'block':'none';
 }
 function showCalibration(c){
   const label=c?(c.state==='complete'?'采集完成':`${c.board_visible?'棋盘格已识别':'等待棋盘格'} · ${c.accepted} / ${c.target}`):'';
   for(const id of ['camCalibration','percCalibration'])document.getElementById(id).style.display=c?'flex':'none';
   for(const id of ['camCalibrationText','percCalibrationText'])document.getElementById(id).textContent=label;
 }
 async function poll(){if(polling)return;polling=true;try{
   const res=await fetch('/api/robot/telemetry',{cache:'no-store',signal:AbortSignal.timeout(4000)});if(!res.ok)throw Error('offline');const v=await res.json();lastSuccess=Date.now();
   window.dispatchEvent(new CustomEvent('robot-localization',{detail:v.localization||{state:'waiting_for_anchor'}}));
   status.textContent='只读连接';
   odom.textContent=v.pose && v.odom_age_s<3 ? `里程计 ${v.pose.x.toFixed(2)}, ${v.pose.y.toFixed(2)} · ${v.pose.frame_id} · ${v.odom_age_s}s` : '里程计 —';
   camera.textContent=v.camera_age_s!==null && v.camera_age_s<3 ? `相机 ${v.camera_age_s}s` : '相机 —';
   const model=v.model && v.model_age_s!==null && v.model_age_s<5 ? v.model : null;
   const labels={crossing:'人行道',start:'起点线'};
   const modelText=model?(model.valid===false?'模型 · 不可用':model.detections.length?'模型 · '+model.detections.map(x=>`${labels[x.label]||x.label} ${x.score.toFixed(2)}`).join(' · '):'模型 · 未识别'):'模型 —';
   document.getElementById('robotModel').textContent=modelText;
   document.getElementById('camModel').textContent=modelText;
   document.getElementById('percModel').textContent=modelText;
   const steering=v.steering && v.steering.valid===true && v.steering_age_s!==null && v.steering_age_s<3 && Number.isFinite(v.steering.steering) ? v.steering.steering : null;
   const processing=steering!==null && Number.isFinite(v.steering.processing_ms) && v.steering.processing_ms>=0 ? ` · 处理 ${Math.round(v.steering.processing_ms)}ms` : '';
   const frameAge=steering!==null && Number.isFinite(v.steering.image_age_ms) && Number.isFinite(v.steering_age_s) ? ` · 帧龄 ${Math.round(Math.max(0,v.steering.image_age_ms)+v.steering_age_s*1000)}ms` : '';
   const replaced=steering!==null && Number.isInteger(v.steering.frames_replaced) ? ` · 覆盖 ${v.steering.frames_replaced} 帧` : '';
   const steeringText=steering===null?'循迹预览 —':`循迹预览 ${steering>=0?'+':''}${steering.toFixed(3)}${processing}${frameAge}${replaced}`;
   for(const id of ['robotSteering','camSteering','percSteering'])document.getElementById(id).textContent=steeringText;
   const control=v.control && v.control_age_s!==null && v.control_age_s<3 ? v.control : null;
   const barrier=v.barrier && v.barrier_age_s!==null && v.barrier_age_s<3 ? v.barrier : null;
   document.getElementById('robotBarrier').textContent=barrier?(barrier.calibrated===false?`挡板 ${barrier.boxes.length} · 待标定`:barrier.valid?`挡板 ${barrier.boxes.length} 个`:'挡板 · 等待识别'):'挡板 —';
   const avoidance=v.avoidance && v.avoidance_age_s!==null && v.avoidance_age_s<3 ? v.avoidance : null;
   const avoidanceLabels={uncalibrated:'待标定',input_unavailable:'等待数据',lane_stop:'停车',lane_geometry_invalid:'车道待确认',confirming_clear:'确认通道',follow_lane:'循迹',barrier_too_close:'距离过近',no_safe_gap:'停车',avoid_barrier:'绕行'};
   document.getElementById('robotAvoidance').textContent=avoidance?`避障 · ${avoidanceLabels[avoidance.reason]||'等待'}`:'避障 —';
   const controlLabels={waiting:'等待',disabled:'未启用',ready:'就绪',estop_latched_or_stale:'急停待复位',odometry_stale:'里程计超时',odometry_limit:'里程计超限',stop_not_confirmed:'停车未确认',safety_mission_or_command_stop:'停车',supervised_follow:'循迹',crosswalk:'人行道前停车礼让行人',one_lap_reference_return:'一圈完成',trial_time_limit:'试跑结束',operator_stop:'已停止',wheel_feedback_limit:'轮速异常',camera_view_obscured:'摄像头被遮挡',camera_model_or_odometry_stale:'画面或反馈中断'};
   const controlElement=document.getElementById('robotControl');
   const safetyAge=control && Number.isFinite(control.safety_age_s) ? ` · 安全信号 ${Math.round((control.safety_age_s+v.control_age_s)*1000)}ms` : '';
   if(controlElement)controlElement.textContent=control?`控制 · ${controlLabels[control.reason]||'等待'}${safetyAge}`:'控制 —';
   showCalibration(v.calibration && v.calibration_age_s!==null && v.calibration_age_s<3 ? v.calibration : null);
   showMission(v.mission && v.mission_age_s!==null && v.mission_age_s<3 ? v.mission : null);
   const cameraLive=v.camera_age_s!==null && v.camera_age_s<3;
   document.getElementById('calCameraState').textContent=cameraLive?'相机实时':'等待相机连接';
   if(cameraLive){
    const url='/api/robot/camera?t='+Date.now();img.style.display='block';img.src=url;
    const camPanel=document.getElementById('p-cam'), live=document.getElementById('camLive'), camImg=document.getElementById('cam');
    if(camPanel.classList.contains('on') && live.style.display!=='none')camImg.src=url;
    if(camPanel.classList.contains('on') && document.getElementById('camPerc').style.display!=='none')document.getElementById('percCam').src=url;
    if(camPanel.classList.contains('on') && document.getElementById('camCal').style.display!=='none')document.getElementById('calLive').src=url;
   }else{img.style.display='none';document.getElementById('calLive').removeAttribute('src')}
 }catch(e){
   if(Date.now()-lastSuccess<5000){status.textContent='连接中';return}
   status.textContent='等待连接';odom.textContent='里程计 —';camera.textContent='相机 —';img.style.display='none';showMission(null);
   window.dispatchEvent(new CustomEvent('robot-offline'));
   document.getElementById('calCameraState').textContent='等待相机连接';document.getElementById('calLive').removeAttribute('src');
   document.getElementById('robotModel').textContent='模型 —';document.getElementById('camModel').textContent='模型 —';document.getElementById('percModel').textContent='模型 —'
   for(const id of ['robotSteering','camSteering','percSteering'])document.getElementById(id).textContent='循迹预览 —';
   const controlElement=document.getElementById('robotControl');if(controlElement)controlElement.textContent='控制 —';
   document.getElementById('robotBarrier').textContent='挡板 —';document.getElementById('robotAvoidance').textContent='避障 —';
   showCalibration(null)
 }finally{polling=false}}
 poll();setInterval(poll,1500);
})();

(function(){
 const button=document.getElementById('autoDemo'),out=document.getElementById('autoDemoResult');
 button.onclick=async()=>{try{
  const response=await fetch('/api/automation/demo',{cache:'no-store'});if(!response.ok)throw Error('演练未启动');
  const data=await response.json(), names={armed:'任务准备',start:'开始计时',lap:'完成圈数',announce:'人行道播报',announcement_complete:'播报完成',yield_complete:'停车完成',finish:'两圈完成'};
  out.textContent=data.timeline.map(e=>`${e.at.toFixed(1)} 秒 · ${names[e.type]||e.type}${e.lap?' '+e.lap+' / 2':''}${e.stationary_seconds?' · '+e.stationary_seconds.toFixed(1)+' 秒':''}`).join('\n');
 }catch(e){out.textContent=e.message}};
})();
