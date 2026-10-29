// Manual review records are independent of the vehicle and simulated controls.
(function(){
 const $=id=>document.getElementById(id), penaltyKeys=['barrier','boundary','intervention','crosswalk1','announcement1','crosswalk2','announcement2','other'],countKeys=['barrier','boundary','intervention'];
 let recordId=null, dirty=false, busy=false, staleEdit=false;
 const reviewInputs=['rrVideoTime','rrVideoPark1','rrVideoPark2','rrBroadcast1','rrBroadcast2','rrNotes',...penaltyKeys.map(k=>'rrP_'+k),...countKeys.map(k=>'rrC_'+k)];
 async function get(url){const r=await fetch(url,{cache:'no-store'}),j=await r.json();if(!r.ok)throw Error(j.error||'请求失败');return j}
 async function post(url,data){const r=await fetch(url,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)}),j=await r.json();if(!r.ok)throw Error(j.error||'请求失败');return j}
 function shown(value){return value===null||value===undefined?'':String(value)}
 function fmt(value){if(value===null||value===undefined)return '—';let cs=Math.max(0,Math.round(value*100)),m=Math.floor(cs/6000),s=Math.floor(cs%6000/100),c=cs%100;return `${String(m).padStart(2,'0')}:${String(s).padStart(2,'0')}.${String(c).padStart(2,'0')}`}
 function hydrate(rec){
  $('rrVideoTime').value=shown(rec.video_time_s);$('rrVideoPark1').value=shown((rec.video_parking_s||[])[0]);$('rrVideoPark2').value=shown((rec.video_parking_s||[])[1]);
  $('rrBroadcast1').value=(rec.broadcast||[])[0]||'unknown';$('rrBroadcast2').value=(rec.broadcast||[])[1]||'unknown';
  for(const k of penaltyKeys)$('rrP_'+k).value=shown((rec.penalties||{})[k]);
  for(const k of countKeys)$('rrC_'+k).value=shown((rec.counts||{})[k]);
  $('rrNotes').value=rec.notes||'';dirty=false;$('rrSaveState').textContent='已保存';
 }
 function parkText(n,p,phase,elapsed){
  if(p.start_s===null)return `第 ${n} 圈：未标记`;
  if(p.end_s!==null)return `第 ${n} 圈：${p.duration_s.toFixed(2)} 秒`;
  if(phase==='running'&&elapsed!==null)return `第 ${n} 圈：停车中 ${Math.max(0,elapsed-p.start_s).toFixed(2)} 秒`;
  return `第 ${n} 圈：停车标记未结束`;
 }
 function render(s,force=false){
  const rec=s.record,phase=s.phase;
  staleEdit=!!(rec&&recordId&&rec.id!==recordId&&dirty);
  $('rrPhase').textContent=({idle:'等待开始',running:'计时中',finished:'两圈已标记',interrupted:'记录已中止'})[phase]||phase;
  $('rrClock').textContent=fmt(s.manual_elapsed_s);const laps=rec?rec.lap_times_s.length:0;$('rrLaps').textContent=`${laps} / 2`;
  $('rrLapTimes').textContent=rec?`第 1 圈经过起点 ${fmt(rec.lap_times_s[0])} · 第 2 圈经过起点 ${fmt(rec.lap_times_s[1])}`:'第 1 圈 — · 第 2 圈 —';
  $('rrCurrentLap').textContent=phase==='finished'?'两圈已完成':`第 ${Math.min(laps+1,2)} 圈`;
  $('rrPark1').textContent=rec?parkText(1,rec.parking[0],phase,s.manual_elapsed_s):'第 1 圈：—';
  $('rrPark2').textContent=rec?parkText(2,rec.parking[1],phase,s.manual_elapsed_s):'第 2 圈：—';
  const p=rec&&phase==='running'?rec.parking[laps]:null;
  $('rrStart').disabled=busy||staleEdit||phase==='running';$('rrLap').disabled=busy||staleEdit||phase!=='running';$('rrLap').textContent=laps===0?'第 1 圈经过起点':'第 2 圈经过起点';
  $('rrInterrupt').disabled=busy||staleEdit||phase!=='running';$('rrParkStart').disabled=busy||staleEdit||!p||p.start_s!==null;
  $('rrParkEnd').disabled=busy||staleEdit||!p||p.start_s===null||p.end_s!==null;
  $('rrParkClear').disabled=busy||staleEdit||!p||p.start_s===null;$('rrSave').disabled=busy||staleEdit||!rec;$('rrSelect').disabled=busy||staleEdit||phase==='running';
  $('rrExport').style.pointerEvents=rec?'':'none';$('rrExport').style.opacity=rec?'1':'.5';
  $('rrPenaltyTotal').textContent=s.penalty_total===null?'—':s.penalty_total;$('rrEffective').textContent=s.effective_time===null?'—':s.effective_time;
  $('rrPath').textContent=rec?`本地记录 ${rec.id}${phase==='interrupted'?' · 计时中断，请以视频复核':''}`:'还没有记录';
  if(staleEdit){$('rrSaveState').textContent='记录已在另一页面切换；请复制未保存内容后刷新。'}
  else if(rec&&(recordId!==rec.id||force)){recordId=rec.id;hydrate(rec)}
 }
 async function history(){
  const j=await get('/api/race-record/history'),sel=$('rrHistory'),old=sel.value;
  sel.replaceChildren();if(!j.records.length){const o=document.createElement('option');o.value='';o.textContent='暂无记录';sel.appendChild(o);return}
  for(const x of j.records){const o=document.createElement('option');o.value=x.id;o.textContent=`${x.started_at||'未知时间'} · ${{running:'计时中',finished:'已完成',interrupted:'已中止'}[x.phase]||x.phase}`;sel.appendChild(o)}
  if([...sel.options].some(o=>o.value===old))sel.value=old;else if(recordId)sel.value=recordId;
 }
 async function poll(){if(busy)return;try{const s=await get('/api/race-record/current');if(!busy){const changed=s.record&&recordId!==s.record.id;render(s);if(changed&&!staleEdit)await history()}}catch(e){$('rrPhase').textContent='连接中断'}}
 async function action(name){
  if(busy)return;
  if(name==='start'&&dirty&&!confirm('当前复核内容尚未保存，开始新记录会切换页面内容。继续吗？'))return;
  if(name==='interrupt'&&!confirm('结束当前计时并保留已记录的数据？'))return;
  if(name==='park_clear'&&!confirm('清除本圈的停车标记？'))return;
  busy=true;
  try{const s=await post('/api/race-record/event',{action:name,id:recordId});if(name==='start')dirty=false;render(s);$('rrSaveState').textContent=dirty?'标记已保存；复核未保存':'标记已保存';await history();if(name==='start')$('rrHistory').value=recordId}
  catch(e){$('rrSaveState').textContent=e.message}finally{busy=false;poll()}
 }
 $('rrStart').onclick=()=>action('start');$('rrLap').onclick=()=>action('lap');$('rrParkStart').onclick=()=>action('park_start');$('rrParkEnd').onclick=()=>action('park_end');$('rrParkClear').onclick=()=>action('park_clear');$('rrInterrupt').onclick=()=>action('interrupt');
 for(const id of reviewInputs)$(id).addEventListener('input',()=>{dirty=true;$('rrSaveState').textContent='未保存'});
 $('rrSave').onclick=async()=>{if(busy)return;busy=true;try{
  const penalties=Object.fromEntries(penaltyKeys.map(k=>[k,$('rrP_'+k).value]));
  const counts=Object.fromEntries(countKeys.map(k=>[k,$('rrC_'+k).value]));
  const payload={id:recordId,video_time_s:$('rrVideoTime').value,video_parking_s:[$('rrVideoPark1').value,$('rrVideoPark2').value],broadcast:[$('rrBroadcast1').value,$('rrBroadcast2').value],penalties,counts,notes:$('rrNotes').value};
  render(await post('/api/race-record/review',payload),true);await history();$('rrSaveState').textContent='已保存';
 }catch(e){$('rrSaveState').textContent=e.message}finally{busy=false;poll()}};
 $('rrSelect').onclick=async()=>{const id=$('rrHistory').value;if(!id||busy)return;if(dirty&&!confirm('当前复核内容尚未保存，切换记录会丢失这些修改。继续吗？'))return;busy=true;try{const s=await post('/api/race-record/select',{id,expected_id:recordId});dirty=false;render(s,true);$('rrSaveState').textContent='已切换'}catch(e){$('rrSaveState').textContent=e.message}finally{busy=false;poll()}};
 poll().then(history).catch(()=>{});setInterval(poll,250);
})();
