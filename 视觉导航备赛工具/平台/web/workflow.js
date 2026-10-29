(function(){
 const $=id=>document.getElementById(id), csv=$('wfCsv'), result=$('wfAuditResult'), state=$('wfState'), log=$('wfLog');
 async function post(path,data){const r=await fetch(path,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(data)});const j=await r.json();if(!r.ok)throw Error(j.error||j.msg||'请求失败');return j}
 $('wfAudit').onclick=async()=>{try{const j=await post('/api/workflow/audit',{csv:csv.value.trim()});result.textContent=`列名：${j.columns.join(' / ')}\n已检查：${j.rows_checked} 行\n图片存在：${j.existing_image_paths}  缺失：${j.missing_image_paths}  空路径：${j.blank_image_paths}${j.truncated?'\n达到检查上限':''}\n${j.notes.join('\n')}`}catch(e){result.textContent=e.message}};
 $('wfTrain').onclick=async()=>{const body={csv:csv.value.trim(),interpreter:$('wfPython').value.trim(),script:$('wfScript').value.trim()};if(!body.csv||!body.interpreter||!body.script){state.textContent='请填写三个路径';return}if(!confirm(`将在本机运行训练脚本：\n${body.interpreter}\n${body.script}\n\n脚本可能写入或覆盖模型文件；CSV 路径仅用于检查，实际读取路径由脚本决定。确定启动？`))return;try{render(await post('/api/workflow/train',body))}catch(e){state.textContent=e.message}};
 $('wfLocalTrain').onclick=async()=>{const epochs=Number($('wfEpochs').value);if(!Number.isInteger(epochs)||epochs<1||epochs>200){state.textContent='请输入 1–200 轮';return}try{render(await post('/api/workflow/local-train',{source:$('wfLocalSource').value.trim(),epochs}))}catch(e){state.textContent=e.message}};
 $('wfStop').onclick=async()=>{try{render(await post('/api/workflow/stop',{}))}catch(e){state.textContent=e.message}};
 function render(j){const labels={idle:'等待训练',running:'训练中',stopping:'正在停止',completed:'训练完成',failed:'训练结束 · 退出码 '+j.exit_code};state.textContent=labels[j.state]||j.state;log.textContent=(j.logs||[]).join('\n')||'训练日志';log.scrollTop=log.scrollHeight;$('wfTrain').disabled=['running','stopping'].includes(j.state);$('wfLocalTrain').disabled=['running','stopping'].includes(j.state);$('wfStop').disabled=!['running','stopping'].includes(j.state)}
 async function poll(){try{const r=await fetch('/api/workflow/status',{cache:'no-store'});if(r.ok)render(await r.json());const rr=await fetch('/api/robot/telemetry',{cache:'no-store'});$('wfRobot').textContent=rr.ok?'小车只读连接':'小车待连接'}catch(e){$('wfRobot').textContent='小车待连接'}}
 poll();setInterval(poll,2000);
})();
(function(){
 const $=id=>document.getElementById(id);
 function quote(s){return "'"+s.replace(/'/g,"'\\''")+"'"}
 $('wfBuildCommand').onclick=()=>{
  const cam=$('wfCamTopic').value.trim(),cmd=$('wfCmdTopic').value.trim(),output=$('wfOutput').value.trim();
  if(!cam||!cmd||!output){$('wfCaptureCommand').textContent='请填写图像话题、控制话题和输出目录。';return}
  $('wfCaptureCommand').textContent=`python3 record_dataset.py --camera-topic ${quote(cam)} --camera-type ${$('wfCamType').value} --command-topic ${quote(cmd)} --command-type ${$('wfCmdType').value} --output ${quote(output)}`;
 };
 $('wfMissionDemo').onclick=async()=>{
  try{const r=await fetch('/api/mission/demo',{cache:'no-store'});if(!r.ok)throw Error('无法运行演练');const data=await r.json();const names={stop:'停车',follow_lane:'车道跟随'};
   $('wfMissionResult').textContent=data.timeline.map(e=>`${e.t.toFixed(2)}s  ${e.event} · ${names[e.action]} · ${e.laps}/2 圈${e.announcement?' · '+e.announcement:''}`).join('\n');
  }catch(e){$('wfMissionResult').textContent=e.message}
 };
})();
