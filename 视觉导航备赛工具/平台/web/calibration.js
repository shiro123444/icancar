// Manual photo collection and offline camera calibration.
(function(){
 const el=id=>document.getElementById(id);
 const select=el('calSession'), message=el('calMessage');
 let current=null,busy=false;
 const query=(id,file)=>`id=${encodeURIComponent(id)}${file?`&file=${encodeURIComponent(file)}`:''}`;
 async function json(url,body){
  const opts=body===undefined?{cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)};
  const response=await fetch(url,opts), data=await response.json();
  if(!response.ok)throw Error(data.error||'操作失败');
  return data;
 }
 function setBusy(value){
  busy=value;
  el('calCreate').disabled=value;
  select.disabled=value;
  const enabled=!!current&&!value;
  el('calCapture').disabled=!enabled;
  el('calImport').disabled=!enabled;
  el('calCompute').disabled=!enabled||current.photos.filter(p=>p.enabled).length<12;
  el('calGallery').querySelectorAll('button').forEach(b=>b.disabled=value);
 }
 function status(text){message.textContent=text;}
 function render(data){
  current=data;
  const photos=data.photos, active=photos.filter(p=>p.enabled), detected=active.filter(p=>p.detected===true);
  const result=data.results[data.results.length-1];
  const skipped=new Map(result&&result.current?result.skipped.map(x=>[x.file,x.reason]):[]);
  if(select.value===data.id && select.selectedOptions[0])select.selectedOptions[0].textContent=`${data.created_at.slice(0,19).replace('T',' ')} · ${photos.length} 张`;
  el('calCols').value=data.board.cols;el('calRows').value=data.board.rows;el('calSquare').value=data.board.square_mm;
  el('calPhotoCount').textContent=`已保存 ${photos.length} 张`;
  el('calSummary').textContent=`已选择 ${active.length} 张 · 已识别 ${detected.length} 张`;
  el('calPath').textContent='保存位置：'+data.path;
  el('calExport').href='/api/calibration/export?'+query(data.id);
  el('calExport').style.display='inline-flex';
  const gallery=el('calGallery');gallery.replaceChildren();
  for(const photo of photos){
   const item=document.createElement('div');
   item.style.cssText='border:1px solid var(--line);border-radius:8px;padding:8px;min-width:0';
   if(!photo.enabled)item.style.opacity='.55';
   const image=document.createElement('img');
   image.src='/api/calibration/photo?'+query(data.id,photo.file);image.alt=photo.file;image.loading='lazy';
   image.style.cssText='width:100%;aspect-ratio:4/3;object-fit:contain;background:#16181c;border-radius:4px';
   const title=document.createElement('div');title.className='mut';title.style.marginTop='6px';
   title.textContent=photo.file+' · '+(photo.detected===true?'已识别':photo.detected===false?'未识别':'待检查');
   if(skipped.has(photo.file)){title.textContent+=' · 本次未使用';title.title=skipped.get(photo.file);}
   const toggle=document.createElement('button');toggle.className='btn ghost sm';toggle.style.marginTop='6px';
   toggle.textContent=photo.enabled?'排除此张':'恢复此张';
   toggle.onclick=async()=>{setBusy(true);try{
    render(await json('/api/calibration/toggle',{id:data.id,file:photo.file,enabled:!photo.enabled}));
    status(photo.enabled?'已排除照片，原图保留':'照片已恢复');
   }catch(e){status(e.message)}finally{setBusy(false)}};
   item.append(image,title,toggle);gallery.append(item);
  }
  const download=el('calDownload');
  if(!result){el('calResult').textContent='暂无标定结果';download.style.display='none';}
  else{
   let line=`${result.current?'标定结果（本机）':'上次结果 · 照片已更改'} · ${result.sample_count} 张 · 误差 ${result.rms_px.toFixed(3)} 像素`;
   if(result.skipped.length)line+=` · ${result.skipped.length} 张未参与计算`;
   if(result.quality_notes.length)line+=' · '+result.quality_notes.join('；');
   el('calResult').textContent=line;
   download.href='/api/calibration/result?'+query(data.id,result.file);download.style.display='inline-flex';
  }
  setBusy(busy);
 }
 function clearSession(){
  current=null;select.value='';el('calPhotoCount').textContent='尚未选择采集组';el('calSummary').textContent='';el('calPath').textContent='';el('calGallery').replaceChildren();el('calResult').textContent='暂无标定结果';el('calExport').style.display='none';el('calDownload').style.display='none';setBusy(false);
 }
 async function loadSessions(preferred){
  const data=await json('/api/calibration/sessions');
  const want=preferred||select.value||localStorage.getItem('visionCalibrationSession')||(data.sessions[0]&&data.sessions[0].id)||'';
  select.replaceChildren();
  const placeholder=document.createElement('option');placeholder.value='';placeholder.textContent='请选择采集组';select.append(placeholder);
  for(const session of data.sessions){
   const option=document.createElement('option');option.value=session.id;
   option.textContent=`${session.created_at.slice(0,19).replace('T',' ')} · ${session.photos.length} 张`;
   select.append(option);
  }
  const chosen=data.sessions.find(s=>s.id===want)||data.sessions[0];
  if(chosen){select.value=chosen.id;localStorage.setItem('visionCalibrationSession',chosen.id);render(chosen);}
  else clearSession();
 }
 window.loadCalibrationSessions=()=>loadSessions().catch(e=>status(e.message));
 select.onchange=async()=>{
  if(!select.value){localStorage.removeItem('visionCalibrationSession');clearSession();return;}
  try{localStorage.setItem('visionCalibrationSession',select.value);render(await json('/api/calibration/session?'+query(select.value)));status('已选择采集组');}catch(e){status(e.message)}
 };
 el('calCreate').onclick=async()=>{setBusy(true);try{
  const data=await json('/api/calibration/create',{cols:Number(el('calCols').value),rows:Number(el('calRows').value),square_mm:Number(el('calSquare').value)});
  await loadSessions(data.id);status('采集组已创建，可以开始拍照');
 }catch(e){status(e.message)}finally{setBusy(false)}};
 el('calCapture').onclick=async()=>{if(!current||busy)return;setBusy(true);status('正在保存当前画面…');try{
  const data=await json('/api/calibration/capture',{id:current.id});render(data);
  const photo=data.photos[data.photos.length-1];status('照片已保存 · '+(photo.detected===true?'棋盘格已识别':photo.detected===false?'未识别完整棋盘格':'待检查'));
 }catch(e){status(e.message)}finally{setBusy(false)}};
 el('calImport').onclick=()=>el('calFiles').click();
 el('calFiles').onchange=async event=>{
  const files=Array.from(event.target.files||[]);event.target.value='';if(!current||!files.length)return;
  setBusy(true);try{
   for(let i=0;i<files.length;i++){
    if(files[i].size>10000000)throw Error('单张照片不能超过 10 MB');
    status(`正在导入 ${i+1} / ${files.length}…`);
    const encoded=await new Promise((resolve,reject)=>{const reader=new FileReader();reader.onload=()=>resolve(String(reader.result).split(',')[1]);reader.onerror=()=>reject(Error('读取照片失败'));reader.readAsDataURL(files[i]);});
    render(await json('/api/calibration/import',{id:current.id,image:encoded}));
   }
   status(`已导入 ${files.length} 张照片`);
  }catch(e){status(e.message)}finally{setBusy(false)}
 };
 el('calCompute').onclick=async()=>{if(!current||busy)return;setBusy(true);status('正在计算标定参数…');try{
  const data=await json('/api/calibration/compute',{id:current.id});render(data);
  status('标定结果已保存');
 }catch(e){status(e.message)}finally{setBusy(false)}};
})();
