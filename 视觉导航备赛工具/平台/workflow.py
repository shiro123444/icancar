"""Local dataset inspection and explicitly selected Python training job."""
import os
import signal
import subprocess
import threading
import time
import uuid
from collections import deque
from pathlib import Path
from tools.audit_data import audit

class TrainingJob:
    def __init__(self):
        self.lock=threading.Lock();self.process=None;self.log=deque(maxlen=300)
        self.info={'state':'idle','exit_code':None,'script':None,'interpreter':None,'dataset':None,'started_at':None}
    def status(self):
        with self.lock:
            return dict(self.info,logs=list(self.log))
    def start(self,interpreter,script,dataset):
        if not all(isinstance(p,str) and p.strip() for p in (interpreter,script,dataset)):
            raise ValueError('请填写解释器、训练脚本和数据 CSV 的绝对路径')
        paths=[Path(p).expanduser() for p in (interpreter,script,dataset)]
        if not all(p.is_absolute() for p in paths): raise ValueError('请使用绝对路径')
        # Preserve the interpreter's venv path; resolving its symlink can select
        # the system Python and lose the environment's installed packages.
        executable=paths[0].absolute()
        source,data=[p.resolve() for p in paths[1:]]
        if not executable.is_file() or not os.access(str(executable),os.X_OK):raise ValueError('Python 解释器不存在或不可执行')
        if not source.is_file() or source.suffix.lower()!='.py':raise ValueError('训练脚本必须是现有的 .py 文件')
        if not data.is_file() or data.suffix.lower()!='.csv':raise ValueError('数据 CSV 不存在')
        result=audit(data)
        if result['rows_checked']==0:raise ValueError('数据 CSV 没有样本')
        return self._launch([str(executable),'-u',str(source)], source.parent,
            {'script':str(source),'interpreter':str(executable),'dataset':str(data)},
            '训练进程已启动；请核对脚本实际读取的数据路径与模型输出路径。')
    def start_local(self,source,epochs=20):
        root=Path(__file__).resolve().parent
        if source is None or source == '':source=str(root/'data'/'参考训练集'/'vendor-2025-heldout')
        if not isinstance(source,str) or not Path(source).expanduser().is_absolute():
            raise ValueError('请输入数据目录的绝对路径')
        data=Path(source).expanduser().resolve()
        if not data.is_dir() or not ((data/'data.csv').is_file() or
                ((data/'train.csv').is_file() and (data/'validation.csv').is_file())):
            raise ValueError('数据目录中没有受支持的 CSV')
        if isinstance(epochs,bool) or not isinstance(epochs,int) or not 1<=epochs<=200:
            raise ValueError('训练轮次应为 1 至 200')
        # Do not resolve this symlink: Python uses its venv location to find packages.
        executable=root/'.venv-training'/'bin'/'python'
        if not executable.is_file() or not os.access(str(executable),os.X_OK):
            raise ValueError('本机训练环境未配置，请参考平台说明')
        output=root/'data'/'训练结果'/('本机-'+time.strftime('%Y%m%d-%H%M%S')+'-'+uuid.uuid4().hex[:8])
        command=[str(executable),'-u','-m','training.run_local','--source',str(data),'--output',str(output),'--epochs',str(epochs)]
        return self._launch(command,root,
            {'script':str(root/'training'/'run_local.py'),'interpreter':str(executable),'dataset':str(data),'output':str(output)},
            '已启动本机训练，数据和模型将保存在独立目录。')
    def _launch(self,command,cwd,details,message):
        with self.lock:
            if self.process is not None and self.process.poll() is None:raise ValueError('已有训练任务正在运行')
            self.log.clear()
            try:
                process=subprocess.Popen(command,cwd=str(cwd),stdin=subprocess.DEVNULL,
                    stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True,encoding='utf-8',errors='replace',bufsize=1,start_new_session=True)
            except OSError as e:raise ValueError('无法启动训练：'+str(e)) from e
            self.process=process
            self.info={'state':'running','exit_code':None,'started_at':time.strftime('%Y-%m-%d %H:%M:%S'),**details}
            self.log.append(message)
            threading.Thread(target=self._collect,args=(process,),daemon=True).start()
        return self.status()
    def _collect(self,process):
        try:
            for line in iter(lambda:process.stdout.readline(4096),''):
                with self.lock:self.log.append(line.rstrip())
            code=process.wait()
            with self.lock:
                if self.process is process:
                    self.info['exit_code']=code
                    self.info['state']='completed' if code==0 else 'failed'
                    self.log.append('进程退出，代码 '+str(code))
        finally:
            if process.stdout:process.stdout.close()
    def stop(self):
        with self.lock:
            proc=self.process
            if proc is None or proc.poll() is not None: return dict(self.info,logs=list(self.log))
            self.info['state']='stopping';self.log.append('已请求停止训练')
            try:os.killpg(proc.pid,signal.SIGTERM)
            except ProcessLookupError:pass
        return self.status()
    def close(self):
        self.stop()
