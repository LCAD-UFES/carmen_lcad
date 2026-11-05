#!/usr/bin/env python3
"""Reload a disposable map, test retained publication and paused-clock shutdown."""
import hashlib,json,os,signal,subprocess,sys,tempfile,time
from pathlib import Path
import yaml
workspace=Path(__file__).resolve().parents[1]
map_dir=Path(sys.argv[1]).resolve()
assert Path('/tmp') in map_dir.parents,'Use a disposable map under /tmp'
logs=Path(tempfile.mkdtemp(prefix='mapping-localization-'))
os.environ.update(ROS_DOMAIN_ID='148',ROS_LOG_DIR=str(logs/'ros'))
def hashes():
    return {str(p.relative_to(map_dir)):hashlib.sha256(p.read_bytes()).hexdigest() for p in map_dir.rglob('*') if p.is_file()}
original=hashes()
def ros2(args):
    return ['env','-u','PYTHONPATH','-u','LD_LIBRARY_PATH','-u','CMAKE_PREFIX_PATH','-u','ROS_PACKAGE_PATH',
            'bash','--noprofile','--norc','-c','source "$1/setup.bash" && shift && exec "$@"','test',str(workspace)]+args

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait();raise RuntimeError('shutdown timed out with paused clock')

for name,path in [('iara','data/iara/parameters/param.yaml'),('argos','bin/argos/params_argos.yaml')]:
    config=yaml.safe_load((Path(os.environ.get('CARMEN_HOME', str(Path.home()/'carmen_lcad')))/path).read_text())
    config['lio_sam'].update(posesEnable=False,posesOutputFile='',posesOriginMode='manual',
        lidarFrame='base_link',baselinkFrame='base_link',loopClosureEnableFlag=False)
    params=logs/(name+'.yaml');params.write_text(yaml.safe_dump(config))
    with (logs/(name+'.log')).open('w') as out:
        mapper=subprocess.Popen(ros2(['ros2','launch','lio_sam','map_optimization.launch.py',
            'params_file:='+str(params),'savePCDDirectory:='+str(map_dir),'localizationMode:=true','use_sim_time:=true']),
            stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
    observer=None
    try:
        time.sleep(4)
        assert mapper.poll() is None,str(logs)
        # Subscribe after startup; no clock or sensor input is ever published.
        observer=subprocess.Popen(ros2(['python3',str(workspace/'tests/mapping_fixture.py')]),
            stdin=subprocess.PIPE,stdout=subprocess.PIPE,stderr=(logs/(name+'_observer.log')).open('w'),text=True,start_new_session=True)
        assert observer.stdout.readline().strip()=='READY',str(logs)
        observer.stdin.write('"report"\n');observer.stdin.flush()
        result=json.loads(observer.stdout.readline())
        assert result['map_global'] and result['map_global'][0]['width']>0,(name,str(logs))
        assert result['map_global'][0]['header']['stamp']=={'sec':0,'nanosec':0}
        print('PASS',name,'map reload and late transient-local subscriber without /clock',flush=True)
    finally:
        if observer is not None:stop(observer)
        stop(mapper)
    assert mapper.returncode==0,(name,mapper.returncode,str(logs))
    assert hashes()==original,'Localization modified the saved map'
    print('PASS',name,'shutdown while paused; map files unchanged',flush=True)
print('Logs:',logs)
