#!/usr/bin/env python3
"""Bounded real-log playback of the native stack, isolated from normal processes."""
import os,signal,subprocess,tempfile,time,sys
from pathlib import Path
root=Path(__file__).resolve().parents[1];carmen=Path(os.environ.get('CARMEN_HOME', str(Path.home()/'carmen_lcad')))
vehicle=sys.argv[1];assert vehicle in ('iara','argos')
logs=Path(tempfile.mkdtemp(prefix='native-playback-'+vehicle+'-'))
os.environ.update(CENTRALHOST='localhost:14849',ROS_DOMAIN_ID='149',ROS_LOG_DIR=str(logs/'ros'))
processes=[]
def launch(package, filename, *arguments):
    return ['bash', '--noprofile', '--norc', '-c',
            'source "$1/setup.bash" && shift && exec ros2 launch "$@"',
            'ros2-playback', str(root), package, filename, *arguments]

def start(name,args):
    with (logs/(name+'.log')).open('w') as f:
        p=subprocess.Popen(args,cwd=carmen/'bin',stdout=f,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append((name,p));return p

def stop(p):
    if p.poll() is None:
        p.send_signal(signal.SIGINT)
        try:p.wait(timeout=20)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

try:
    start('central',['./central','-p14849']);time.sleep(1)
    param='../data/iara/parameters/carmen.ini' if vehicle=='iara' else 'argos/carmen-argos.ini'
    yaml=carmen/('data/iara/parameters/param.yaml' if vehicle=='iara' else 'bin/argos/params_argos.yaml')
    sensor='0' if vehicle=='iara' else '5'
    sim_time='use_sim_time:='+('true' if vehicle=='iara' else 'false')
    log='/dados/log_iara-2026-10-02_15-09-00.txt' if vehicle=='iara' else '/dados/log_argos_mapa_ufes-1.txt'
    start('params',['./param_daemon',param]);time.sleep(2)
    start('bridge',launch('carmen_ros_bridge', 'carmen_ros_bridge.launch.py','params_file:='+str(yaml),'lidar_sensor_id:='+sensor,sim_time))
    start('slam',launch('lio_sam', 'run.launch.py','params_file:='+str(yaml),'savePCDDirectory:='+str(logs/'map'),
                  sim_time,'rviz:=false'))
    start('playback',['./playback',log]);time.sleep(8)
    assert all(p.poll() is None for _,p in processes),str(logs)
    subprocess.run(['/tmp/playback_command','t 30:90'],env=os.environ,check=True,stdout=subprocess.DEVNULL)
    for _ in range(14):
        time.sleep(5)
        assert all(p.poll() is None for _,p in processes),str(logs)
    subprocess.run(['/tmp/playback_command','stop'],check=True,stdout=subprocess.DEVNULL)
finally:
    for _,p in reversed(processes):stop(p)
    print('Logs and map:',logs,flush=True)
s=(logs/'slam.log').read_text()
assert s.count('[PUBLISH_GLOBAL]')>20,(s.count('[PUBLISH_GLOBAL]'),str(logs))
assert 'process has died' not in s and 'Segmentation fault' not in s,str(logs)
for filename in ('trajectory.pcd','cloudCorner.pcd','cloudSurf.pcd'):
    assert (logs/'map'/filename).is_file(),(filename,str(logs))
print('PASS real playback',vehicle,'poses',s.count('[PUBLISH_GLOBAL]'),flush=True)
