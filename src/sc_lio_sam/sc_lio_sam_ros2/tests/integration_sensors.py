#!/usr/bin/env python3
"""Differential ROS1/ROS2 sensor test; source Noetic before running."""
import io
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xmlrpc.client

workspace = Path(__file__).resolve().parents[1]
carmen = Path(os.environ.get('CARMEN_HOME', '/home/eduardopia/carmen_lcad'))
reference = Path(os.environ.get('ROS1_SENSOR_REFERENCE', workspace.parent/'devel/lib/carmen_ros_bridge'))
logs = Path(tempfile.mkdtemp(prefix='ros2-sensors-test-'))
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11442', ROS_IP='127.0.0.1',
                  ROS_DOMAIN_ID='142', CENTRALHOST='localhost:14842',
                  ROS_HOME=str(logs/'ros'), ROS_LOG_DIR=str(logs/'logs'))
processes=[]

def start(name,args):
    with (logs/(name+'.log')).open('w') as out:
        p=subprocess.Popen(args,cwd=carmen/'bin',stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p)
    return p


def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try: p.wait(timeout=10)
        except subprocess.TimeoutExpired:
            os.killpg(p.pid,signal.SIGKILL);p.wait()


def normalized(msg):
    msg.header.seq=0
    buffer=io.BytesIO();msg.serialize(buffer)
    return buffer.getvalue()

try:
    start('master',['roscore','-p','11442'])
    master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    for _ in range(100):
        try: master.getPid('/test');break
        except OSError: time.sleep(.1)
    else: raise RuntimeError('master failed')
    start('central',['./central','-p14842']);time.sleep(1)
    start('params',['./param_daemon','../data/iara/parameters/carmen.ini']);time.sleep(2)
    import rospy
    from sensor_msgs.msg import Imu
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import QuaternionStamped
    from tf2_msgs.msg import TFMessage
    rospy.init_node('sensors_integration',disable_signals=True)
    rospy.set_param('/use_sim_time',True)
    topics={'/imu_raw':Imu,'/gps/heading_raw':QuaternionStamped}
    for nr in (1,2,7):topics['/gps/xyz_raw/gps_'+str(nr)]=Odometry
    received={}; subscribers=[]
    for prefix in ('','/reference'):
        for topic,typ in topics.items():
            key=prefix+topic;received[key]=[]
            subscribers.append(rospy.Subscriber(key,typ,received[key].append,queue_size=2000))
    static=[]
    subscribers.append(rospy.Subscriber('/tf_static',TFMessage,static.append))
    rotation=[0.,-1.,0.,1.,0.,0.,0.,0.,1.]
    refs=[]
    for kind in ('imu_xsens','imu_lidar','gps_xyz','gps_nmea'):
        name='carmen_to_ros_'+kind
        rospy.set_param('/reference_'+kind+'/imu_rotation',rotation)
        args=[str(reference/name),'__name:=reference_'+kind,'_lidar_imu_sensor_id:=5','_warn_on_xsens:=false']
        args += [topic+':=/reference'+topic for topic in topics]
        args += ['/tf_static:=/reference/tf_static']
        refs.append(start('reference_'+kind,args))
    bridge=start('sensors',[str(workspace/'run_sensors_ros1.sh'),'lidar_sensor_id:=5','use_sim_time:=true',
                           'imu_rotation:=[0.0,-1.0,0.0,1.0,0.0,0.0,0.0,0.0,1.0]'])
    time.sleep(5)
    assert all(p.poll() is None for p in refs+[bridge]),str(logs)
    assert start('warmup',['/tmp/publish_ipc_sensors','warmup']).wait(timeout=10)==0
    time.sleep(2)
    assert all(sub.get_num_connections()>0 for sub in subscribers),str(logs)
    assert start('samples',['/tmp/publish_ipc_sensors']).wait(timeout=10)==0
    time.sleep(2)
    for topic in topics:
        actual=[m for m in received[topic] if m.header.stamp.secs>1000000000]
        expected=[m for m in received['/reference'+topic] if m.header.stamp.secs>1000000000]
        assert expected and actual,(topic,len(expected),len(actual),str(logs))
        assert sorted(map(normalized,actual))==sorted(map(normalized,expected)),(topic,len(expected),len(actual),str(logs))
        print('PASS exact ROS1 bytes (except seq):',topic,len(actual),flush=True)
    assert len([m for m in received['/imu_raw'] if m.header.stamp.secs>1000000000])==48
    late=[]
    late_sub=rospy.Subscriber('/tf_static',TFMessage,late.append)
    time.sleep(1)
    assert late and any(t.child_frame_id=='imu_link' and t.header.frame_id=='base_link'
                        and t.transform.rotation.w==1.0 and t.header.stamp.secs==1000
                        for m in late for t in m.transforms),str(logs)
    print('PASS retained static IMU transform for late ROS1 subscriber.',flush=True)
    assert 'IMU do LiDAR ja publicou e Xsens esta publicando' in (logs/'sensors.log').read_text(),str(logs)
    print('PASS simultaneous IMUs and configurable Xsens warning.',flush=True)
    stop(bridge)
    for p in refs:stop(p)
    for key in received:received[key].clear()
    silent=start('silent',[str(workspace/'run_sensors_ros1.sh'),'lidar_sensor_id:=5',
                           'warn_on_xsens:=false'])
    time.sleep(4)
    assert silent.poll() is None,str(logs)
    assert start('silent_samples',['/tmp/publish_ipc_sensors']).wait(timeout=10)==0
    time.sleep(1)
    assert len(received['/imu_raw'])==48,(len(received['/imu_raw']),str(logs))
    stop(silent)
    assert 'IMU do LiDAR ja publicou e Xsens esta publicando' not in (logs/'silent.log').read_text()
    print('PASS warn_on_xsens=false silences warning while both IMUs publish.',flush=True)
    for key in received:received[key].clear()
    disabled=start('disabled',[str(workspace/'run_sensors_ros1.sh'),'lidar_sensor_id:=5',
                               'publish_imu:=false','publish_gps:=false','warn_on_xsens:=false'])
    time.sleep(4)
    assert disabled.poll() is None,str(logs)
    assert start('disabled_samples',['/tmp/publish_ipc_sensors']).wait(timeout=10)==0
    time.sleep(1)
    assert all(not received[t] for t in topics),str(logs)
    stop(disabled)
    print('PASS publish_imu=false and publish_gps=false.',flush=True)
finally:
    for p in reversed(processes):stop(p)
    print('Logs:',logs,flush=True)
for name in ('sensors','silent','disabled'):
    content=(logs/(name+'.log')).read_text()
    assert 'Traceback' not in content and '[ERROR]' not in content,str(logs)
print('PASS clean startup and shutdown.')
