#!/usr/bin/env python3
"""Compare the native ROS1 LiDAR path with migrated ROS2 and the bridge."""
import io
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import xmlrpc.client

workspace=Path(__file__).resolve().parents[1]
carmen=Path(os.environ.get('CARMEN_HOME','/home/eduardopia/carmen_lcad'))
reference=Path(os.environ.get('ROS1_REMAINING_REFERENCE','/tmp/ros1_remaining_reference'))
logs=Path(tempfile.mkdtemp(prefix='ros2-remaining-test-'))
os.environ.update(ROS_MASTER_URI='http://127.0.0.1:11443',ROS_IP='127.0.0.1',
                  ROS_DOMAIN_ID='143',CENTRALHOST='localhost:14843',
                  ROS_HOME=str(logs/'ros'),ROS_LOG_DIR=str(logs/'logs'))
processes=[]
def start(name,args):
    with (logs/(name+'.log')).open('w') as out:
        p=subprocess.Popen(args,cwd=carmen/'bin',stdout=out,stderr=subprocess.STDOUT,start_new_session=True)
    processes.append(p);return p

def stop(p):
    if p.poll() is None:
        os.killpg(p.pid,signal.SIGINT)
        try:p.wait(timeout=12)
        except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

def normalized(m):
    if hasattr(m,'header'):m.header.seq=0
    buf=io.BytesIO();m.serialize(buf);return buf.getvalue()

try:
    start('master',['roscore','-p','11443']);master=xmlrpc.client.ServerProxy(os.environ['ROS_MASTER_URI'])
    for _ in range(100):
        try:master.getPid('/test');break
        except OSError:time.sleep(.1)
    else:raise RuntimeError('master failed')
    start('central',['./central','-p14843']);time.sleep(1)
    start('params',['./param_daemon','../data/iara/parameters/carmen.ini']);time.sleep(2)
    import rospy
    from sensor_msgs.msg import PointCloud2,TimeReference
    from rosgraph_msgs.msg import Clock
    from geometry_msgs.msg import PoseWithCovarianceStamped,TransformStamped
    from tf2_msgs.msg import TFMessage
    rospy.init_node('remaining_test',disable_signals=True)
    rospy.set_param('/use_sim_time',True)
    for key,value in [('robot_collision_height',2.5),('semi_trailer_collision_height',5.),('collision_safety_margin',.25)]:
        rospy.set_param('/ipc_bridge/'+key,value)
    topics={'/points_raw':PointCloud2,'/points_raw_no_vehicle':PointCloud2,
            '/clock':Clock,'/carmen/scan_time_reference':TimeReference}
    received={};subs=[]
    for prefix in ('','/reference'):
        for topic,typ in topics.items():
            key=prefix+topic;received[key]=[]
            subs.append(rospy.Subscriber(key,typ,received[key].append,queue_size=100))
    refs=[]
    for name in ('carmen_to_ros_lidar_raw','ros_to_ros_lidar_collision_model'):
        args=[str(reference/name),'__name:=reference_'+name,'_laser_frame_id:=velodyne',
              '_lidar_sensor_id:=0','_use_exact_lidar_timestamp:=false']
        args += [t+':=/reference'+t for t in topics]
        args += ['/tf_static:=/reference/tf_static','/collision_model_markers:=/reference/markers',
                 '/car_model_marker:=/reference/car','/car_model_trailer_markers:=/reference/trailers']
        refs.append(start(name,args))
    bridge=start('remaining',[str(workspace/'run_remaining_ros1.sh'),
                 'params_file:='+str(carmen/'data/iara/parameters/param.yaml'),'use_sim_time:=true','gps_nrs:=[1]'])
    sensors=start('sensors',[str(workspace/'run_sensors_ros1.sh'),'publish_imu:=false','publish_gps:=false'])
    time.sleep(5)
    assert all(p.poll() is None for p in refs+[bridge,sensors]),str(logs)
    assert start('scans',['/tmp/publish_ipc_lidar']).wait(timeout=10)==0
    time.sleep(3)
    for topic in topics:
        expected=received['/reference'+topic];actual=received[topic]
        assert len(expected)==len(actual)==20,(topic,len(expected),len(actual),str(logs))
        assert list(map(normalized,expected))==list(map(normalized,actual)),(topic,str(logs))
        print('PASS identical ROS1 bytes:',topic,len(actual),flush=True)
    assert received['/points_raw_no_vehicle'][0].width<received['/points_raw'][0].width
    print('PASS collision model removes vehicle points.',flush=True)
    from nav_msgs.msg import Odometry
    gps_actual=[];gps_expected=[];initialposes=[]
    subs.append(rospy.Subscriber('/odometry/gps',Odometry,gps_actual.append,queue_size=50))
    subs.append(rospy.Subscriber('/reference/odometry/gps',Odometry,gps_expected.append,queue_size=50))
    subs.append(rospy.Subscriber('/initialpose',PoseWithCovarianceStamped,initialposes.append,queue_size=50))
    rospy.set_param('/reference_gps/gps_nrs',[1])
    gps_ref=start('gps_reference',[str(reference/'gps_localization_node'),'__name:=reference_gps',
                                  '/odometry/gps:=/reference/odometry/gps','/tf_static:=/reference/gps_tf'])
    receiver=start('fused_messages',['/tmp/receive_fused'])
    fused=start('fused',[str(workspace/'run_ros2_node.sh'),'fused_odometry_from_slam_node',
                        '--ros-args','-p','origin_x:=7750000.0','-p','lidar_frame_id:=base_link'])
    pose_pub=rospy.Publisher('/initialpose',PoseWithCovarianceStamped,queue_size=10)
    odom_pub=rospy.Publisher('/odometry/imu',Odometry,queue_size=50)
    tf_pub=rospy.Publisher('/tf',TFMessage,queue_size=100)
    recorder=start('recorder',[str(workspace/'run_ros2_node.sh'),'generate_poses_opt',str(logs/'poses.dat'),
        '--ros-args','-p','base_frame_id:=test_base','-p','map_base_link_frame_id:=test_map',
        '-p','lidar_topic:=/test/record_cloud','-p','use_sim_time:=true'])
    time.sleep(2)
    fixture=start('fixture',['env','-u','PYTHONPATH','-u','LD_LIBRARY_PATH','-u','ROS_DISTRO',
        '-u','ROS_VERSION','-u','CMAKE_PREFIX_PATH','-u','ROS_PACKAGE_PATH',
        '/bin/bash','--noprofile','--norc','-c',
        'source /opt/ros/lyrical/setup.bash; exec python3 "$1" "$2"','fixture',
        str(workspace/'tests/remaining_ros2_fixture.py'),str(logs/'reverse.json')])
    time.sleep(3)
    p=PoseWithCovarianceStamped();p.header.frame_id='ros1_input';p.pose.pose.orientation.w=1.
    pose_pub.publish(p)
    for i in range(5):
        m=Odometry();m.header.stamp=rospy.Time(3000+i);m.header.frame_id='map'
        m.pose.pose.position.x=10.+i;m.pose.pose.position.y=20.;m.pose.pose.position.z=3.
        m.pose.pose.orientation.w=1.;m.twist.twist.angular.z=.125
        odom_pub.publish(m)
        t=TransformStamped();t.header.stamp=m.header.stamp;t.header.frame_id='test_base'
        t.child_frame_id='test_map';t.transform.rotation.w=1.;t.transform.translation.x=10.+i
        tf_pub.publish(TFMessage(transforms=[t]));time.sleep(.2)
    assert fixture.wait(timeout=15)==0,str(logs)
    time.sleep(1)
    assert len(gps_actual)==len(gps_expected)==5,(len(gps_actual),len(gps_expected),str(logs))
    assert list(map(normalized,gps_actual))==list(map(normalized,gps_expected)),str(logs)
    print('PASS GPS antenna correction/fusion equals ROS1 byte for byte.',flush=True)
    reverse=json.loads((logs/'reverse.json').read_text())
    assert reverse['odom']==[[3000+i,10.+i,.125] for i in range(5)],reverse
    assert reverse['tf'].count('test_map')==5,reverse
    assert reverse['pose'].count('ros1_input')==1 and reverse['pose'].count('ros2_input')==1,reverse
    assert [m.header.frame_id for m in initialposes].count('ros1_input')==1
    assert [m.header.frame_id for m in initialposes].count('ros2_input')==1
    print('PASS ROS1 -> ROS2 odometry/TF and initialpose in both directions without loops.',flush=True)
    lines=[l.split()[1:] for l in (logs/'fused_messages.log').read_text().splitlines() if l.startswith('FUSED ')]
    values=[list(map(float,l)) for l in lines]
    assert values==[[3000.+i,7750010.+i,20.,3.,0.,0.,.125] for i in range(5)],(values,str(logs))
    print('PASS SLAM ROS1 -> ROS2 -> CARMEN fused odometry.',flush=True)
    poses=[list(map(float,line.split())) for line in (logs/'poses.dat').read_text().splitlines()]
    assert len(poses)==5,(poses,str(logs))
    for i,row in enumerate(poses):
        assert row[:6]==[10.+i,0.,0.,0.,0.,0.],row
        assert abs(row[6]-(3000+i+.05))<1e-6 and row[6]==row[7],row
    print('PASS generate_poses_opt records TF pose and CARMEN scan-end timestamp.',flush=True)
    assert all(p.poll() is None for p in [gps_ref,receiver,fused,bridge,sensors]),str(logs)

finally:
    for p in reversed(processes):stop(p)
    print('Logs:',logs,flush=True)
for name in ('remaining','sensors','fused','recorder'):
    text=(logs/(name+'.log')).read_text()
    assert 'Traceback' not in text and '[ERROR]' not in text,str(logs)
print('PASS clean shutdown.')
