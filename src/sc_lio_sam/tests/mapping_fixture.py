#!/usr/bin/env python3
"""Native ROS2 mapping inputs and output collection for the differential test."""
import base64,json,sys,time
import rclpy
from rclpy.qos import QoSProfile,DurabilityPolicy
from rosidl_runtime_py.set_message import set_message_fields
from rosidl_runtime_py.convert import message_to_ordereddict
from lio_sam.msg import CloudInfo
from nav_msgs.msg import Odometry,Path
from sensor_msgs.msg import PointCloud2
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Header,Float64MultiArray
from geometry_msgs.msg import PoseWithCovarianceStamped
from visualization_msgs.msg import MarkerArray

rclpy.init()
node=rclpy.create_node('mapping_fixture')
pub=node.create_publisher(CloudInfo,'/lio_sam/feature/cloud_info',20)
clock=node.create_publisher(Clock,'/clock',10)
outputs={name:[] for name in ('odometry','odometry_incremental','path','trajectory','map_global','loc_reset','gps_in_map','loop_closure_constraints')}
types={'odometry':Odometry,'odometry_incremental':Odometry,'path':Path,'trajectory':PointCloud2,'map_global':PointCloud2,'loc_reset':Header,'gps_in_map':Odometry,'loop_closure_constraints':MarkerArray}

def convert(msg):
    result=message_to_ordereddict(msg)
    if isinstance(msg,PointCloud2):result['data']=base64.b64encode(bytes(msg.data)).decode()
    return result

subs=[]
for name,typ in types.items():
    qos=QoSProfile(depth=1,durability=DurabilityPolicy.TRANSIENT_LOCAL) if name=='map_global' else 100
    subs.append(node.create_subscription(typ,'/lio_sam/mapping/'+name,lambda msg,name=name:outputs[name].append(convert(msg)),qos))

def spin(seconds):
    end=time.monotonic()+seconds
    while time.monotonic()<end:rclpy.spin_once(node,timeout_sec=.02)

commands={'pose':(PoseWithCovarianceStamped,'/initialpose'),'gps':(Odometry,'/odometry/gps'),'loop':(Float64MultiArray,'/lio_loop/loop_closure_detection'),'clock':(Clock,'/clock')}
command_pubs={k:node.create_publisher(typ,topic,10) for k,(typ,topic) in commands.items()}
spin(1)
print('READY',flush=True)
try:
    for line in sys.stdin:
        record=json.loads(line)
        if record=='report':
            spin(.3);print(json.dumps(outputs),flush=True);continue
        if isinstance(record,list):
            key,values=record
            if key=='wait':spin(values)
            else:
                msg=commands[key][0]();set_message_fields(msg,values);command_pubs[key].publish(msg);spin(.4)
            print('OK',flush=True);continue
        msg=CloudInfo()
        for field in ('cloud_corner','cloud_surface','cloud_deskewed'):
            record[field]['data']=list(base64.b64decode(record[field]['data']['base64']))
        set_message_fields(msg,record)
        c=Clock();c.clock=msg.header.stamp;clock.publish(c);pub.publish(msg)
        spin(.35)
        print('OK',flush=True)
finally:
    node.destroy_node();rclpy.shutdown()
