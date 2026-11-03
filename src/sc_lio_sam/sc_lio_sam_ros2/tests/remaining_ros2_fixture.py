#!/usr/bin/env python3
import json
import sys
import time
import struct
import rclpy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import QuaternionStamped,PoseWithCovarianceStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import PointCloud2,PointField

rclpy.init()
node=rclpy.create_node('remaining_fixture')
gps=node.create_publisher(Odometry,'/gps/xyz_raw/gps_1',50)
heading=node.create_publisher(QuaternionStamped,'/gps/heading_raw',50)
pose=node.create_publisher(PoseWithCovarianceStamped,'/initialpose',10)
received={'odom':[],'pose':[],'tf':[]}
cloud_pub=node.create_publisher(PointCloud2,'/test/record_cloud',10)
pending=[]
node.create_subscription(Odometry,'/odometry/imu',lambda m:received['odom'].append([m.header.stamp.sec,m.pose.pose.position.x,m.twist.twist.angular.z]),50)
node.create_subscription(PoseWithCovarianceStamped,'/initialpose',lambda m:received['pose'].append(m.header.frame_id),50)
def on_tf(msg):
    for t in msg.transforms:
        received['tf'].append(t.child_frame_id)
        if t.child_frame_id=='test_map':pending.append((time.monotonic()+.3,t.header.stamp))
node.create_subscription(TFMessage,'/tf',on_tf,100)
start=time.monotonic();step=-1
try:
    while time.monotonic()-start<10:
        rclpy.spin_once(node,timeout_sec=.01)
        for due,stamp in list(pending):
            if time.monotonic()>=due:
                cloud=PointCloud2();cloud.header.stamp=stamp;cloud.height=1;cloud.width=1
                cloud.point_step=4;cloud.row_step=4
                cloud.fields=[PointField(name='time',offset=0,datatype=PointField.FLOAT32,count=1)]
                cloud.data=struct.pack('<f',.05);cloud_pub.publish(cloud)
                pending.remove((due,stamp))
        now=int(time.monotonic()-start)
        if now!=step and 2<=now<=6:
            step=now
            h=QuaternionStamped();h.header.stamp.sec=2000+now;h.quaternion.w=1.
            heading.publish(h)
            time.sleep(.1)
            g=Odometry();g.header.stamp.sec=2000+now;g.header.frame_id='utm';g.child_frame_id='gps_1'
            g.pose.pose.position.x=7750000.+now;g.pose.pose.position.y=-350000.;g.pose.pose.position.z=10.
            g.pose.pose.orientation.w=1.
            g.pose.covariance[0]=g.pose.covariance[7]=.04;g.pose.covariance[14]=.36
            gps.publish(g)
            if now==3:
                p=PoseWithCovarianceStamped();p.header.frame_id='ros2_input';p.pose.pose.orientation.w=1.
                pose.publish(p)
    with open(sys.argv[1],'w') as stream:json.dump(received,stream)
finally:
    node.destroy_node();rclpy.shutdown()
