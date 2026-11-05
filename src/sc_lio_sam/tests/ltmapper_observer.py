import json,sys,time
import rclpy
from sensor_msgs.msg import PointCloud2,Image
rclpy.init();node=rclpy.create_node('ltmapper_observer');seen={};subs=[]
def receive(topic,message):
 record=seen.setdefault(topic,{'messages':0});record['messages']+=1
 record['frame']=message.header.frame_id;record['width']=message.width;record['height']=message.height
 record['bytes']=len(message.data)
 if isinstance(message,Image):record['encoding']=message.encoding
 else:record['fields']=[field.name for field in message.fields]
for topic in ('/scan_rimg_single','/map_rimg_single','/diff_rimg_single','/map_rimg_ptidx_single'):
 subs.append(node.create_subscription(Image,topic,lambda msg,t=topic:receive(t,msg),100))
for topic in ('/removert/scan_single_local','/removert/scan_single_global','/removert/original_map','/removert/curr_map','/removert/static_map','/removert/dynamic_map'):
 subs.append(node.create_subscription(PointCloud2,topic,lambda msg,t=topic:receive(t,msg),100))
print('READY',flush=True);deadline=time.monotonic()+12
while time.monotonic()<deadline:rclpy.spin_once(node,timeout_sec=.1)
with open(sys.argv[1],'w') as stream:json.dump(seen,stream,indent=2)
node.destroy_node();rclpy.shutdown()
