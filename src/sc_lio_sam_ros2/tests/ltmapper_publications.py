import argparse,json,os,signal,subprocess,time
from pathlib import Path
parser=argparse.ArgumentParser()
parser.add_argument('fixture',type=Path)
parser.add_argument('--timeout', type=float, default=30.0, help='Processing timeout in seconds')
parser.add_argument('--no-rviz', action='store_true', help='Check publications without opening RViz')
args=parser.parse_args()
root=args.fixture;ws=Path(__file__).resolve().parents[1]
env=dict(os.environ,ROS_DOMAIN_ID='159',ROS_LOG_DIR=str(root/'ros-native'),OMP_THREAD_LIMIT='1')
def cmd(args):return ['env','-u','PYTHONPATH','-u','LD_LIBRARY_PATH','-u','CMAKE_PREFIX_PATH','bash','--noprofile','--norc','-c','source "$1/setup.bash" && shift && exec "$@"','native-check',str(ws)]+args
observer=subprocess.Popen(cmd(['python3',str(ws/'tests/ltmapper_observer.py'),str(root/'ros2-publications.json')]),env=env,stdout=subprocess.PIPE,text=True)
assert observer.stdout.readline().strip()=='READY'
with (root/'native-rviz.log').open('w') as output:
 process=subprocess.Popen(cmd(['ros2','launch','removert','run_ltmapper.launch.py','params_file:='+str(root/'removert.yaml'),'save_pcd_directory:='+str(root/'native-rviz-map'),'keep_alive:=true','rviz:='+str(not args.no_rviz).lower(),'use_sim_time:=true']),env=env,stdout=output,stderr=subprocess.STDOUT)
try:
 observer.wait(timeout=25);assert observer.returncode==0
 data=json.loads((root/'ros2-publications.json').read_text())
 for topic in ('/scan_rimg_single','/map_rimg_single','/diff_rimg_single','/map_rimg_ptidx_single'):
  assert data[topic]['messages']>0 and data[topic]['encoding']=='bgr8' and data[topic]['bytes']>0,(topic,data)
 assert data['/removert/scan_single_local']['frame']=='removert'
 assert data['/removert/scan_single_local']['fields']==['x','y','z','intensity']
 assert process.poll() is None
 deadline=time.monotonic()+args.timeout
 while 'Processing completed' not in (root/'native-rviz.log').read_text():
  assert process.poll() is None
  if time.monotonic() >= deadline:raise TimeoutError('removert processing')
  time.sleep(.5)
 log=(root/'native-rviz.log').read_text()
 if not args.no_rviz:
  assert 'OpenGl version' in log and 'PluginlibFactory' not in log and 'could not be loaded' not in log,log
  print('PASS RViz2 OpenGL/plugins',flush=True)
 print('PASS ROS2 cloud/image publication and keep_alive with paused simulated time',flush=True)
finally:
 process.send_signal(signal.SIGINT)
 try:process.wait(timeout=15)
 except subprocess.TimeoutExpired:process.kill();process.wait()
log=(root/'native-rviz.log').read_text();assert 'removert_removert-1]: process has finished cleanly' in log
print('PASS native Removert clean shutdown',flush=True)
