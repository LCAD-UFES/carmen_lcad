from pathlib import Path
import argparse,os,signal,subprocess,time
parser=argparse.ArgumentParser()
parser.add_argument('fixture',type=Path)
parser.add_argument('--timeout', type=float, default=30.0, help='Processing timeout in seconds')
args=parser.parse_args()
processing_timeout=args.timeout
root=args.fixture;workspace=Path(__file__).resolve().parents[1]
env=dict(os.environ,ROS_DOMAIN_ID='161',ROS_LOG_DIR=str(root/'lifecycle-ros'),OMP_THREAD_LIMIT='1')
for package,launch,param,output in [('ltslam','run','ltslam','save_directory'),('removert','run_ltmapper','removert','save_pcd_directory')]:
 args=['ros2','launch',package,launch+'.launch.py','params_file:='+str(root/(param+'.yaml')),output+':='+str(root/(package+'-lifecycle')),'use_sim_time:=true']
 if package=='removert':args.append('rviz:=false')
 with (root/(package+'-lifecycle.log')).open('w') as stream:
  p=subprocess.Popen(args,env=env,stdout=stream,stderr=subprocess.STDOUT)
 try:
  deadline=time.monotonic()+processing_timeout
  while time.monotonic()<deadline:
   assert p.poll() is None
   if 'Processing completed' in (root/(package+'-lifecycle.log')).read_text():break
   time.sleep(.5)
  else:raise TimeoutError(package)
  time.sleep(.5);assert p.poll() is None
 finally:
  p.send_signal(signal.SIGINT);p.wait(timeout=10)
 assert 'process has finished cleanly' in (root/(package+'-lifecycle.log')).read_text()
 print('PASS default lifecycle and paused-clock shutdown:',package,flush=True)
