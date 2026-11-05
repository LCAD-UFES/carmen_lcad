#!/usr/bin/env python3
"""Differential file-output validation against an isolated ROS1 build."""
import argparse,copy,json,os,signal,subprocess,time
from pathlib import Path
import numpy as np
import yaml
parser=argparse.ArgumentParser()
parser.add_argument('fixture',type=Path)
parser.add_argument('--reference',type=Path,default=Path('/tmp/ltmapper-ros1-reference/build'))
args=parser.parse_args()
root=args.fixture
workspace=Path(__file__).resolve().parents[1]
reference=args.reference
ros2_distro = os.environ.get('SC_LIO_SAM_ROS_DISTRO') or (os.environ.get('ROS_DISTRO') if os.environ.get('ROS_VERSION') == '2' else '')
if ros2_distro:
 os.environ['SC_LIO_SAM_ROS_DISTRO'] = ros2_distro
env=dict(os.environ,OMP_THREAD_LIMIT='1',OMP_NUM_THREADS='1',ROS_MASTER_URI='http://localhost:11458',ROS_DOMAIN_ID='158',ROS_LOG_DIR=str(root/'ros'))
print('Results:',root,flush=True)

def command(version,args):
 script='source /opt/ros/noetic/setup.bash && '
 if version=='ros2':script='source "$1/setup.bash" && shift && '
 return ['env','-u','PYTHONPATH','-u','LD_LIBRARY_PATH','-u','CMAKE_PREFIX_PATH','-u','ROS_PACKAGE_PATH','-u','ROS_DISTRO','-u','ROS_VERSION','bash','--noprofile','--norc','-c',script+'exec "$@"','ltmapper-test']+([str(workspace)] if version=='ros2' else [])+args

def start(name,version,args):
 out=(root/(name+'.log')).open('w')
 return subprocess.Popen(command(version,args),env=env,stdout=out,stderr=subprocess.STDOUT,start_new_session=True)

def stop(p):
 if p.poll() is None:
  p.send_signal(signal.SIGINT)
  try:p.wait(timeout=15)
  except subprocess.TimeoutExpired:os.killpg(p.pid,signal.SIGKILL);p.wait()

master=start('roscore','ros1',['roscore','-p','11458'])
time.sleep(3)
try:
 for mode,package,exe,launch,key,arg,marker in [
  ('ltslam','ltslam','ltslam_ltslam','run','ltslam','save_directory','query_central_aft_intersession_loops.txt'),
  ('removert','removert','removert_removert','run_ltmapper','removert','save_pcd_directory','scans_nd_strong/000011.pcd'),
  ('reconstruct','removert','removert_reconstruct_feature_map','reconstruct_feature_map','map_reconstruct','output_dir','transformations.pcd')]:
  for version in ('ros1','ros2'):
   config=yaml.safe_load((root/(mode+'.yaml')).read_text())
   output=root/(mode+'_'+version)
   config[key][arg]=str(output)+'/'
   path=root/(mode+'_'+version+'.yaml');path.write_text(yaml.safe_dump(config))
   if version=='ros1':
    subprocess.run(command('ros1',['rosparam','load',str(path)]),env=env,check=True)
    args=[str(reference/exe)]
   else:args=['ros2','launch',package,launch+'.launch.py','params_file:='+str(path),arg+':='+str(output),'keep_alive:=false']
   if version=='ros2' and mode=='removert':args.append('rviz:=false')
   p=start(mode+'_'+version,version,args)
   try:
    deadline=time.monotonic()+180
    while time.monotonic()<deadline:
     if version=='ros2' and p.poll() is not None:
      assert p.returncode==0,(mode,version,p.returncode)
      break
     if version=='ros1' and (output/marker).exists():time.sleep(2);break
     assert p.poll() is None,(mode,version,p.returncode)
     time.sleep(1)
    else:raise TimeoutError((mode,version,str(root)))
   finally:stop(p)
   assert (output/marker).is_file(),(mode,version,marker)
   print('PASS execution',mode,version,flush=True)
  a=root/(mode+'_ros1');b=root/(mode+'_ros2')
  left={p.relative_to(a) for p in a.rglob('*') if p.is_file()};right={p.relative_to(b) for p in b.rglob('*') if p.is_file()}
  assert left==right,(left-right,right-left)
  for file in left:
   if file.suffix=='.txt':np.testing.assert_allclose(np.loadtxt(a/file),np.loadtxt(b/file),rtol=1e-5,atol=1e-5)
   elif file.suffix=='.pcd':
    def load_pcd(path):
     header,body=path.read_bytes().split(b'DATA binary\n',1)
     meta={line.split()[0]:line.split()[1:] for line in header.decode().splitlines() if line and not line.startswith('#')}
     fields=meta['FIELDS'];sizes=map(int,meta['SIZE']);types=meta['TYPE'];counts=map(int,meta['COUNT'])
     dtype=np.dtype([(field, {'F':'f','I':'i','U':'u'}[kind]+str(size), (count,)) for field,size,kind,count in zip(fields,sizes,types,counts)])
     return meta,np.frombuffer(body,dtype=dtype)
    ma,pa=load_pcd(a/file);mb,pb=load_pcd(b/file)
    assert ma==mb,('PCD metadata differs',mode,str(file))
    for field in pa.dtype.names:
     np.testing.assert_allclose(pa[field],pb[field],rtol=1e-6,atol=1e-5,err_msg=str(file)+':'+field)
  print('PASS differential',mode,len(left),'files',flush=True)
finally:stop(master)
