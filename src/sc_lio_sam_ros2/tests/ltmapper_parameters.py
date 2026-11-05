from pathlib import Path
import argparse,subprocess,os,yaml
parser=argparse.ArgumentParser()
parser.add_argument('fixture',type=Path)
args=parser.parse_args()
root=args.fixture;workspace=Path(__file__).resolve().parents[1]
cases=[]
config=yaml.safe_load((root/'removert.yaml').read_text());config['removert']['ExtrinsicLiDARtoPoseBase']=[1.0,0.0]
cases.append(('removert','removert_removert',config,'requires 16 finite'))
config=yaml.safe_load((root/'ltslam.yaml').read_text());config['ltslam']['save_directory']='/'
cases.append(('ltslam','ltslam_ltslam',config,'cannot be the filesystem root'))
config=yaml.safe_load((root/'ltslam.yaml').read_text());config['ltslam']['save_directory']=str(root/'sessions')
cases.append(('ltslam','ltslam_ltslam',config,'must not contain an input'))
for index,(package,exe,config,expected) in enumerate(cases):
 flat={section+'.'+key:value for section,data in config.items() for key,value in data.items()}
 path=root/f'invalid-{index}.yaml';path.write_text(yaml.safe_dump({'/**':{'ros__parameters':flat}}))
 p=subprocess.run([str(workspace/'install'/package/'lib'/package/exe),'--ros-args','--params-file',str(path)],env=dict(os.environ,ROS_DOMAIN_ID='160',ROS_LOG_DIR=str(root/'invalid-ros')),text=True,stdout=subprocess.PIPE,stderr=subprocess.STDOUT,timeout=15)
 assert p.returncode==1 and expected in p.stdout,(p.returncode,p.stdout)
 print('PASS invalid config:',expected)
assert len(list((root/'sessions/central/Scans').glob('*.pcd')))==12
print('PASS inputs preserved')
