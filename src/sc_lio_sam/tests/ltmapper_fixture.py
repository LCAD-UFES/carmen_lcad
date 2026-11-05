from pathlib import Path
import argparse,shutil,tempfile,yaml,numpy as np
parser=argparse.ArgumentParser()
parser.add_argument("source_map",type=Path)
args=parser.parse_args()
root=Path(tempfile.mkdtemp(prefix='ltmapper-validation-'));source=args.source_map
count=12
for name in ('central','query'):
 session=root/'sessions'/name
 for sub in ('Scans','SCDs'):
  (session/sub).mkdir(parents=True)
  for i in range(count):shutil.copy2(source/sub/(f'{i:06}'+('.pcd' if sub=='Scans' else '.scd')),session/sub/(f'{i:06}'+('.pcd' if sub=='Scans' else '.scd')))
 graph=[]
 for line in (source/'singlesession_posegraph.g2o').read_text().splitlines():
  fields=line.split()
  if fields[0]=='VERTEX_SE3:QUAT' and int(fields[1])<count:graph.append(line)
  if fields[0]=='EDGE_SE3:QUAT' and max(int(fields[1]),int(fields[2]))<count:graph.append(line)
 (session/'singlesession_posegraph.g2o').write_text('\n'.join(graph)+'\n')
 poses=[]
 for line in graph:
  f=line.split()
  if f[0]!='VERTEX_SE3:QUAT':continue
  x,y,z,qx,qy,qz,qw=map(float,f[2:]);q=np.array([qx,qy,qz]);r=(qw*qw-np.dot(q,q))*np.eye(3)+2*np.outer(q,q)+2*qw*np.array([[0,-qz,qy],[qz,0,-qx],[-qy,qx,0]])
  poses.append(np.column_stack([r,[x,y,z]]).ravel())
 np.savetxt(session/'poses.txt',poses)
configs={}
configs['ltslam']={'ltslam':{'sessions_dir':str(root/'sessions')+'/', 'central_sess_name':'central','query_sess_name':'query','save_directory':str(root/'alignment')+'/', 'numberOfCores':1,'scNumCandidates':3,'kNumSCLoopsUpperBound':4,'kNumRSLoopsUpperBound':4,'loopFitnessScoreThreshold':0.5,'scDistThres':0.6,'rsSearchRadius':10.0}}
configs['removert']={'removert':{'isScanFileKITTIFormat':False,'central_sess_scan_dir':str(root/'sessions/central/Scans'),'central_sess_pose_path':str(root/'sessions/central/poses.txt'),'query_sess_scan_dir':str(root/'sessions/query/Scans'),'query_sess_pose_path':str(root/'sessions/query/poses.txt'),'saveMapPCD':True,'saveCleanScansPCD':True,'save_pcd_directory':str(root/'cleaned'),'ExtrinsicLiDARtoPoseBase':np.eye(4).ravel().tolist(),'start_idx':0,'end_idx':count-1,'keyframe_gap':1,'use_keyframe_gap':True,'remove_resolution_list':[0.5],'revert_resolution_list':[0.5],'sequence_vfov':60.0,'sequence_hfov':360.0,'downsample_voxel_size':0.3,'num_nn_points_within':2,'dist_nn_points_within':0.2,'num_omp_cores':1}}
configs['reconstruct']={**configs['removert'],'map_reconstruct':{'output_dir':str(root/'reconstructed'),'use_precomputed_global_map':False,'feature_knn':10,'scan_predownsample_leaf':0.15}}
for name,config in configs.items():(root/(name+'.yaml')).write_text(yaml.safe_dump(config))
Path('/tmp/ltmapper-validation-path').write_text(str(root));print(root)
