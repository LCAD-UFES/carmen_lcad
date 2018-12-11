
# calibrate odometry to map logs
# convert images to png
# run semantic segmentation for the png images
# synchronize
# convert latlong to xyz  
# run loop closures (optional)
# optimize
# convert pointclouds to ply
# create the following data dir
# rename to kitti format

rm -rf data
mkdir data
mkdir data/velodyne
mkdir data/bb3
mkdir data/semantic

cp /dados/logs/optimized_20180907-2.txt.00.txt data/optimized.txt
cp /dados/logs/sync_latlong_20180907-2.txt data/sync_latlong.txt
awk '{system("cp /dados/logs/log_volta_da_ufes-20180907-2.txt_velodyne_pcd/"$5".ply data/velodyne");}' /dados/logs/optimized_20180907-2.txt.00.txt
awk '{system("cp /dados/logs/log_volta_da_ufes-20180907-2.txt_bumblebee_png/"$6"-r.png data/bb3");}' /dados/logs/optimized_20180907-2.txt.00.txt
awk '{system("cp /dados/experiments/2018_segmap/results_semantic_segmentation/ufes/result_20180907-2/seg_"$6"-r.png data/semantic");}' /dados/logs/optimized_20180907-2.txt.00.txt

mv data data_20180907-2
