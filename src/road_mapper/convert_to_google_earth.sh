cd $CARMEN_HOME/src/road_mapper/
python convert_enet_train_to_google_earth_coordinates.py -d 12.0 -i road_mapper_train_guarapari.txt -o road_mapper_train_guarapari_google_earth.txt
python convert_graphslam_poses_to_google_earth_coordinates.py -d 12.0 -i $CARMEN_HOME/data/graphslam/poses_opt-log_guarapari-20170403-2.txt -o poses_opt-log_guarapari-20170403-2_google_earth.txt
python convert_graphslam_poses_to_google_earth_coordinates.py -d 12.0 -i $CARMEN_HOME/data/graphslam/poses-opt-log_volta_da_ufes_road_mapper_20180122.txt.txt -o poses-opt-log_volta_da_ufes_road_mapper_20180122_google_earth.txt
echo "set xrange [330000:370000]" > convert_to_google_earth.gp
echo "set yrange [7700000:7770000]" >> convert_to_google_earth.gp
echo "plot \"<awk -F, '{print \$1,\$2}' poses_opt-log_guarapari-20170403-2_google_earth.txt\" u 1:2 w points ps 0.01 lc rgb 'blue' title 'not trained'" >> convert_to_google_earth.gp
echo "replot \"<awk -F, '{print \$1,\$2}' road_mapper_train_guarapari_google_earth.txt\" u 1:2 w points ps 0.01 lc rgb 'red' title 'trained'" >> convert_to_google_earth.gp
echo "replot \"<awk -F, '{print \$1,\$2}' voltadaufes_train_google_earth.txt\" u 1:2 w points ps 0.01 lc rgb 'red' title 'trained'" >> convert_to_google_earth.gp
echo "pause -1" >> convert_to_google_earth.gp
echo "Press any key to finish..."
gnuplot convert_to_google_earth.gp
