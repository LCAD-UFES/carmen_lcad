1.0 Export bumblebee images from one of the logs found in process-mapper.ini
1.1 cd $CARMEN_HOME/bin
1.2 ln -s ../src/localize_neural/process-dataset.ini .
1.3 (move to a second terminal) cd $CARMEN_HOME/src/localize_neural
1.4 mkdir 20140418
1.5 edit process-dataset.ini and uncomment this line ./localize_neural_dataset 8 20140418
1.6 ./proccontrol process-dataset.ini
1.7 mkdir -p /dados/ufes
1.8 mv image_pose.txt /dados/ufes/imagepos-20140418.txt
1.9 mv 20140418 /dados/ufes

2.0 Play the same log to see the groundtruth images and poses exported
2.1 (in the first terminal or) cd $CARMEN_HOME/bin
2.2 ln -s ../src/localize_neural/process-mapper.ini .
2.3 ./proccontrol process-mapper.ini
2.4 From Viewer3D options menu, select Localize Neural to show the TV

3.0 Run ufes_csv.py script to build the ground thruth data for the train/test/validation datasets exported above
3.1 git clone https://<username>@bitbucket.org/lcad/deepslam
3.2 cd deepslam/tflow/deepodom2
3.3 python make_data.py
3.4 python train.py