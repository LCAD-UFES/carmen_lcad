0.0 Install tensorflow & opencv
0.1.1 git clone https://github.com/opencv/opencv.git
0.1.2 git clone https://github.com/opencv/opencv_contrib.git
0.1.3 cd opencv && mkdir build && cmake -D WITH_CUDA=OFF -D WITH_OPENCL=OFF -D OPENCV_EXTRA_MODULES_PATH=../../opencv_contrib/modules ..
0.1.4 make -j5 && sudo make install && sudo ldconfig
0.2.1 git clone https://github.com/aforechi/tensorflow_cc
0.2.2 cd tensorflow_cc/tensorflow_cc && mkdir build && cmake -DTENSORFLOW_STATIC=OFF -DTENSORFLOW_SHARED=ON ..
0.2.3 make -j5 && sudo make install
0.2.4 export LD_LIBRARY_PATH=/usr/local/lib/tensorflow_cc:$LD_LIBRARY_PATH
0.2.5 Install tensorflow from sources (or virtualenv) using version v1.2.1

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

