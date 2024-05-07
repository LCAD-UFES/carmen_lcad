include ../Makefile.conf

MODULE_NAME = Neural Object Detector
MODULE_COMMENT = Detect vehicles using convolutional neural networks.

LINK = g++
CXXFLAGS = -std=c++11

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/include/bullet/ -I/usr/local/include/opencv4 -I/usr/local/include/bullet/ -I/usr/local/carmen_boost/include -I/usr/include/python3.8 -DBOOST_SIGNALS_NO_DEPRECATION_WARNING 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib 

IFLAGS += -I/usr/local/include/onnxruntime/include
LFLAGS += -L/usr/lib/x86_64-linux-gnu
#LFLAGS += -L/usr/local/lib
#LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet2/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -L$(CARMEN_HOME)/sharedlib/darknet3/lib -L$(CARMEN_HOME)/sharedlib/libtf/src

# Application specific include directories.
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libsickldmrs2/include
IFLAGS += -I$(CARMEN_HOME)/src/lpl/play_with_tensorrt/pj_tensorrt_perception_yolopv2/image_processor
IFLAGS += -I$(CARMEN_HOME)/src/lpl/play_with_tensorrt/common_helper

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libsickldkemrs2 
#LFLAGS += -L/home/alefe/packages_carmen/opencv-4.5.5/build/lib
LFLAGS += -L/usr/lib/gcc/x86_64-linux-gnu/9
LFLAGS += -L/usr/lib/x86_64-linux-gnu

LFLAGS += -L/usr/local/cuda/lib64
LFLAGS += -L$(CARMEN_HOME)/src/lpl/play_with_tensorrt/pj_tensorrt_perception_yolopv2/build/image_processor/common_helper
LFLAGS += -L$(CARMEN_HOME)/src/lpl/play_with_tensorrt/pj_tensorrt_perception_yolopv2/build/image_processor/inference_helper
LFLAGS += -L$(CARMEN_HOME)/src/lpl/play_with_tensorrt/pj_tensorrt_perception_yolopv2/build/image_processor



#LFLAGS += -lglobal -lipc -lvelodyne_interface -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -lmoving_objects_interface -ldarknet2 `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration \
		  -ltraffic_light_interface -llaser_ldmrs_interface -llaser_ldmrs_utils -lrddf_interface `python3.8-config --libs` \
		  -lcamera_drivers -lpython3.8

#LFLAGS += -lglobal -lipc -lvelodyne_interface  -lpthread -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
		  -lmoving_objects_interface -lopencv_imgproc -lopencv_imgcodecs -ldarknet `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
		  -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration \
		  -ltraffic_light_interface -llaser_ldmrs_interface -llaser_ldmrs_utils -lrddf_interface `python3.8-config --libs` \
		  -lcamera_drivers -lpython3.8 -lonnxruntime  -lopencv_core -lopencv_features2d -lopencv_highgui  \
		 -lImageProcessor -lCommonHelper -lInferenceHelper -lopencv_imgproc -lopencv_imgcodecs -lopencv_core  -lopencv_highgui -pthread -fopenmp -lcuda -lcudart \
		 -lopencv_calib3d -lopencv_core -lopencv_dnn -lopencv_features2d -lopencv_flann -lopencv_gapi -lopencv_highgui -lopencv_imgcodecs -lopencv_imgproc -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching -lopencv_video -lopencv_videoio -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib -lopencv_datasets -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc -lopencv_optflow -lopencv_phase_unwrapping -lopencv_plot -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_structured_light -lopencv_superres -lopencv_surface_matching -lopencv_text -lopencv_tracking -lopencv_videostab -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_ximgproc -lopencv_xobjdetect -lopencv_xphoto \
		 -lnvinfer -lnvinfer_plugin -lnvonnxparser

LFLAGS += -lglobal -lipc -lvelodyne_interface -lpthread -lbumblebee_basic_interface -lparam_interface -llocalize_ackerman_interface \
          -lmoving_objects_interface -ldarknet `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics \
          -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system -lvelodyne_camera_calibration \
          -ltraffic_light_interface -llaser_ldmrs_interface -llaser_ldmrs_utils -lrddf_interface `python3.8-config --libs` \
          -lcamera_drivers -lpython3.8 -lonnxruntime -lopencv_imgproc -lopencv_imgcodecs -lopencv_core -lopencv_highgui \
          -lImageProcessor -lCommonHelper -lInferenceHelper -pthread -fopenmp -lcuda -lcudart -lopencv_calib3d -lopencv_dnn \
          -lopencv_features2d -lopencv_flann -lopencv_gapi -lopencv_ml -lopencv_objdetect -lopencv_photo -lopencv_stitching \
          -lopencv_video -lopencv_videoio -lopencv_aruco -lopencv_barcode -lopencv_bgsegm -lopencv_bioinspired -lopencv_ccalib \
          -lopencv_datasets -lopencv_dnn_objdetect -lopencv_dnn_superres -lopencv_dpm -lopencv_face -lopencv_freetype -lopencv_fuzzy \
          -lopencv_hdf -lopencv_hfs -lopencv_img_hash -lopencv_intensity_transform -lopencv_line_descriptor -lopencv_mcc \
          -lopencv_optflow -lopencv_phase_unwrapping -lopencv_plot -lopencv_quality -lopencv_rapid -lopencv_reg -lopencv_rgbd \
          -lopencv_saliency -lopencv_shape -lopencv_stereo -lopencv_structured_light -lopencv_superres -lopencv_surface_matching \
          -lopencv_text -lopencv_tracking -lopencv_videostab -lopencv_wechat_qrcode -lopencv_xfeatures2d -lopencv_ximgproc \
          -lopencv_xobjdetect -lopencv_xphoto -lnvinfer -lnvinfer_plugin -lnvonnxparser

LFLAGS += -lopencv_core -lopencv_highgui -lopencv_imgproc -lopencv_imgcodecs



SOURCES = neural_object_detector_tracker.cpp

PUBLIC_BINARIES = neural_object_detector_tracker

TARGETS = darknet3 neural_object_detector_tracker

neural_object_detector_tracker: neural_object_detector_tracker.o

darknet3:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/darknet3

download:
#	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1rE0lY4d4wyMp4mvcx23y4qGK97bvzTnC/view?usp=sharing $(CARMEN_HOME)/sharedlib/darknet3/yolov3.weights
	mkdir pedestrian_tracker/data; $(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1MgaW3ndzOiKWJI8VVkAgTLL4tHzDpxc8/view?usp=sharing $(CARMEN_HOME)/src/neural_object_detector3/pedestrian_tracker/data/googlenet_part8_all_xavier_ckpt_56.h5
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1a84AIWEsFufsXplKRS26z7FsxNt31KUb/view?usp=sharing $(CARMEN_HOME)/src/neural_object_detector3/pedestrian_tracker/data/squeezenet_small40_coco_mot16_ckpt_10.h5
	cd $(CARMEN_HOME)/sharedlib/darknet3; wget https://github.com/AlexeyAB/darknet/releases/download/darknet_yolo_v3_optimal/yolov4.weights

include ../Makefile.rules
