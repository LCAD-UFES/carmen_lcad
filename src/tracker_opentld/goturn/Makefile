include ../../Makefile.conf

MODULE_NAME = GOTURN-tracker
MODULE_COMMENT = Thrun tracker inside the tracker_opentld module

LINK = g++

#SUBDIRS += Smoother

CFLAGS += -Wall
CXXFLAGS += -Wall -Wno-ignored-qualifiers
IFLAGS += -I $(CUDA_HOME)/targets/x86_64-linux/include -I. -I $(CAFFE_HOME)/include -I $(CAFFE_HOME)/build/include \
		  -I $(CAFFE_HOME)/build/src `pkg-config --cflags opencv` -I /usr/local/cuda-7.5/include/ \
		  # -lcudnn -lcublas \
		  # -D CPU_ONLY 
		  
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I$(CARMEN_HOME)/sharedlib/alglib/src

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/alglib/

#VOICE
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libvoice -Wno-write-strings
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libvoice -lvoice
LFLAGS += -lespeak

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/
	  
LFLAGS += -lboost_system -lboost_filesystem -lboost_regex `pkg-config --libs opencv` \
		  -L $(CAFFE_HOME)/build/lib -lcaffe -ltinyxml -lglog -lopencv_video -lparam_interface -lipc \
		  -lglobal -lbumblebee_basic_interface -lvisual_tracker_interface -lvelodyne_camera_calibration \
		  -lvelodyne_interface -lstereo_interface -llocalize_ackerman_interface -lrotation_geometry \
		  -lobstacle_distance_mapper_interface -lobstacle_avoider_interface -L Smoother -lsmoother -lcollision_detection -lalglib

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath\
		-lboost_thread-mt -lrt -lboost_signals -lboost_system \
		-L/usr/local/cuda-7.5/lib64/ -lcublas

SOURCES = goturn_main.cpp \
		  gui.cpp \
		  regressor/regressor_base.cpp \
		  regressor/regressor_train_base.cpp \
		  regressor/regressor.cpp \
		  regressor/regressor_train.cpp \
		  tracker/tracker.cpp \
		  tracker/tracker_manager.cpp \
		  train/example_generator.cpp \
		  train/tracker_trainer.cpp \
		  loader/loader_alov.cpp \
		  loader/loader_vot.cpp \
		  loader/video_loader.cpp \
		  loader/loader_imagenet_det.cpp \
		  loader/video.cpp \
		  helper/bounding_box.cpp \
		  helper/helper.cpp \
		  helper/high_res_timer.cpp \
		  helper/image_proc.cpp
 
PUBLIC_INCLUDES = 
    
PUBLIC_LIBRARIES =
TARGETS = alglib libvoice.a velodyne_camera_calibration libsmoother goturn_tracker
PUBLIC_BINARIES = 

goturn_tracker: goturn_main.o \
		gui.o \
		regressor/regressor_base.o \
		regressor/regressor_train_base.o \
		regressor/regressor.o \
		regressor/regressor_train.o \
		tracker/tracker.o \
		tracker/tracker_manager.o \
		train/example_generator.o \
		train/tracker_trainer.o \
		loader/loader_alov.o \
		loader/loader_vot.o \
		loader/video_loader.o \
		loader/loader_imagenet_det.o \
		loader/video.o \
		helper/bounding_box.o \
		helper/helper.o \
		helper/high_res_timer.o \
		helper/image_proc.o

# rules
libvoice.a:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libvoice
	
alglib:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/alglib

velodyne_camera_calibration:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne_camera_calibration

libsmoother:
	$(MAKE) -C Smoother

clean: clean_libsmoother

clean_libsmoother:
	$(MAKE) clean -C Smoother

include ../../Makefile.rules
