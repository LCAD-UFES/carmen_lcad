include ../../Makefile.conf

MODULE_NAME = GOTURN-tracker
MODULE_COMMENT = Thrun tracker inside the tracker_opentld module

LINK = g++

CFLAGS += -Wall
CXXFLAGS += -Wall
IFLAGS += -I $(CUDA_HOME)/targets/x86_64-linux/include -I. -I $(CAFFE_HOME)/include -I $(CAFFE_HOME)/build/include \
		  -I $(CAFFE_HOME)/build/src `pkg-config --cflags opencv` -I /usr/local/cuda-7.5/include/ # -D CPU_ONLY
		  
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
		  
LFLAGS += -lboost_system -lboost_filesystem -lboost_regex `pkg-config --libs opencv` \
		  -L $(CAFFE_HOME)/build/lib -lcaffe -ltinyxml -lglog -lopencv_video -lparam_interface -lipc \
		  -lglobal -lbumblebee_basic_interface -lvisual_tracker_interface -lcudnn -lcublas -lvelodyne_camera_calibration \
		  -lvelodyne_interface

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

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
TARGETS = goturn_tracker
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


include ../../Makefile.rules
