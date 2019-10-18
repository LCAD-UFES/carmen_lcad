include ../Makefile.conf

MODULE_NAME = VELODYNE CAMERA CALIBRATION
MODULE_COMMENT = Velodyne Camera Calibration

CFLAGS += -std=c11
CXXFLAGS += -std=c++11 -Wno-deprecated

PCL_INC = $(wildcard /usr/local/include/pcl-* /usr/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system 
LFLAGS += -lpcl_common -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_registration -lpcl_surface -lpcl_io -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization \
			-lpcl_segmentation

SOURCES = velodyne_camera_calibration_main.cpp velodyne_camera_publish_range.cpp velodyne_camera_calibration.cpp save_velodyne_txt.cpp save_velodyne_txt_SqueezeSeg.cpp

PUBLIC_INCLUDES = velodyne_camera_calibration.h
PUBLIC_LIBRARIES = libvelodyne_camera_calibration.a
PUBLIC_BINARIES = 
TARGETS = visual_tracker velodyne_camera_calibration_main velodyne_camera_publish_range save_velodyne_txt save_velodyne_txt_SqueezeSeg libvelodyne_camera_calibration.a

visual_tracker:
	$(MAKE) -C $(CARMEN_HOME)/src/visual_tracker

clean: clean_visual_tracker

clean_visual_tracker:
	$(MAKE) clean -C $(CARMEN_HOME)/src/visual_tracker

velodyne_camera_calibration_main: velodyne_camera_calibration_main.o libvelodyne_camera_calibration.a
velodyne_camera_publish_range: velodyne_camera_publish_range.o libvelodyne_camera_calibration.a
libvelodyne_camera_calibration.a: velodyne_camera_calibration.o
save_velodyne_txt: save_velodyne_txt.o libvelodyne_camera_calibration.a
save_velodyne_txt_SqueezeSeg: save_velodyne_txt_SqueezeSeg.o libvelodyne_camera_calibration.a


include ../Makefile.rules
