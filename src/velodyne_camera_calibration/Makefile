include ../Makefile.conf

MODULE_NAME = VELODYNE CAMERA CALIBRATION
MODULE_COMMENT = Velodyne Camera Calibration

CFLAGS += -std=c11
CXXFLAGS += -std=c++11 -Wno-deprecated -Wno-parentheses -Wno-deprecated-copy -Wno-implicit-fallthrough -Wno-deprecated-declarations

PCL_INC = $(wildcard /usr/local/include/pcl-*)
#VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5* /usr/include/vtk-6*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) # -I $(VTK_INC)

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/include/bullet/ -I/usr/local/include/bullet/ -I/usr/local/carmen_boost/include

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib
LFLAGS += -lglobal -lipc -lparam_interface -lvelodyne_interface -lbumblebee_basic_interface -lvisual_tracker_interface \
		  `pkg-config --libs opencv` -ltf -lpthread -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
		  -lboost_thread-mt -lrt -lboost_signals -lboost_system -lparam_interface
LFLAGS += -lpcl_common -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_registration -lpcl_surface -lpcl_io -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization \
			-lpcl_segmentation # -lvtkRendering -lvtkFiltering -lvtkCommon

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ -I/usr/local/carmen_boost/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib

#SOURCES = velodyne_camera_calibration_main.cpp velodyne_camera_calibration_manual.cpp velodyne_camera_publish_range.cpp velodyne_camera_calibration.cpp save_velodyne_txt.cpp
SOURCES = velodyne_camera_calibration_main.cpp velodyne_camera_publish_range.cpp velodyne_camera_calibration.cpp save_velodyne_txt.cpp

PUBLIC_INCLUDES = velodyne_camera_calibration.h
PUBLIC_LIBRARIES = libvelodyne_camera_calibration.a
PUBLIC_BINARIES = 
#TARGETS = velodyne_camera_calibration_main velodyne_camera_calibration_manual velodyne_camera_publish_range save_velodyne_txt libvelodyne_camera_calibration.a
TARGETS = velodyne_interface visual_tracker velodyne_camera_calibration_main velodyne_camera_publish_range libvelodyne_camera_calibration.a #save_velodyne_txt
#TARGETS += visual_tracker

velodyne_interface:
	$(MAKE) -C $(CARMEN_HOME)/src/velodyne

visual_tracker:
	$(MAKE) -C $(CARMEN_HOME)/src/visual_tracker

clean: clean_velodyne_interface clean_visual_tracker

clean_velodyne_interface:
	$(MAKE) clean -C $(CARMEN_HOME)/src/velodyne

clean_visual_tracker:
	$(MAKE) clean -C $(CARMEN_HOME)/src/visual_tracker


velodyne_camera_calibration_main: velodyne_camera_calibration_main.o libvelodyne_camera_calibration.a
# velodyne_camera_calibration_manual: velodyne_camera_calibration_manual.o libvelodyne_camera_calibration.a
velodyne_camera_publish_range: velodyne_camera_publish_range.o libvelodyne_camera_calibration.a
libvelodyne_camera_calibration.a: velodyne_camera_calibration.o
#save_velodyne_txt: save_velodyne_txt.o libvelodyne_camera_calibration.a


include ../Makefile.rules
