include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEGMAP GICP
MODULE_COMMENT = Library and programs for running registration using GICP.

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-deprecated-declarations -Wno-sign-compare

CXXFLAGS += -std=c++11 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

CXXFLAGS += -fopenmp
LFLAGS += -fopenmp

PCL_INC = $(wildcard /usr/local/include/pcl-* /usr/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)
IFLAGS += `pkg-config --cflags opencv`

# SEGMAP PART 1
LFLAGS += -lsegmap_initializations -lsegmap_datasets -lsegmap_readers -lsegmap_slam \
	-lsegmap_readers -lsegmap_motion_models -lutil 

LFLAGS += -L/usr/local/lib  
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -lboost_system -lboost_program_options -lboost_thread-mt -lboost_filesystem

# SEGMAP PART 2
LFLAGS += -lsegmap_types 
LFLAGS += -lsegmap_visualization 

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src
LFLAGS += -lvtkRendering -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system
LFLAGS += -lgrid_mapping -lmapper -lmap_io -lmap_util -lmap_interface -lipc -lprob_models -lm -lz -lglobal 		\
	-lparam_interface -llaser_interface -lipc -lmapper_interface -lobstacle_avoider_interface		\
	-llocalize_ackerman_interface -lmap_interface -lmapper -lipc		\
	-lultrasonic_filter_interface -lstereo_mapping_interface -lrotation_geometry -lvelodyne_interface	\
	-lfused_odometry_interface  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface		\
	`pkg-config --libs opencv` -lgeometry -lmap_server_interface -lsimulator_ackerman_interface -lrddf_interface \
	-lparking_assistant_interface  -llocalize_ackerman_core -llaser_ldmrs_interface -llaser_ldmrs_utils -lmoving_objects_interface
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

SOURCES = generate_loop_closures.cpp gicp.cpp generate_loop_closures_between_logs.cpp pf_loop_closures.cpp \
	segmap_loop_closures.cpp

TARGETS = libsegmap_loop_closures.a generate_loop_closures_between_logs \
	generate_loop_closures pf_loop_closures 

PUBLIC_INCLUDES += gicp.h segmap_loop_closures.h
PUBLIC_LIBRARIES += libsegmap_loop_closures.a

DEPS = $(CARMEN_HOME)/src/segmap/libsegmap/types/libsegmap_types.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/datasets/libsegmap_datasets.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsegmap_readers.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/motion_models/libsegmap_motion_models.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/slam/libsegmap_slam.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/visualization/libsegmap_visualization.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/initializations/libsegmap_initializations.a \
	$(CARMEN_HOME)/src/segmap/util/libutil.a

libsegmap_loop_closures.a: gicp.o segmap_loop_closures.o
generate_loop_closures: generate_loop_closures.o gicp.o segmap_loop_closures.o $(DEPS)
generate_loop_closures_between_logs: generate_loop_closures_between_logs.o gicp.o segmap_loop_closures.o $(DEPS)
pf_loop_closures: pf_loop_closures.o gicp.o segmap_loop_closures.o $(DEPS)

include $(CARMEN_HOME)/src/Makefile.rules
