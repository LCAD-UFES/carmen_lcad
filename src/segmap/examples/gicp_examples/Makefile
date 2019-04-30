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

LFLAGS += -L/usr/local/lib  
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -lboost_system -lboost_program_options -lboost_thread-mt

# SEGMAP PART 1
LFLAGS += -lsegmap_datasets -lsegmap_readers -lsegmap_slam \
	-lsegmap_visualization -lsegmap_readers -lsegmap_motion_models \
	-lcarmen_util

LFLAGS += -lvtkCommon -lvtkFiltering -lvtkRendering -lpcl_common -lpcl_surface -lpcl_io \
	-lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search \
	-lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -lglobal  

# SEGMAP PART 2
LFLAGS += -lsegmap_types

SOURCES = generate_loop_closures.cpp gicp.cpp
	#gicp_odometry_parallel.cpp \
	#gicp_reconstruction.cpp gicp_odometry_view_result.cpp gicp_odometry_view_matching_result.cpp \
	#gicp_odometry_manual_correction.cpp convert_loops_to_graphslam_format.cpp gicp_odometry_simple.cpp \
	#generate_loop_closures_between_logs.cpp gicp_odometry_view_matching_result_loop_closures_between_logs.cpp \
  #filter_incorrect_gicps.cpp 

TARGETS = libgicp.a generate_loop_closures 
	# gicp_odometry gicp_reconstruction gicp_odometry_view_result \
	#gicp_odometry_view_matching_result gicp_odometry_manual_correction convert_loops_to_graphslam_format gicp_odometry_simple \
	#generate_loop_closures_between_logs gicp_odometry_view_matching_result_loop_closures_between_logs filter_incorrect_gicps 

PUBLIC_INCLUDES += gicp.h
PUBLIC_LIBRARIES += libgicp.a

libgicp.a: gicp.o
generate_loop_closures: generate_loop_closures.o ../libsegmap/libsegmap.a gicp.o

#filter_incorrect_gicps: filter_incorrect_gicps.o ../libsegmap/libsegmap.a
#gicp_odometry_view_matching_result_loop_closures_between_logs: gicp_odometry_view_matching_result_loop_closures_between_logs.o ../libsegmap/libsegmap.a gicp.o
#generate_loop_closures_between_logs: generate_loop_closures_between_logs.o ../libsegmap/libsegmap.a  gicp.o
#gicp_odometry: gicp_odometry_parallel.o ../libsegmap/libsegmap.a gicp.o
#gicp_reconstruction: gicp_reconstruction.o ../libsegmap/libsegmap.a gicp.o
#gicp_odometry_view_result: gicp_odometry_view_result.o ../libsegmap/libsegmap.a gicp.o
#gicp_odometry_view_matching_result: gicp_odometry_view_matching_result.o ../libsegmap/libsegmap.a gicp.o
#gicp_odometry_manual_correction: gicp_odometry_manual_correction.o ../libsegmap/libsegmap.a gicp.o
#convert_loops_to_graphslam_format: convert_loops_to_graphslam_format.o ../libsegmap/libsegmap.a gicp.o
#gicp_odometry_simple: gicp_odometry_simple.o ../libsegmap/libsegmap.a gicp.o

include $(CARMEN_HOME)/src/Makefile.rules
