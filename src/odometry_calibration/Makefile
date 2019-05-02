include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = ODOMETRY_CALIBRATION
MODULE_COMMENT = Module to calibrate the bias of the Odometry

LINK = g++

CFLAGS += -std=c11
CXXFLAGS += -std=c++11

SUBDIRS += $(CARMEN_HOME)/src/utilities/libcarmen_util $(CARMEN_HOME)/sharedlib/pso/ $(CARMEN_HOME)/sharedlib/libtf/src

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ 
LFLAGS += -lglobal -lipc -llocalize_ackerman_interface -lfused_odometry_interface -lvelodyne_interface -lparam_interface -lGL -lglut -lGLU \
	 	-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system    
LFLAGS += -lrobot_ackerman_interface -lobstacle_avoider_interface \
	-lmap_io -lmap_util -llaser_interface -lfused_odometry_interface -lvelodyne_interface \
	-lGLEW -lGL -lGLU -lglut -lX11 -lmap_interface -lbase_ackerman_interface \
	-lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface -llocalize_ackerman_core -llocalize_ackerman_interface \
	-llocalize_ackerman_motion -lmap_server_interface -lgps_xyz_interface -lvisual_odometry_interface -lxsens_mtig_interface -lxsens_interface
IFLAGS += -I/usr/include/eigen3 
LFLAGS += -L /usr/local/lib -lcholmod -lg2o_core -lg2o_stuff -lg2o_types_slam2d -lg2o_cli
IFLAGS += -I /usr/include/suitesparse -I/usr/include/eigen3 -I /usr/local/include/EXTERNAL/csparse/ -I /usr/local/include/g2o/
LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree \
		  -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system

PCL_INC = $(wildcard /usr/local/include/pcl-* /usr/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter -lprob_models -lrotation_geometry -lgsl -lgslcblas
IFLAGS += `pkg-config --cflags opencv`
IFLAGS += -I$(CARMEN_HOME)/sharedlib/pso 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/pso -lpso++ -fopenmp
LFLAGS += `pkg-config --libs opencv`	  
LFLAGS += -lmapper_interface -lgrid_mapping -lmap_io -lmap_interface
LFLAGS += -lcarmen_util -lboost_program_options

SOURCES = grab_data_for_calibration.cpp calibrate_bias.cpp calibrate_bias_from_log.cpp filter_data_for_calibration.cpp tf_test.cpp # calibrate_bias_from_log_with_neural_phi.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = grab_data_for_calibration calibrate_bias calibrate_bias_from_log filter_data_for_calibration calibrate_bias_from_log_with_neural_phi 
TARGETS = calibrate_bias calibrate_bias_from_log filter_data_for_calibration grab_data_for_calibration tf_test # calibrate_bias_from_log_with_neural_phi

LIBS_DEPS = $(CARMEN_HOME)/lib/libtf.a $(CARMEN_HOME)/lib/libpso++.a $(CARMEN_HOME)/lib/libcarmen_util.a 

grab_data_for_calibration: grab_data_for_calibration.o
calibrate_bias: calibrate_bias.o
calibrate_bias_from_log: calibrate_bias_from_log.o $(LIBS_DEPS)
filter_data_for_calibration: filter_data_for_calibration.o
#calibrate_bias_from_log_with_neural_phi: calibrate_bias_from_log_with_neural_phi.o
tf_test: tf_test.o $(LIBS_DEPS)

include $(CARMEN_HOME)/src/Makefile.rules
