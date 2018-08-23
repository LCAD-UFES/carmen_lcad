include ../Makefile.conf

CFLAGS += -std=c99
LFLAGS += 

LINK = g++

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src

LFLAGS += -lgrid_mapping -lmapper_interface -lglobal -lparam_interface -lmap_io -lmap_util -llaser_interface \
	  -lrobot_ackerman_interface -lobstacle_avoider_interface -lipc -lmap_server_interface \
	  -lfused_odometry_interface -lvelodyne_interface -lrotation_geometry -lprob_models -lgl_plotter \
	  -lGLEW -lGL -lGLU -lglut -lX11 -lmap_interface -lbase_ackerman_interface \
	  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface -lmoving_objects_interface \
	  -llocalize_ackerman_interface -llocalize_ackerman_core -llocalize_ackerman_motion

LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

IFLAGS += -I/usr/include/eigen3 -I/usr/local/include/pcl-1.7/  -I/usr/include/vtk-55.8/
LFLAGS += -lvtkCommon -lvtkFiltering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization

MODULE_NAME = VELODYNE ODOMETRY
MODULE_COMMENT = Odometry using Velodyne remission

SOURCES = velodyne_odometry_main.cpp velodyne_odometry_velodyne.cpp 
	  
PUBLIC_INCLUDES = 
		  
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = velodyne_odometry

MAN_PAGES =

TARGETS = velodyne_odometry		

# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0`
LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0`
endif


PUBLIC_LIBRARIES_SO = 
ifndef NO_PYTHON
TARGETS += 
endif

# rules

velodyne_odometry:	velodyne_odometry_main.o velodyne_odometry_velodyne.o

include ../Makefile.rules
