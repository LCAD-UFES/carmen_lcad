include ../Makefile.conf

#SUBDIRS = model_learn_ackerman

CFLAGS += -std=c11 -fopenmp
CXXFLAGS += -std=c++11 -fopenmp
LINK = g++

#CFLAGS += -std=c11 -fopenmp -fsanitize=address -fno-omit-frame-pointer
#CXXFLAGS += -std=c++11 -fopenmp -fsanitize=address -fno-omit-frame-pointer
#LINK = g++ -fsanitize=address -fno-omit-frame-pointer

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/gl_plotter 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/gl_plotter

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ -I/usr/include/bullet/ 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/lib/x86_64-linux-gnu

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarmodel
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol

LFLAGS += -lgrid_mapping -lmapper_interface -lglobal -lparam_interface -lmap_io -lmap_util -llaser_interface \
	  -lrobot_ackerman_interface -lobstacle_avoider_interface -lipc -lmap_server_interface \
	  -lfused_odometry_interface -lvelodyne_interface -lrotation_geometry -lprob_models -lgl_plotter \
	  -lGLEW -lGL -lGLU -lglut -lX11 -lmap_interface -lbase_ackerman_interface -lcarmodel \
	  -lstereo_velodyne -lstereo_velodyne_interface -lstereo_interface -lmoving_objects_interface -lxsens_interface \
	  -ltask_manager_interface -lbehavior_selector_interface -lgsl -lgslcblas -lcarmodel -ldbscan \
	  -lgps_xyz_interface `pkg-config --libs opencv`

LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lrt -lboost_signals -lboost_system

UBUNTU_VERSION ="$(shell cat /etc/issue | grep Ubuntu | cut -c8-12)"
ifeq ($(UBUNTU_VERSION),"16.04")
	PCL_INC = $(wildcard /usr/include/pcl-*)
	VTK_INC1 = $(wildcard /usr/local/include/vtk-5*)
	VTK_INC2 = $(wildcard /usr/include/vtk-*)
	IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC1) -I $(VTK_INC2)
else
	PCL_INC = $(wildcard /usr/include/pcl-*)
	VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
	IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)
endif

LFLAGS += -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization

MODULE_NAME = LOCALIZE ACKERMAN
MODULE_COMMENT = Markov Localization Module

SOURCES = localize_ackerman_main.cpp localize_ackerman_using_map.cpp localize_ackerman_likelihood_map.c localize_ackerman_core.cpp localize_ackerman_interface.c \
	  localize_ackerman_initialize.c localize_ackerman_motion.c localize_ackerman_velodyne.cpp localize_ackerman_particle_viewer.c localize_ackerman_beta_particle_filter.cpp
	  
PUBLIC_INCLUDES = localize_ackerman_messages.h localize_ackerman_interface.h localize_ackerman_core.h \
	          localize_ackerman_likelihood_map.h localize_ackerman_motion.h localize_ackerman_velodyne.h localize_ackerman_beta_particle_filter.h
		  
PUBLIC_LIBRARIES = liblocalize_ackerman_interface.a liblocalize_ackerman_core.a liblocalize_ackerman_motion.a liblocalize_ackerman_beta_particle_filter.a liblocalize_ackerman_trailers_theta.a
PUBLIC_BINARIES = localize_ackerman localize_ackerman_particle_viewer

MAN_PAGES =

TARGETS = libprob_models.a libcontrol.a libcarmodel.a libgl_plotter.a localize_ackerman liblocalize_ackerman_interface.a \
		liblocalize_ackerman_core.a liblocalize_ackerman_motion.a liblocalize_ackerman_beta_particle_filter.a model_learn_ackerman/model_learner_ackerman \
		localize_ackerman_particle_viewer		

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


PUBLIC_LIBRARIES_SO = liblocalize_ackerman_interface.so
ifndef NO_PYTHON
TARGETS += liblocalize_ackerman_interface.so.1
endif

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

libcontrol.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcontrol/

libcarmodel.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarmodel/

libgl_plotter.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/gl_plotter/

liblocalize_ackerman_interface.a:	localize_ackerman_interface.o

liblocalize_ackerman_interface.so.1:	localize_ackerman_interface.o

liblocalize_ackerman_core.a: localize_ackerman_core.o localize_ackerman_initialize.o localize_ackerman_likelihood_map.o localize_ackerman_velodyne.o localize_ackerman_trailers_theta.o

liblocalize_ackerman_motion.a: localize_ackerman_motion.o

liblocalize_ackerman_beta_particle_filter.a: localize_ackerman_beta_particle_filter.o

localize_ackerman:	localize_ackerman_main.o localize_ackerman_using_map.o localize_ackerman_velodyne.o localize_ackerman_trailers_theta.o liblocalize_ackerman_core.a liblocalize_ackerman_interface.a \
	liblocalize_ackerman_motion.a liblocalize_ackerman_beta_particle_filter.a 

localize_ackerman_graph:	localize_ackerman_graph.o localize_ackerman_likelihood_map.o liblocalize_ackerman_interface.a

localize_ackerman_particle_viewer: localize_ackerman_particle_viewer.o localize_ackerman_interface.o

clean: clean_prob_models clean_gl_plotter clean_libcontrol clean_libcarmodel

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/

clean_libcontrol: 
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/

clean_libcarmodel: 
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarmodel/

clean_gl_plotter:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/gl_plotter/

model_learn_ackerman/model_learner_ackerman: 
	$(MAKE) -C model_learn_ackerman

include ../Makefile.rules



