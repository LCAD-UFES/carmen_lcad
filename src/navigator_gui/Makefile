include ../Makefile.conf

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src -L/usr/local/carmen_boost/lib

IFLAGS += `$(GTK_CONFIG) --cflags`
LFLAGS += $(INTERFACE_LIBS) -lgrid_mapping -lprob_models -lX11 -lglobal -lipc \
	-lgeometry -lmap_interface -lobstacle_avoider_interface -ldynamic_object_detector_interface \
	-lparam_interface -lmapper_interface -lbase_ackerman_interface \
	-lsimulator_ackerman_interface -llocalize_ackerman_interface -lnavigator_ackerman_interface  \
	-lmap_graphics -lglobal_graphics `pkg-config --libs gtk+-2.0` -lfused_odometry_interface \
	-lmap_io -lrddf_interface -lmap_server_interface -lbehavior_selector_interface \
	-ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_thread-mt -lboost_system \
	-lrt -lboost_signals -lrotation_geometry -lrobot_ackerman_interface


MODULE_NAME = NAVIGATOR GUI
MODULE_COMMENT = The motion planner Interface!

SOURCES = navigator_graphics.c navigator_panel.c navigator_gui_interface.c

PUBLIC_LIBRARIES = libnavigator_gui_interface.a
PUBLIC_BINARIES += navigator_gui
PUBLIC_INCLUDES = navigator_gui_interface.h navigator_gui_messages.h

TARGETS += libprob_models.a navigator_gui libnavigator_gui_interface.a

libnavigator_gui_interface.a: navigator_gui_interface.o

navigator_gui: navigator_panel.o navigator_graphics.o libnavigator_gui_interface.a

# rules

libprob_models.a: 
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/prob_models/

clean: clean_prob_models

clean_prob_models:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/prob_models/
	
include ../Makefile.rules
