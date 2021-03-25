include ../Makefile.conf

MODULE_NAME = "LOCALIZE NEURAL"
MODULE_COMMENT = "LOCALIZE NEURAL Module"

IFLAGS += -isystem /usr/include/bullet/
IFLAGS += -isystem $(CARMEN_HOME)/sharedlib/libtf/src 
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 
LFLAGS += -lparam_interface -lcarmenserial -lipc -lglobal -lmovement
LFLAGS += -llocalize_ackerman_interface -lbumblebee_basic_interface -lfused_odometry_interface -lvelodyne_interface
LFLAGS += -lrotation_geometry -lmapper_interface -lcamera_drivers
LFLAGS += -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath  
LFLAGS += -lboost_thread-mt -lrt -lboost_signals -lboost_system -lm -pthread

# Torch dependencies
TORCH := $(shell if [ -e $(HOME)/torch/install/lib ] ; then echo $(HOME)/torch/install ; else echo /usr/local ; fi)
IFLAGS += -I $(TORCH)/include -I $(TORCH)/include/TH
LFLAGS += -L$(TORCH)/lib -lTH -lluaT -lluajit -llapack

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

#CFLAGS += -nostartfiles 

SOURCES = localize_neural_dataset.cpp localize_neural_interface.cpp #localize_neural_main.cpp localize_neural_torch.cpp localize_neural_dataset.cpp 

PUBLIC_INCLUDES = localize_neural_messages.h localize_neural_interface.h
PUBLIC_LIBRARIES =  liblocalize_neural_interface.a
PUBLIC_LIBRARIES_SO = liblocalize_neural_interface.so
PUBLIC_BINARIES = localize_neural_dataset #localize_neural localize_neural_dataset
TARGETS = localize_neural_dataset liblocalize_neural_interface.a #localize_neural localize_neural_dataset

ifndef NO_PYTHON
TARGETS += liblocalize_neural_interface.so.1
endif

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

liblocalize_neural_interface.a:	localize_neural_interface.o

liblocalize_neural_interface.so.1: localize_neural_interface.o

localize_neural_dataset: localize_neural_dataset.o 

#localize_neural: liblocalize_neural_interface.a localize_neural_main.o localize_neural_torch.o

include ../Makefile.rules
