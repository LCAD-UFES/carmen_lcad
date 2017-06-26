include ../Makefile.conf

MODULE_NAME = "LOCALIZE NEURAL"
MODULE_COMMENT = "LOCALIZE NEURAL Module"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src 
LFLAGS += -lparam_interface -lcarmenserial -lipc -lglobal 
LFLAGS += -llocalize_ackerman_interface -lbumblebee_basic_interface -lfused_odometry_interface -lvelodyne_interface
LFLAGS += -lrotation_geometry -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath  
LFLAGS += -lboost_thread-mt -lrt -lboost_signals -lboost_system -lm -pthread

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

SOURCES = localize_neural_dataset.cpp localize_neural_main.cpp

PUBLIC_INCLUDES = localize_neural_messages.h localize_neural_interface.h
PUBLIC_LIBRARIES =  liblocalize_neural_interface.a
PUBLIC_LIBRARIES_SO = liblocalize_neural_interface.so
PUBLIC_BINARIES = localize_neural localize_neural_dataset
TARGETS = localize_neural_dataset localize_neural liblocalize_neural_interface.a

ifndef NO_PYTHON
TARGETS += liblocalize_neural_interface.so.1
endif

liblocalize_neural_interface.a:	localize_neural_interface.o

liblocalize_neural_interface.so.1:	localize_neural_interface.o

localize_neural_dataset: localize_neural_dataset.o 

localize_neural: localize_neural_main.o liblocalize_neural_interface.a

include ../Makefile.rules
