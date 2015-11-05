OBJS= carmenwrapper.o
APPS= 

LIBS+= -L$(CARMEN_HOME)/lib -lnavigator_interface -lrobot_ackerman_interface \
	-llocalize_core -llocalize_motion \
	-lnavigator_ackerman_interface -lsimulator_ackerman_interface -lrobot_ackerman_interface \
	-lrobot_ackerman_interface \
	-llaser_interface -lparam_interface -lglobal -lipc -lm -lutils -lsensor_range -llog -lscanmatcher -lpthread -lz
CPPFLAGS+=-I../sensor -I$(CARMEN_HOME)/include

-include ../global.mk

ifeq ($(CARMENSUPPORT), 0)
OBJS=
	-include ../build_tools/Makefile.app
else
	-include ../build_tools/Makefile.generic-shared-object
endif
