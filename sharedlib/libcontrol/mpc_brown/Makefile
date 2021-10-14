
include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = MPC_BROWN
MODULE_COMMENT = mpc_brown

IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS += -lglobal -lipc
SOURCES = main.cpp trajectories.cpp   math.cpp model_predictive_control.cpp vehicle_dynamics.cpp vehicles.cpp coupled_lat_long.cpp decoupled_lat_long.cpp mpc_brown_interface.cpp


PUBLIC_INCLUDES = mpc_brown_interface.h mpc_brown_messages.h
PUBLIC_LIBRARIES = mpc_brown_interface.a
PUBLIC_BINARIES = mpc_brown

TARGETS = mpc_brown mpc_brown_interface.a



mpc_brown: main.o trajectories.o   math.o model_predictive_control.o vehicle_dynamics.o vehicles.o coupled_lat_long.o decoupled_lat_long.o mpc_brown_interface.o

mpc_brown_interface.a: mpc_brown_interface.o 
include $(CARMEN_HOME)/src/Makefile.rules
