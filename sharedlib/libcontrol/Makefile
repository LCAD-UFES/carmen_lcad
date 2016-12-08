include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x

MODULE_NAME = LIBCONTROL
MODULE_COMMENT = Low Level Control Library

IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS += -lcarmodel -lgsl -lgslcblas -lfann -lobstacle_avoider_interface

SOURCES = mpc/mpc.cpp mpc/mpc_position_control.cpp mpc/mpc_velocity.cpp mpc/mpc_plot.cpp pid/pid.cpp rlpid/rlpid.cpp

PUBLIC_INCLUDES = control.h 

TARGETS = libcontrol.a

clean: clean_mpc
clean_mpc:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/mpc/

clean: clean_pid
clean_pid:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/pid/

clean: clean_rlpid
clean_rlpid:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/rlpid/

libcontrol.a: mpc/mpc.o mpc/mpc_position_control.o mpc/mpc_velocity.o mpc/mpc_plot.o pid/pid.o rlpid/rlpid.o

include $(CARMEN_HOME)/src/Makefile.rules