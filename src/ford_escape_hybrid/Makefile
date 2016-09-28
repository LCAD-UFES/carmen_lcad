include ../Makefile.conf

CFLAGS += -DOLD_MOTION_MODEL -O4

IFLAGS += -I../obstacle_avoider
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/ojTorc/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/libjaus/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/libopenJaus/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcarneuralmodel
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/pid
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol/mpc
IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS += -lmap_interface -lparam_interface -llocalize_ackerman_interface -lobstacle_avoider -lobstacle_avoider_interface \
	  -llaser_interface -lgeometry -llocalize_ackerman_motion -lmotion_planner_interface -lbase_ackerman_interface \
	  -lglobal -lipc -lmapper_interface -lfused_odometry_interface -lcollision_detection -lmap_server_interface \
	  -lopenJaus -ljaus -lojTorc -lmpc -lgsl -lgslcblas -lcarneuralmodel -lpid -lfann

LFLAGS += -L$(CARMEN_HOME)/sharedlib/OpenJAUS/libjaus/lib/  
LFLAGS += -L$(CARMEN_HOME)/sharedlib/OpenJAUS/libopenJaus/lib/
LFLAGS += -L$(CARMEN_HOME)/sharedlib/OpenJAUS/ojTorc/lib/	  
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarneuralmodel
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol/pid
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol/mpc

MODULE_NAME = FORD_ESCAPE_HYBRID
MODULE_COMMENT = "LCAD driver for TORC/JAUS Ford Escape Hybrid"

SOURCES = ford_escape_hybrid.c ford_escape_hybrid_interface.c ford_escape_hybrid_train_base.c \
	  ford_escape_hybrid_test.c ford_escape_hybrid_test_curves.c #ford_escape_pid_controller.c

PUBLIC_INCLUDES = ford_escape_hybrid_messages.h ford_escape_hybrid_interface.h ford_escape_hybrid.h
PUBLIC_LIBRARIES = libford_escape_hybrid_interface.a 
PUBLIC_BINARIES = ford_escape_hybrid ford_escape_hybrid_train_base ford_escape_hybrid_test ford_escape_hybrid_test_curves
MAN_PAGES =

TARGETS = OpenJAUS libcarneuralmodel libpid libmpc ford_escape_hybrid ford_escape_hybrid_train_base libford_escape_hybrid_interface.a ford_escape_hybrid_test ford_escape_hybrid_test_curves


ifndef NO_PYTHON
PUBLIC_LIBRARIES_SO = libford_escape_hybrid_interface.so
TARGETS += libford_escape_hybrid_interface.so.1
endif

# rules
OpenJAUS:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/OpenJAUS/

libcarneuralmodel:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcarneuralmodel
clean: clean_libcarneuralmodel
clean_libcarneuralmodel:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcarneuralmodel

libpid:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcontrol/pid/
clean: clean_libpid
clean_libpid:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/pid/

libmpc:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libcontrol/mpc/
clean: clean_libmpc
clean_libmpc:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libcontrol/mpc/

ford_escape_hybrid: ford_escape_hybrid.o #ford_escape_pid_controller.o

ford_escape_hybrid_train_base: ford_escape_hybrid_train_base.o
ford_escape_hybrid_test: ford_escape_hybrid_test.o
ford_escape_hybrid_test_curves: ford_escape_hybrid_test_curves.o

libford_escape_hybrid_interface.a: ford_escape_hybrid_interface.o #ford_escape_pid_controller.o

libford_escape_hybrid_interface.so.1: ford_escape_hybrid_interface.o

include ../Makefile.rules
