include ../Makefile.conf

CFLAGS += -DOLD_MOTION_MODEL

IFLAGS += -I../obstacle_avoider
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/ojTorc/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/libjaus/include/
IFLAGS += -I$(CARMEN_HOME)/sharedlib/OpenJAUS/libopenJaus/include/

LFLAGS += -lmap_interface -lparam_interface -llocalize_ackerman_interface -lobstacle_avoider -lobstacle_avoider_interface \
	  -llaser_interface -lgeometry -llocalize_ackerman_motion -lmotion_planner_interface -lbase_ackerman_interface \
	  -lglobal -lipc -lgrid_mapping_interface -lfused_odometry_interface -lcollision_detection -lmap_server_interface  -lopenJaus -ljaus -lojTorc

LFLAGS	+= -L$(CARMEN_HOME)/sharedlib/OpenJAUS/libjaus/lib/  
LFLAGS	+= -L$(CARMEN_HOME)/sharedlib/OpenJAUS/libopenJaus/lib/
LFLAGS	+= -L$(CARMEN_HOME)/sharedlib/OpenJAUS/ojTorc/lib/	  
	  

MODULE_NAME = FORD_ESCAPE_HYBRID
MODULE_COMMENT = "LCAD driver for TORC/JAUS Ford Escape Hybrid"

SOURCES = ford_escape_hybrid.c ford_escape_pid_controller.c ford_escape_hybrid_interface.c \
	  ford_escape_hybrid_train_base.c ford_escape_hybrid_test.c ford_escape_hybrid_test_curves.c

PUBLIC_INCLUDES = ford_escape_hybrid_messages.h ford_escape_hybrid_interface.h
PUBLIC_LIBRARIES = libford_escape_hybrid_interface.a 
PUBLIC_BINARIES = ford_escape_hybrid ford_escape_hybrid_train_base ford_escape_hybrid_test ford_escape_hybrid_test_curves
MAN_PAGES =

TARGETS = OpenJAUS ford_escape_hybrid ford_escape_hybrid_train_base libford_escape_hybrid_interface.a ford_escape_hybrid_test ford_escape_hybrid_test_curves


ifndef NO_PYTHON
PUBLIC_LIBRARIES_SO = libford_escape_hybrid_interface.so
TARGETS += libford_escape_hybrid_interface.so.1
endif

# rules

# rules
OpenJAUS:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/OpenJAUS/

ford_escape_hybrid: ford_escape_hybrid.o ford_escape_pid_controller.o

ford_escape_hybrid_train_base: ford_escape_hybrid_train_base.o
ford_escape_hybrid_test: ford_escape_hybrid_test.o
ford_escape_hybrid_test_curves: ford_escape_hybrid_test_curves.o

libford_escape_hybrid_interface.a: ford_escape_hybrid_interface.o ford_escape_pid_controller.o

libford_escape_hybrid_interface.so.1: ford_escape_hybrid_interface.o

include ../Makefile.rules
