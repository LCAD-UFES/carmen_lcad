include ../Makefile.conf

LFLAGS += -lglobal -lgeometry -lrobot_ackerman_interface -lbase_ackerman_interface \
	-llaser_interface -lparam_interface -lipc 

MODULE_NAME = ROBOT_ACKERMAN_DAEMON
MODULE_COMMENT = Module integrating all robot sensors and configuration

SOURCES = robot_ackerman.c robot_ackerman_interface.c \
	robot_ackerman_main.c robot_ackerman_laser.c 
PUBLIC_INCLUDES = robot_ackerman_interface.h robot_ackerman_messages.h 
PUBLIC_LIBRARIES = librobot_ackerman_interface.a librobot_ackerman.a 
PUBLIC_BINARIES = robot_ackerman

TARGETS =  librobot_ackerman.a robot_ackerman librobot_ackerman_interface.a 

PUBLIC_LIBRARIES_SO = librobot_ackerman_interface.so
ifndef NO_PYTHON
TARGETS += librobot_ackerman_interface.so.1
endif

robot_ackerman:	robot_ackerman.o librobot_ackerman.a

librobot_ackerman.a: robot_ackerman_main.o robot_ackerman_laser.o 

librobot_ackerman_interface.a:	robot_ackerman_interface.o
librobot_ackerman_interface.so.1: robot_ackerman_interface.o

include ../Makefile.rules
