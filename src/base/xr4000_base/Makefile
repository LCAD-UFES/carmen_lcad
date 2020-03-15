include ../../Makefile.conf

CFLAGS += -DNOJOYSTICK
IFLAGS += 
LFLAGS += -lbase_interface -lparam_interface -lglobal -lipc

MODULE_NAME = XR4000
MODULE_COMMENT = CARMEN Base control module for Nomadic Technologies XR4000

SOURCES = arcnet.c arcnet_sensors.c hash.c i200m.c pc_d51.c buffer.c llc1.c robot_client.c mac.c rpc.c default_setup.c setup.c xr4000.c xr4000_control.c xr4000_ipc.c 

TARGETS = ../xr4000 

# rules

../xr4000: arcnet.o arcnet_sensors.o hash.o i200m.o pc_d51.o buffer.o llc1.o robot_client.o mac.o rpc.o default_setup.o setup.o xr4000.o xr4000_control.o xr4000_ipc.o

include ../../Makefile.rules
