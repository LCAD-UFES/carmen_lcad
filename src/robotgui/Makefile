include ../Makefile.conf

INTERFACE_LIBS = -lrobot_interface -lbase_interface

LFLAGS += $(INTERFACE_LIBS) -lparam_interface -lglobal -lipc 

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags`
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` -lpswrap
endif

ifndef NO_JOYSTICK
LFLAGS += -ljoyctrl
endif

MODULE_NAME = ROBOTGUI
MODULE_COMMENT = Module for displaying output of robot, lasers and sonars and odometry

ifndef NO_GRAPHICS
SOURCES = robotgui.c keyctrl.c
PUBLIC_BINARIES = robotgui
TARGETS = robotgui
endif

robotgui: robotgui.o keyctrl.o

# rules

include ../Makefile.rules
