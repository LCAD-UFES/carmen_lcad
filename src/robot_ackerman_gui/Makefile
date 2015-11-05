include ../Makefile.conf

INTERFACE_LIBS = -lobstacle_avoider_interface -lrobot_ackerman_interface -lbase_ackerman_interface

LFLAGS += $(INTERFACE_LIBS) -lparam_interface -lglobal -lipc 

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags`
LFLAGS += -lglobal_graphics `$(GTK_CONFIG) --libs` -lpswrap
endif

ifndef NO_JOYSTICK
LFLAGS += -ljoyctrl
endif

MODULE_NAME = ACKERMAN ROBOT GUI
MODULE_COMMENT = Module for displaying output of ackermans, lasers and sonars and odometry

ifndef NO_GRAPHICS
SOURCES = robot_ackerman_gui.c keyctrl.c
PUBLIC_BINARIES = robot_ackerman_gui
TARGETS = robot_ackerman_gui
endif

robot_ackerman_gui: robot_ackerman_gui.o keyctrl.o

# rules

include ../Makefile.rules
