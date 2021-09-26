include ../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lglobal -lparam_interface -lipc -lbase_ackerman_interface -lobstacle_avoider_interface -lbehavior_selector_interface

MODULE_NAME = JOYSTICK_VEHICLE
MODULE_COMMENT = Joystick library and control modules for VEHICLE like a car or truck

ifndef NO_JOYSTICK
SOURCES += joyctrl.c joystick_interface.c joystick_vehicle.c
IFLAGS += $(JOYSTICK_INCLUDE)	
PUBLIC_INCLUDES += joyctrl.h joystick_interface.h joystick_messages.h
PUBLIC_LIBRARIES += libjoyctrl.a libjoystick_interface.a
PUBLIC_BINARIES += joystick_vehicle
TARGETS += libjoyctrl.a joystick_vehicle
endif

libjoystick_interface.a: joystick_interface.o

libjoyctrl.a: joyctrl.o

joystick_vehicle: joystick_vehicle.o joyctrl.o

include ../Makefile.rules
