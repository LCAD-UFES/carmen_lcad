include ../Makefile.conf

CFLAGS +=
IFLAGS += 
LFLAGS += -lglobal -lparam_interface -lipc -lbase_ackerman_interface -lobstacle_avoider_interface

MODULE_NAME = JOYSTICK
MODULE_COMMENT = Joystick library and control modules

ifndef NO_JOYSTICK
SOURCES += joyctrl.c joystick.c joystick_interface.c wingman_control.c
IFLAGS += $(JOYSTICK_INCLUDE)	
PUBLIC_INCLUDES += joyctrl.h joystick_interface.h joystick_messages.h
PUBLIC_LIBRARIES += libjoyctrl.a libjoystick_interface.a
PUBLIC_BINARIES += wingman_control joystick
TARGETS += libjoyctrl.a wingman_control joystick
endif

libjoystick_interface.a: joystick_interface.o

libjoyctrl.a: joyctrl.o

joystick: joystick.o joyctrl.o libjoystick_interface.a

wingman_control: wingman_control.o joyctrl.o

include ../Makefile.rules
