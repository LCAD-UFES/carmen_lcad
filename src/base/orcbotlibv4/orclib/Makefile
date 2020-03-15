include ../../../Makefile.conf

LFLAGS += -lglobal -lipc -lparam_interface -lcarmenserial

MODULE_NAME = "ORCLIB_V4"
MODULE_COMMENT = "Program for controlling an Orc robot"

SOURCES = orc_lib.c orc_test.c 

TARGETS = liborc.a orc_test

orc_test:	orc_test.o liborc.a

orc_arm:	arm_main.o orc_arm_lib.o

liborc.a:  	orc_lib.o

include ../../../Makefile.rules
