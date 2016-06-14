include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SICK LD-MRS Library
MODULE_COMMENT = SICK LD-MRS Library

LINK = g++

LFLAGS += -lrt
IFLAGS += -Iinclude

SOURCES = src/commands.c src/print.c src/sickldmrs.c test/sickldmrstest.c
TARGETS = sickldmrstest libsickldmrs.a 

ifndef NO_PYTHON
TARGETS += libsickldmrs.so.1
endif

sickldmrstest: test/sickldmrstest.o libsickldmrs.a

libsickldmrs.a: src/commands.o src/print.o src/sickldmrs.o

libsickldmrs.so.1: src/commands.o src/print.o src/sickldmrs.o

clean: clean_sickldmrs

clean_sickldmrs:
	rm -rf src/*.o test/*.o *.a *~
				 
# rules
include $(CARMEN_HOME)/src/Makefile.rules
