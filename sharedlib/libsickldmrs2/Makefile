include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SICK LD-MRS Library
MODULE_COMMENT = SICK LD-MRS Library

LINK = g++

LFLAGS += -lrt
IFLAGS += -Iinclude

SOURCES = src/sickldmrs.cpp src/vpTime.cpp test/sickldmrstest.cpp
TARGETS = sickldmrstest libsickldmrs.a 

ifndef NO_PYTHON
TARGETS += libsickldmrs.so.1
endif

sickldmrstest: test/sickldmrstest.o libsickldmrs.a

libsickldmrs.a: src/sickldmrs.o src/vpTime.o

libsickldmrs.so.1: src/sickldmrs.o src/vpTime.o

clean: clean_sickldmrs

clean_sickldmrs:
	rm -rf src/*.o test/*.o *.a *~
				 
# rules
include $(CARMEN_HOME)/src/Makefile.rules
