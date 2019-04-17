
include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SHAREDLIB EXAMPLE MAIN
MODULE_COMMENT = Sharedlib example main

SUBDIRS += liba libb

# Note that even libb.so depending on liba.so, we do not
# need to explicitly add it here.
LFLAGS += -lb
SOURCES = main.cpp
TARGETS = main

main: main.o

include $(CARMEN_HOME)/src/Makefile.rules

