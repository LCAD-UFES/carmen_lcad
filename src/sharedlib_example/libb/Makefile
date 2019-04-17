include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SHAREDLIB EXAMPLE LIBB
MODULE_COMMENT = Sharedlib example libb

LFLAGS += -la

SOURCES = b.cpp
PUBLIC_INCLUDES = b.h
PUBLIC_LIBRARIES = libb.so

TARGETS = libb.so

libb.so: b.o

include $(CARMEN_HOME)/src/Makefile.rules

