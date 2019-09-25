include ../../../Makefile.conf

ifndef CYGWIN
CFLAGS += -fPIC 
endif

CFLAGS += -std=gnu99

MODULE_NAME = "ORCLIB_V5"
MODULE_COMMENT = "LIbrary for communication with Orcboard version 5"

SOURCES = orc.c rawprovider.c tcpprovider.c packet.c timespec.c clog.c orc_test.c
TARGETS = liborc.a liborc.so orc_test

PUBLIC_LIBRARIES = liborc.a liborc.so

ORC_OBJS = orc.o rawprovider.o tcpprovider.o packet.o timespec.o clog.o
liborc.a: $(ORC_OBJS)

liborc.so: $(ORC_OBJS)

orc_test: orc_test.o liborc.a
#	$(CC) $(ORCTEST_OBJS) -o $@ -lpthread -lorc -L.
#	$(CC) $(ORCTEST_OBJS) -o $@ -lpthread liborc.a

include ../../../Makefile.rules
