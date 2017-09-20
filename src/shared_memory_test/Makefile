include ../Makefile.conf

MODULE_NAME = "SHARED_MEMORY_TEST"
MODULE_COMMENT = "SHARED_MEMORY_TEST Module"

LINK = g++
#CXXFLAGS += -fpermissive

LFLAGS += -lshared_memory_test_interface -lparam_interface -lcarmenserial -lipc -lglobal -lmovement
LFLAGS += -lgrid_mapping -lmapper -lmap_io -lmap_util -lmap_interface 
LFLAGS += -lmapper_interface -lrt -lpthread

IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

SOURCES = shared_memory_test_client.cpp shared_memory_test_server.cpp shared_memory_test_interface.cpp

PUBLIC_INCLUDES = shared_memory_test_messages.h shared_memory_test_interface.h
PUBLIC_LIBRARIES =  libshared_memory_test_interface.a
PUBLIC_LIBRARIES_SO = shared_memory_test_interface.so
PUBLIC_BINARIES = shared_memory_test_server shared_memory_test_client
TARGETS = libshared_memory_test_interface.a shared_memory_test_server shared_memory_test_client

ifndef NO_PYTHON
TARGETS += libshared_memory_test_interface.so.1
endif


libshared_memory_test_interface.a:	shared_memory_test_interface.o

libshared_memory_test_interface.so.1:	shared_memory_test_interface.o


shared_memory_test_server: shared_memory_test_server.o libshared_memory_test_interface.a 
shared_memory_test_client: shared_memory_test_client.o libshared_memory_test_interface.a 

include ../Makefile.rules
