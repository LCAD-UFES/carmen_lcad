include ../Makefile.conf

MODULE_NAME = ipc_watcher
MODULE_COMMENT = "Ipc Watcher"

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libcontrol -Wno-switch
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol

IFLAGS += -I/opt/ros/foxy/include
LFLAGS += -L/opt/ros/foxy/lib
IFLAGS += $(shell pkg-config --cflags gtkmm-3.0)
LFLAGS += $(shell pkg-config --libs gtkmm-3.0)

LFLAGS += -lglobal -lipc -lparam_interface -pthread 

CXXFLAGS += -std=c++17 

SOURCES = ipc_watcher.cpp ipc_watcher_usertime_logger.cpp
TARGETS = ipc_watcher libipc_watcher.a 
PUBLIC_BINARIES = ipc_watcher
PUBLIC_INCLUDES = ipc_watcher.h ipc_watcher_usertime_logger.h
PUBLIC_LIBRARIES = libipc_watcher.a 

ipc_watcher: ipc_watcher.o  
libipc_watcher.a: ipc_watcher_usertime_logger.o 

include ../Makefile.rules
