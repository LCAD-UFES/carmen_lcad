include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = LIB OUSTER CONN
MODULE_COMMENT = Library with functions to communicate with Ouster OS-1 64 LiDAR

LINK = g++ 
CXXFLAGS += -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11 

IFLAGS += -I /usr/include/jsoncpp/

SOURCES = os1_util.cpp os1.cpp
TARGETS = libouster_conn.a

libouster_conn.a: os1_util.o os1.o

include $(CARMEN_HOME)/src/Makefile.rules
