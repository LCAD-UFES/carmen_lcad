include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = PROMETHEUS_DRIVERS
MODULE_COMMENT = Driver for controlling and send odometry data prometheus

LINK = g++

CXXFLAGS += -std=c++17 -Wno-deprecated -Wno-deprecated-declarations 
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++17

# libraries required by the driver 
LFLAGS += -pthread -lunitree_sdk2 -lddsc -lddscxx
IFLAGS += -I/usr/local/include -I/usr/local/include/ddscxx # Unitree headers 



SOURCES = prometheus_can2move.cpp prometheus_odom2can.cpp

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = prometheus_can2move prometheus_odom2can
TARGETS = prometheus_can2move prometheus_odom2can

prometheus_can2move:prometheus_can2move.o  
	$(CXX) $(CXXFLAGS) $(IFLAGS) $^ -o $@ $(LFLAGS)

prometheus_odom2can:prometheus_can2move.o  
	$(CXX) $(CXXFLAGS) $(IFLAGS) $^ -o $@ $(LFLAGS)

include $(CARMEN_HOME)/src/Makefile.rules
