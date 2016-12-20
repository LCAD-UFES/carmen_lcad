include ../../Makefile.conf

LINK = g++
CXXFLAGS += -std=c++0x


IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

LFLAGS += -lglobal -lipc  -llocalize_ackerman_interface -lrobot_ackerman_interface -lbase_ackerman_interface -llocalize_ackerman_core \
	-lrddf_interface -lbehavior_selector_interface 

MODULE_NAME = BYPASS RDDF AS PLANNER

SOURCES = bypass_rddf_as_planner.cpp
 
PUBLIC_BINARIES = bypass_rddf_as_planner

TARGETS = bypass_rddf_as_planner

bypass_rddf_as_planner: bypass_rddf_as_planner.o

include ../../Makefile.rules
