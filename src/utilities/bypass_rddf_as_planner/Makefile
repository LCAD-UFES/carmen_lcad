include ../../Makefile.conf

LFLAGS += -lglobal -lipc  -llocalize_ackerman_interface -lrobot_ackerman_interface -lbase_ackerman_interface \
	-lrddf_interface -lbehavior_selector_interface 
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

MODULE_NAME = BYPASS RDDF AS PLANNER
MODULE_COMMENT = 

SOURCES = bypass_rddf_as_planner.cpp
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
PUBLIC_BINARIES = bypass_rddf_as_planner

TARGETS = bypass_rddf_as_planner

bypass_rddf_as_planner: bypass_rddf_as_planner.o

include ../../Makefile.rules
