include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEGMAP VISUALIZATION AND ANALYSIS
MODULE_COMMENT = Visualization and Analysis Programs

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

# g2o
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/
LFLAGS += -L/usr/local/lib -lcholmod -lg2o_core -lg2o_types_slam2d -lg2o_solver_cholmod -lg2o_stuff -lboost_program_options -lutil

SOURCES = compute_performance_statistics.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 
TARGETS = compute_performance_statistics

compute_performance_statistics: compute_performance_statistics.o

include $(CARMEN_HOME)/src/Makefile.rules
