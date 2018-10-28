
include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++
CXXFLAGS += -Wno-write-strings -std=c++0x -Wno-sign-compare `pkg-config --cflags opencv` -Wno-missing-field-initializers 

IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ \
	 -I/usr/local/include/ -I$(CARMEN_HOME)/sharedlib/prob_models $(G2O_IFLAGS)

LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcarmodel
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libcontrol
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models	

LFLAGS += -lglobal -lipc -llocalize_ackerman_interface -lrobot_ackerman_interface \
	-lbehavior_selector_interface \
	-lobstacle_avoider_interface -lcollision_detection -lobstacle_distance_mapper_interface \
	-lparam_interface -lrobot_ackerman_interface \
	-llaser_interface `pkg-config --libs opencv` -lparam_interface \
	-lsimulator_ackerman -lcontrol -lcarmodel -lfann -lgsl -lgslcblas -lgeometry -lrddf_interface \
	-lnavigator_ackerman_interface -lmapper_interface -lsimulator_ackerman_interface \
	-lbase_ackerman_interface -lprob_models -lrotation_geometry

# TODO: Checar por que precisa adicionar essas libs de novo no final.
LFLAGS += -lgrid_mapping -lipc -lglobal -lparam_interface  \
	-lmap_interface -llocalize_ackerman_interface -lmap_server_interface -lmap_io -lmap_interface

LFLAGS += -lipc -lglobal -lparam_interface

# TODO: checar por que nao basta adicionar a flag abaixo nas CXXFLAGS
IFLAGS += -fopenmp
LFLAGS += -fopenmp

MODULE_NAME = carmen_sim
MODULE_COMMENT = Offline simulator 

PUBLIC_INCLUDES = carmen_sim.h
PUBLIC_LIBRARIES = libcarmen_sim_cpp.a 

SOURCES = carmen_sim.cpp carmen_sim_test.cpp
TARGETS = pycarmen_sim libcarmen_sim_cpp.a carmen_sim_test

carmen_sim_test: carmen_sim.o carmen_sim_test.o
libcarmen_sim_cpp.a: carmen_sim.o
clean: pycarmen_sim_clean

PY_INC = -I `python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())"`
# PY_LIB = -L `python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"` 
# PY_LIB = -L /usr/lib/x86_64-linux-gnu/
PY_LIB += `python3-config --libs`

pycarmen_sim:
	swig -python -c++ sim.i
	g++ -fPIC -c sim_wrap.cxx $(PY_INC) $(IFLAGS) $(CXXFLAGS)
	g++ -shared sim_wrap.o libcarmen_sim_cpp.a -o _sim.so $(PY_LIB) $(LFLAGS) 

pycarmen_sim_clean:
	rm -rf sim.py sim_wrap.cxx *.a *.so __pycache__

include $(CARMEN_HOME)/src/Makefile.rules


