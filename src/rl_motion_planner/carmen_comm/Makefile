
include $(CARMEN_HOME)/src/Makefile.conf

CXXFLAGS += -Wno-write-strings 
IFLAGS += 
LFLAGS += -lglobal -lipc -llocalize_ackerman_interface -lrobot_ackerman_interface

MODULE_NAME = carmen_comm
MODULE_COMMENT = Infra-structure for communicating with carmen via Python 

SOURCES = carmen_comm.cpp carmen_comm_test.cpp 
TARGETS =  libcarmen_comm_cpp.a pycarmen_comm carmen_comm_test 

carmen_comm_test: carmen_comm_test.o -lcarmen_comm_cpp
libcarmen_comm_cpp.a: carmen_comm.o
clean: pycarmen_comm_clean

PY_INC = -I `python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())"`
PY_LIB = -L `python -c "from distutils.sysconfig import get_python_lib; print(get_python_lib())"` 
PY_LIB += `python3-config --libs`

pycarmen_comm:
	swig -python -c++ carmen_comm.i
	g++ -fPIC -c carmen_comm_wrap.cxx $(PY_INC)
	g++ -shared carmen_comm_wrap.o libcarmen_comm_cpp.a -o _carmen_comm.so $(PY_LIB) $(LFLAGS)

pycarmen_comm_clean:
	rm carmen_comm.py carmen_comm_wrap.cxx

include $(CARMEN_HOME)/src/Makefile.rules
