include $(CARMEN_HOME)/src/Makefile.conf

# Module name and description
MODULE_NAME = CAR_PANEL_GUI
MODULE_COMMENT = Graphics User Interface for car panel 

LINK = g++

CXXFLAGS += 
DEL_FILE = rm -f

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/src/car_panel_gui
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models 

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -lglobal -lGLU -lipc -lz -lparam_interface -lmap_interface -llocalize_ackerman_interface -lsimulator_ackerman_interface \
	  -lobstacle_avoider_interface -lprob_models -llaser_interface -lmapper_interface -lnavigator_ackerman_interface \
	  -lglobal_graphics_qt -lrrt_planner_interface -lGL -lglut -lGLEW -lQtCore -lpthread -lfused_odometry_interface -lobstacle_avoider_interface \
	  -lmotion_planner_interface -lrobot_ackerman_interface -lbase_ackerman_interface

# Source code files (.c, .cpp) 
SOURCES	= 	character.cpp \
		speedometer.cpp \
		steering.cpp \
		lights.cpp \
		arrow.cpp \
		accelerator.cpp \
		without_time.cpp \
		panel.cpp \
		panel_test.cpp		

OBJECTS =	character.o \
		speedometer.o \
		steering.o \
		lights.o \
		arrow.o \
		accelerator.o \
		without_time.o \
		panel.o \

TARGETS = panel_test libpanel.a pycarmen_panel

libpanel.a: $(OBJECTS)
panel_test: $(OBJECTS) panel_test.o

clean: pycarmen_panel_clean

PY_INC = -I `python -c "from distutils.sysconfig import get_python_inc; print(get_python_inc())"`
# PY_LIB = -L /usr/lib/x86_64-linux-gnu/
PY_LIB += `python3-config --libs`

pycarmen_panel:
	swig -python -c++ pycarmen_panel.i
	g++ -fPIC -c pycarmen_panel_wrap.cxx $(PY_INC) $(IFLAGS) $(CXXFLAGS) -Wno-missing-field-initializers
	g++ -shared pycarmen_panel_wrap.o libpanel.a -o _pycarmen_panel.so $(PY_LIB) $(LFLAGS) 

pycarmen_panel_clean:
	rm -rf pycarmen_panel.py pycarmen_panel_wrap.cxx *.a *.so __pycache__
	
include $(CARMEN_HOME)/src/Makefile.rules

