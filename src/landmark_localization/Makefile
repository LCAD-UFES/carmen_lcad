include ../Makefile.conf

# Module name and description
MODULE_NAME = LANDMARK_LOCALIZATION
MODULE_COMMENT = Landmark EKF Localization based on Visual Features 

LINK = g++

# Application specific include directories.
IFLAGS += -I../../sharedlib/libtf/src -I../../sharedlib/prob_models -I/usr/include/eigen3 -I../../sharedlib/libviso2.3/src -I/usr/local/include/bullet/

# Required default libraries to comunicate with Carmen Core.
LFLAGS += -L../../sharedlib/libtf/src -L../../sharedlib/libviso2.3/src
LFLAGS += -lparam_interface -lipc -lglobal -lvisual_odometry_interface -lviso -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath
CFLAGS += -Wall
# Source code files (.c, .cpp) 
SOURCES = landmark_localization_main.cpp landmark_localization_ekf.cpp landmark_localization_motion_model_odometry_ekf.cpp landmark_localization_measurement_model_ekf.cpp landmark_localization_motion_model_odometry.cpp landmark_localization_interface.c

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = landmark_localization_messages.h landmark_localization_interface.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = liblandmark_localization_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = landmark_localization

TARGETS = liblandmark_localization_interface.a landmark_localization
PUBLIC_LIBRARIES_SO = liblandmark_localization_interface.so

ifndef NO_PYTHON
TARGETS += liblandmark_localization_interface.so.1
endif

# If you set graphics support in configure (only for GTK).
ifndef NO_GRAPHICS
FLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += `pkg-config --libs gtk+-2.0`
#Source code file (.c)
TARGETS += 
SOURCES += 
endif

liblandmark_localization_interface.a:	landmark_localization_interface.o

liblandmark_localization_interface.so.1:	landmark_localization_interface.o

landmark_localization:	landmark_localization_main.o landmark_localization_ekf.o landmark_localization_motion_model_odometry_ekf.o landmark_localization_measurement_model_ekf.o landmark_localization_motion_model_odometry.o liblandmark_localization_interface.a 

# rules

include ../Makefile.rules
