include ../Makefile.conf

# Module name and description
MODULE_NAME = EKF_ODOMETRY
MODULE_COMMENT = Robot Pose Ekf Estimation

LINK = g++

# Application specific include directories.
IFLAGS += -I../../sharedlib/libtf/src -I/usr/local/include/bullet/

# Required default libraries to comunicate with Carmen Core.
CXXFLAGS += -I/usr/local/include/
CFLAGS += -I/usr/local/include/
LFLAGS += -L../../sharedlib/libtf/src -lopencv_core -lopencv_features2d -lopencv_ml -lopencv_imgproc -lopencv_highgui  
LFLAGS += -lglobal -lipc -lparam_interface -lmap_interface -lvisual_odometry_interface -lneural_global_localizer_interface -lstereo_interface -lvisual_search_thin_interface -llocalize_ackerman_interface -ltf -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_system -lboost_thread-mt -lrt -lboost_signals -lorocos-bfl    
# Source code files (.c, .cpp) 
SOURCES = ekf_odometry_main.cpp ekf_odometry_interface.c ekf_odometry_core.cpp ekf_odometry_util.cpp nonlinearanalyticconditionalgaussianodo.cpp nonlinearanalyticconditionalgaussianmeasurement.cpp

# Public headers, linked to 'carmen/include/carmen/'.
PUBLIC_INCLUDES = ekf_odometry_interface.h ekf_odometry_messages.h

# Public libraries, linked to 'carmen/lib'.
PUBLIC_LIBRARIES = libekf_odometry_interface.a

# Public binaries, your application specific module laucher, copied to 'carmen/bin'.
PUBLIC_BINARIES = ekf_odometry

TARGETS = ekf_odometry
PUBLIC_LIBRARIES_SO = 

ekf_odometry: ekf_odometry_main.o ekf_odometry_core.o ekf_odometry_util.o nonlinearanalyticconditionalgaussianodo.o nonlinearanalyticconditionalgaussianmeasurement.o libekf_odometry_interface.a

libekf_odometry_interface.a: ekf_odometry_interface.o

# rules

include ../Makefile.rules
