include ../Makefile.conf

LINK = g++

MODULE_NAME = ll_mapper 
MODULE_COMMENT = Life Long Mapper

IFLAGS += -I../../sharedlib/libtf/src -I../../sharedlib/libviso2.3/src -I/usr/local/include/bullet/ -I/usr/local/cuda/include `pkg-config --cflags opencv`
		  
IFLAGS += -I/usr/local/include/pcl-1.8/  -I/usr/include/vtk-6.3/

IFLAGS += -I/usr/include/eigen3

LFLAGS += -lgps_nmea_interface -lxsens_interface -lgtsam -lboost_system -lboost_filesystem  -lvelodyne_interface
LFLAGS += -lglobal -lipc -lparam_interface -lstereo_interface -lvisual_odometry_interface -lGL -lGLU -lglut `pkg-config --libs opencv` -ltbb
LFLAGS += -L/usr/lib64 -L../../sharedlib/libtf/src -L../../sharedlib/libviso2.3/src -L/usr/local/cuda/lib64/ -L/usr/local/lib/
LFLAGS += -ltf -lviso -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath  -lboost_thread-mt -lrt -lboost_signals 
LFLAGS += -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization  
LFLAGS += -lfused_odometry_interface -llocalize_ackerman_interface -lm -lpthread
LFLAGS += -lceres -lglog -lgflags  -lcudart -lcublas -lcholmod -lccolamd -lcamd -lcolamd -lamd -llapack -lcxsparse -lrt -lcusolver 


SOURCES = laserPosegraphOptimization.cpp Scancontext.cpp LTslam.cpp RosParamServer.cpp Scancontext_slam.cpp Session.cpp utility.cpp scanRegistration.cpp laserOdometry.cpp laserMapping.cpp
		  

 
PUBLIC_INCLUDES = common.h KDTreeVectorOfVectorsAdaptor.h nanoflann.hpp tic_toc.h BetweenFactorWithAnchoring.h tictoc.h
PUBLIC_BINARIES = lltslam laserPosegraphOptimization

TARGETS = laserPosegraphOptimization #ltslam
#PUBLIC_LIBRARIES_SO = libvoslam_interface.so

#CXXFLAGS += -Wno-unused-parameter -Wno-deprecated -Wno-unknown-pragmas -Wno-sign-compare


laserPosegraphOptimization: scanRegistration.o laserOdometry.o laserMapping.o Scancontext.o  laserPosegraphOptimization.o   
ltslam: LTslam.o RosParamServer.o Scancontext_slam.o Session.o utility.o
# rules
include ../Makefile.rules
