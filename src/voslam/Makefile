include ../Makefile.conf

LINK = g++

MODULE_NAME = VOSLAM 
MODULE_COMMENT = A Visual Odometry Based 6D SLAM

IFLAGS += -I../../sharedlib/libtf/src -I../../sharedlib/libviso2.3/src -I/usr/local/include/bullet/ -I/usr/local/cuda/include -I/home/lcad/NVIDIA_GPU_Computing_SDK/C/common/inc `pkg-config --cflags opencv`
		  
PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I/usr/include/eigen3 -I $(PCL_INC) -I $(VTK_INC)


LFLAGS += -L/usr/lib64 -L../../sharedlib/libtf/src -L../../sharedlib/libviso2.3/src -L/usr/local/cuda/lib64/ -L/usr/local/lib/
LFLAGS += -lglobal -lipc -lparam_interface -lstereo_interface -lvisual_odometry_interface -lGL -lGLU -lglut `pkg-config --libs opencv` \
          -ltf -lviso -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath -lboost_system -lboost_thread-mt -lrt -lboost_signals -lboost_system
LFLAGS += -lvtkCommon -lvtkFiltering -lvtkRendering -lpcl_common -lpcl_surface -lpcl_io -lpcl_registration -lpcl_kdtree -lpcl_features -lpcl_search -lpcl_octree -lpcl_sample_consensus -lpcl_filters -lpcl_visualization 
LFLAGS += -lfused_odometry_interface -llocalize_ackerman_interface



SOURCES = voslam_main.cpp voslam_interface.c voslam_opengl_camera.cpp voslam_opengl_interface.cpp voslam_opengl_thread.cpp \
		  voslam_messages_interpolation.cpp voslam_generalized_icp.cpp voslam_keyframes.cpp voslam_loop_detector.cpp voslam_rigid_transformation.cpp voslam_util.cpp
		  
		  
ifndef NO_CUDA
SOURCES += voslam_kernels.cu
LFLAGS  += -lcudart
endif
 
PUBLIC_INCLUDES = voslam_messages.h voslam_interface.h
PUBLIC_LIBRARIES = libvoslam_interface.a 
PUBLIC_BINARIES = voslam

TARGETS = libvoslam_interface.a voslam 
PUBLIC_LIBRARIES_SO = libvoslam_interface.so

CXXFLAGS += -Wno-unused-parameter -Wno-deprecated -Wno-unknown-pragmas -Wno-sign-compare

ifndef NO_PYTHON
TARGETS += libsvoslam_interface.so.1
endif

ifndef NO_CUDA
voslam:	voslam_main.o libvoslam_interface.a voslam_kernels.o voslam_opengl_camera.o voslam_opengl_interface.o voslam_opengl_thread.o voslam_messages_interpolation.o voslam_generalized_icp.o voslam_keyframes.o voslam_loop_detector.o voslam_rigid_transformation.o voslam_util.o
else
voslam:	voslam_main.o libvoslam_interface.a voslam_opengl_camera.o voslam_opengl_interface.o voslam_opengl_thread.o voslam_messages_interpolation.o voslam_generalized_icp.o voslam_keyframes.o voslam_loop_detector.o voslam_rigid_transformation.o voslam_util.o
endif

libvoslam_interface.a: voslam_interface.o

libvoslam_interface.so.1: voslam_interface.o

				 
# rules
include ../Makefile.rules
