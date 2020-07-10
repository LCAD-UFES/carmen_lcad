include ../Makefile.conf

LINK = g++

MODULE_NAME = SLAM6D
MODULE_COMMENT = SLAM6D Based on Kinect Fusion

PCL_INC = $(wildcard /usr/local/include/pcl-*)
VTK_INC = $(wildcard /usr/local/include/vtk-5* /usr/include/vtk-5*)
IFLAGS += -I $(PCL_INC) -I $(VTK_INC)

IFLAGS += -I../../sharedlib/libkinfu \
			   -I/usr/local/cuda/include -I/home/lcad12/NVIDIA_GPU_Computing_SDK/C/common/inc
IFLAGS += -I/usr/local/cuda/include -I/usr/include/eigen3  `pkg-config --cflags opencv` -I/usr/local/cuda/lib64/
LFLAGS += -L/usr/local/lib/ -L/usr/lib64 -L../../sharedlib/libkinfu -L../../sharedlib/libsiftgpu -L/usr/local/cuda/lib64/
LFLAGS += -lglobal -lipc -lparam_interface -lkinfu -lkinect_interface -lpcl_common -lpcl_gpu_containers -lpcl_gpu_kinfu -lpcl_geometry -lpcl_io -lGL -lGLU -lglut -lcudart `pkg-config --libs opencv`

SOURCES = slam6d_main.cpp slam6d_opengl_interface.cpp slam6d_opengl_thread.cpp slam6d_interface.c kernelsVbo.cu
 
PUBLIC_INCLUDES = slam6d_messages.h slam6d_interface.h
PUBLIC_LIBRARIES = libslam6d_interface.a 
PUBLIC_BINARIES = slam6d 

TARGETS = libslam6d_interface.a slam6d 
PUBLIC_LIBRARIES_SO = libslam6d_interface.so

CXXFLAGS += -Wno-unused-parameter

ifndef NO_PYTHON
TARGETS += libslam6d_interface.so.1
endif

slam6d:	slam6d_main.o slam6d_opengl_interface.o slam6d_opengl_thread.o kernelsVbo.o libslam6d_interface.a

libslam6d_interface.a:	slam6d_interface.o

libslam6d_interface.so.1: slam6d_interface.o

				 
# rules
include ../Makefile.rules
