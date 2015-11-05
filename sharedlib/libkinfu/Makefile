include $(CARMEN_HOME)/src/Makefile.conf

LINK = g++

MODULE_NAME = LIBKINFU
MODULE_COMMENT = Kinect Fusion Library

IFLAGS += -I/home/lcad/tplib/pcl/build/include -I/home/lcad/tplib/pcl/trunk/common/include -I/home/lcad/tplib/pcl/trunk/gpu/containers/include/ \
       	  -I/home/lcad/tplib/pcl/trunk/gpu/utils/include -I/home/lcad/tplib/pcl/trunk/gpu/kinfu/include/ -I/home/lcad/tplib/pcl/trunk/gpu/kinfu/tools -I/home/lcad/tplib/pcl/trunk/gpu/kinfu/src -I/home/lcad/tplib/pcl/trunk/geometry/include  \
	  -I/home/lcad/tplib/pcl/trunk/io/include -I/usr/local/cuda/include -I/home/lcad/NVIDIA_GPU_Computing_SDK/C/common/inc
IFLAGS += -I/usr/local/cuda/include -I/usr/include/eigen3  `pkg-config --cflags opencv` -I/usr/local/cuda/lib64/
LFLAGS += -L/home/lcad/tplib/pcl/build/lib -L/usr/lib64 -L/usr/local/cuda/lib64/
LFLAGS += -lpcl_common -lpcl_gpu_containers -lpcl_gpu_kinfu -lpcl_geometry -lpcl_io -lGL -lGLU -lglut -lcudart `pkg-config --libs opencv`

SOURCES = kinfu_wrapper.cpp
 
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = libkinfu.a 
PUBLIC_BINARIES = 

TARGETS = libkinfu.a  
PUBLIC_LIBRARIES_SO = libkinfu.so

CXXFLAGS += -Wno-unused-parameter

ifndef NO_PYTHON
TARGETS += libkinfu.so.1
endif

libkinfu.a:	kinfu_wrapper.o

libkinfu.so.1:  kinfu_wrapper.o

				 
# rules
include $(CARMEN_HOME)/src/Makefile.rules
