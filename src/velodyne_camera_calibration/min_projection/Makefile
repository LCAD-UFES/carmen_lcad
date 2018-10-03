CFLAGS = -Wno-deprecated
INC = -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LIB = -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization
LIB += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LIB += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LIB += `pkg-config --libs opencv`

all:
	g++ -o viewers viewers.cpp $(CFLAGS) $(INC) $(LIB)

