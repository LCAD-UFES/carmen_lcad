
# Migração Ubuntu 26.04: faltavam os -I do OpenCV (headers agora em /usr/include/opencv4).
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`
CFLAGS = -Wno-deprecated
INC = -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`
LIB = -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization -lpcl_filters
LIB += -L /usr/lib/x86_64-linux-gnu/
LIB += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LIB += `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`

all:
	g++ -o point_annotation_tool point_annotation_tool.cpp $(CFLAGS) $(INC) $(LIB)
clean:
	rm -rf point_annotation_tool
