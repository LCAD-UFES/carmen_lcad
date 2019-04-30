include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEG_MAP_EXAMPLES
MODULE_COMMENT = Examples from segmap

LINK = g++ 

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11 

# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11
IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LFLAGS += `pkg-config --libs opencv`
LFLAGS += -lcarmen_util -lsegmap_utils

SOURCES = carmen_param_file_test.cpp
	#example_project_vel_to_cam_ufes.cpp example_project_vel_to_cam.cpp \
	#example_oxmts2poses.cpp example_euler2rotation_and_transforms.cpp \
	#example_view_ufes_data.cpp example_view_kitti_as_map.cpp example_view_xsens.cpp \
	#test_mapper_mesh.cpp 

TARGETS = carmen_param_file_test
	#example_project_vel_to_cam_ufes example_project_vel_to_cam \
	#example_oxmts2poses example_euler2rotation_and_transforms \
	#example_view_ufes_data example_view_kitti_as_map example_view_xsens test_mapper_mesh

example_project_vel_to_cam_ufes: example_project_vel_to_cam_ufes.o
example_project_vel_to_cam: example_project_vel_to_cam.o
example_oxmts2poses: example_oxmts2poses.o
example_euler2rotation_and_transforms: example_euler2rotation_and_transforms.o 
example_view_ufes_data: example_view_ufes_data.o
example_view_kitti_as_map: example_view_kitti_as_map.o
example_view_xsens: example_view_xsens.o
test_mapper_mesh: test_mapper_mesh.o 
carmen_param_file_test: carmen_param_file_test.o

include $(CARMEN_HOME)/src/Makefile.rules

