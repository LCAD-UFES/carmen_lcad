CFLAGS = -O3 -Wno-deprecated -Wno-write-strings -Wno-unused-result -std=c++11
INC = -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LIB = -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization
LIB += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LIB += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LIB += `pkg-config --libs opencv`

all:
	g++ -o example_project_vel_to_cam example_project_vel_to_cam.cpp $(CFLAGS) $(INC) $(LIB)
	g++ -o example_oxmts2poses  example_oxmts2poses.cpp $(CFLAGS) $(INC) $(LIB)
	g++ -o example_euler2rotation example_euler2rotation.cpp $(CFLAGS) $(INC) $(LIB)
	g++ -o example_view_ufes_data example_view_ufes_data.cpp $(CFLAGS) $(INC) $(LIB)
	g++ -o example_view_kitti_as_map example_view_kitti_as_map.cpp $(CFLAGS) $(INC) $(LIB)
clean:
	rm -rf example_project_vel_to_cam example_oxmts2poses example_euler2rotation example_view_ufes_data example_view_kitti_as_map
