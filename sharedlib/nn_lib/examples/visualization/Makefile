INC_DIR = ../../nn_lib
INC_CMD = -I $(INC_DIR)
LIB_CMD = `pkg-config --libs opencv`

all:
	g++ -Wall -o simple_minchinton_cell_visualization simple_minchinton_cell_visualization.cpp $(INC_CMD) $(LIB_CMD)
	g++ -Wall -o simple_log_polar_visualization simple_log_polar_visualization.cpp $(INC_CMD) $(LIB_CMD)
clean:
	rm -rf simple_minchinton_cell_visualization
	rm -rf simple_log_polar_visualization
