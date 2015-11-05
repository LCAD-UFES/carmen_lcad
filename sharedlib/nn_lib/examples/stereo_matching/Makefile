INC_DIR = ../../nn_lib
INC_CMD = -I $(INC_DIR)
LIB_CMD = `pkg-config --libs opencv`

all:
	g++ -Wall -O3 -o neural_stereo_matching neural_stereo_matching.cpp $(INC_CMD) $(LIB_CMD)
clean:
	rm -rf neural_stereo_matching
run:
	./neural_stereo_matching ../../data/images/tsukuba_left_image.bmp ../../data/images/tsukuba_right_image.bmp stereo.bmp
