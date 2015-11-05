INC_DIR = ../../nn_lib
INC_CMD = -I $(INC_DIR)
LIB_CMD = `pkg-config --libs opencv`

all:
	g++ -Wall -o ocr_main ocr_main.cpp $(INC_CMD) $(LIB_CMD)
clean:
	rm -rf ocr_main
run:
	./ocr_main 
