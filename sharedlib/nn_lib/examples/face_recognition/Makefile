INC_DIR = ../../nn_lib
INC_CMD = -I $(INC_DIR)
LIB_CMD = `pkg-config --libs opencv`

all:
	g++ -Wall -o face_recognition face_recognition.cpp $(INC_CMD) $(LIB_CMD)
clean:
	rm -rf face_recognition
run:
	./face_recognition train-data.txt
