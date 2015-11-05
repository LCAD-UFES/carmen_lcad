INC_DIR = ../../nn_lib
INC_CMD = -I $(INC_DIR)
LIB_CMD = `pkg-config --libs opencv`

all:
	g++ -Wall -o simple_face_parts_recognition simple_face_parts_recognition.cpp $(INC_CMD) $(LIB_CMD)
clean:
	rm -rf simple_face_parts_recognition
run:
	./simple_face_parts_recognition
