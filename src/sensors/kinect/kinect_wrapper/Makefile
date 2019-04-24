CFLAGS=-g -Wall -fPIC
INC =  -I. -I/usr/include -I/usr/local/include -I/usr/local/include/libfreenect -I/usr/local/include/libusb-1.0
LIBS = -L. -L/usr/local/lib64 -L/usr/local/lib -L/usr/lib/x86_64-linux-gnu/ -lfreenect -lGL -lGLU -lglut -lpthread
LIB_OBJ = kinect_wrapper.o kinect_device.o
PRG_OBJ = kinect_glmain.o kinect_glview.o
LIBRARY = kinect_wrapper.a
PROGRAM = kinect_glview

ifdef CARMEN_DEBUG
#CFLAGS += -DDEBUG
endif

all:$(LIBRARY) $(PROGRAM) copy

$(LIBRARY): $(LIB_OBJ)
	ar cr $(LIBRARY) $(LIB_OBJ)

$(PROGRAM): $(PRG_OBJ) $(LIB_OBJ)
	g++ $(PRG_OBJ) $(LIB_OBJ) -o $(PROGRAM) $(LIBS)

kinect_wrapper.o: kinect_wrapper.c kinect_wrapper.h
	g++ $(CFLAGS) $(INC) $(LIBS) -c $<

kinect_device.o: kinect_device.cpp kinect_device.hpp
	g++ $(CFLAGS) $(INC) $(LIBS) -c $<

kinect_glview.o: kinect_glview.cpp kinect_glview.hpp
	g++ $(CFLAGS) $(INC) $(LIBS) -c $<

kinect_glmain.o: kinect_glmain.c
	g++ $(CFLAGS) $(INC) $(LIBS) -lkinect_wrapper -c $<

copy:
	cp $(LIBRARY) ..
	
clean:
	rm -rf *.o $(LIBRARY) $(PROGRAM)

