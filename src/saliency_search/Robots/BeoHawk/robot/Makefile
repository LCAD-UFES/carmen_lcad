
GUMSTIXDIR = /home/uscr/gumstix/gumstix-oe/tmp

CXX = /home/uscr/gumstix/gumstix-oe/tmp/cross/bin/arm-angstrom-linux-gnueabi-g++

CXXFLAGS = -I/home/uscr/gumstix/gumstix-oe/tmp/staging/arm-angstrom-linux-gnueabi/opt/icee/include \
	-I/home/uscr/gumstix/gumstix-oe/tmp/staging/arm-angstrom-linux-gnueabi/opt/libdc1394/include \
	-I. -I./miniIce

LIBS = -L/home/uscr/gumstix/gumstix-oe/tmp/staging/arm-angstrom-linux-gnueabi/opt/icee/lib \
	-L/home/uscr/gumstix/gumstix-oe/tmp/staging/arm-angstrom-linux-gnueabi/opt/libdc1394/lib \
	-L/home/uscr/gumstix/gumstix-oe/tmp/staging/arm-angstrom-linux-gnueabi/opt/libusb/lib	

LDFLAGS = -lIceE -lrt -ldc1394 -lusb


#OBJS_SIMPLE = test.o
#HW_SIMPLE = simpletest

OBJS_ICE = 	test-AerialRetina.o \
			SimpleRobotBrainComponent.o \
			miniIce/ImageIce.o \
			miniIce/SimpleRobotSimEvents.o \
			miniIce/IceStorm.o

HW_ICE = icetest


all: $(OBJS_ICE)
	$(CXX) -o $(HW_ICE) $(OBJS_ICE) $(LDFLAGS) $(LIBS)
	
clean:
	rm -f *.o

