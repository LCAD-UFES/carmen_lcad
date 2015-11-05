
#Ice related 
IceE_BASEDIR	    = /opt/IceE-1.2.0
BASELIBS    = $(STLPORT_LIBS) -lpthread
LIBS        = -lIceE $(BASELIBS)
MINLIBS     = -lIceEC $(BASELIBS)
CPPFLAGS	  = -I$(IceE_BASEDIR)/include -I. -O2 -DUSEMJPEG=1
LDFLAGS     = -L$(IceE_BASEDIR)/lib 
SLICE2CPPE=/opt/IceE-1.2.0/bin/slice2cppe

CXX=/lab/ilab21/gumstix/gumstix-oe/tmp/cross/arm-angstrom-linux-gnueabi/bin/g++
CC=/lab/ilab21/gumstix/gumstix-oe/tmp/cross/arm-angstrom-linux-gnueabi/bin/gcc

#Project related
SERVER=iRobotService

TARGETS=$(CLIENT) $(SERVER)

OBJS=IRobot.o ImageIce.o

SERVER_OBJS= captureMjpeg/color.o \
						captureMjpeg/utils.o \
						captureMjpeg/v4l2uvc.o  \
						IRobotI.o \
						Server.o \
            capture.o \
            serial.o

SRCS    = $(OBJS:.o=.cpp) \
	      $(COBJS:.o=.cpp) \
	      $(SERVER_OBJS:.o=.cpp)


all: slice $(TARGETS)

%.o :: %.c
	@echo Compiling C File $<
	$(CC) -c -o $@ $< 


slice:
	$(SLICE2CPPE) -DIROBOT_GUMSTIX -I. IRobot.ice 
	$(SLICE2CPPE) -DIROBOT_GUMSTIX -I. ImageIce.ice

$(SERVER): $(OBJS) $(SERVER_OBJS)
	rm -f $@       #Remove the object
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) $(SERVER_OBJS) $(LIBS)

