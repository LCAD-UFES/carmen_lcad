
#Ice related 
IceE_BASEDIR	    = /opt/IceE-1.2.0
BASELIBS    = $(STLPORT_LIBS) -lpthread
LIBS        = -lIceE $(BASELIBS)
MINLIBS     = -lIceEC $(BASELIBS)
CPPFLAGS	  = -I$(IceE_BASEDIR)/include -I.
LDFLAGS     = -L$(IceE_BASEDIR)/lib
SLICE2CPPE=/opt/IceE-1.2.0/bin/slice2cppe

GUMSTIX_BUILDROOT=/home/teacher/gumstix/gumstix-buildroot/
CXX=$(GUMSTIX_BUILDROOT)/build_arm_nofpu/staging_dir/bin/arm-linux-c++
AR=$(GUMSTIX_BUILDROOT)/build_arm_nofpu/staging_dir/bin/arm-linux-ar

#Project related
SERVER=GumbotService

TARGETS=$(CLIENT) $(SERVER)

OBJS=Gumbot.o ImageIce.o

SERVER_OBJS=GumbotI.o \
						Server.o \
            capture.o \
            serial.o

SRCS    = $(OBJS:.o=.cpp) \
	      $(COBJS:.o=.cpp) \
	      $(SERVER_OBJS:.o=.cpp)


all: slice $(TARGETS)

slice:
	$(SLICE2CPPE) -I. Gumbot.ice
	$(SLICE2CPPE) -I. ImageIce.ice

$(SERVER): $(OBJS) $(SERVER_OBJS)
	rm -f $@       #Remove the object
	$(CXX) $(LDFLAGS) -o $@ $(OBJS) $(SERVER_OBJS) $(LIBS)

