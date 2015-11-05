PROGRAM_NAME=helibot
#

GCC=/usr/local/bin/gumgcc
CC=/usr/local/bin/gumg++

GUMROOT = /home/teacher/CSCI-445/gumstix
CAPTURE = $(GUMROOT)/captureMjpeg

SRCS = helibot.C

LIBS              = -lpthread 
CPPFLAGS          = -O2 -I. -I$(GUMROOT)  -I$(INVT) 

CPPFLAGS += -DINT_IS_32_BITS=1 -DLONG_IS_32_BITS=1 

#LDFLAGS           = -L$(IceE_BASEDIR)/lib -DLINUX -DVERSION=\"0.2.4\"
#CFLAGS += --sysroot=/home/elazary/gumstix/gumstix-oe/tmp/cross/arm-angstrom-linux-gnueabi/

DEPENDENCIES =  $(CAPTURE)/color.o \
								$(CAPTURE)/utils.o            $(CAPTURE)/v4l2uvc.o \
								Capture.o

all: $(PROGRAM_NAME)

$(PROGRAM_NAME): $(PROGRAM_NAME).o $(DEPENDENCIES)
	@echo Compiling Application $<
	$(CC)  $(DEPENDENCIES) $(PROGRAM_NAME).o $(CFLAGS) $(CPPFLAGS) $(LDFLAGS) $(LIBS) -o $(PROGRAM_NAME)

%.o :: %.c 
	@echo Compiling C File $<
	$(GCC) -c -o $@ $< $(CPPFLAGS) -std=gnu99

%.o :: %.C 
	@echo Compiling C++ File $<
	$(CC) -c -o $@ $< $(CFLAGS) $(CPPFLAGS)

%.o :: %.cpp 
	@echo Compiling C++ File $<
	$(CC) -c -o $@ $< $(CPPFLAGS)

%.o :: %.cc 
	@echo Compiling C++ File $<
	$(CC) -c -o $@ $< $(CPPFLAGS)

install:
	@scp helibot root@192.168.2.135:


clean:
	rm $(DEPENDENCIES)
