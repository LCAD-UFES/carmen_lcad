##############################################################

INC_PATH += include
LIB_PATH += lib

##############################################################

CPP = g++
CC = gcc
CFLAGS = -Wall -O3
AR = ar
ARFLAGS = -rc

ROOT_INCLUDE = /usr/local/include
LOCALLIB = pgr-stereo-examples-libdc-2.0.2/pgrlibdcstereo/

CFLAGS += -I$(ROOT_INCLUDE)/dc1394
CFLAGS += -I$(LOCALLIB)

##############################################################

EXECUTABLE = libbee
OBJECTS = libbee.o \
		
##############################################################

.c.o:	
	$(CC) $(CFLAGS) -c -I$(INC_PATH) -o $@ $*.c
	
.cpp.o:	
	$(CPP) $(CFLAGS) -c -I$(INC_PATH) -o $@ $*.cpp

##############################################################

build: prglibdcstereo.a libbee.a

prglibdcstereo.a:
	$(MAKE) -C pgr-stereo-examples-libdc-2.0.2/pgrlibdcstereo

libbee.a: $(OBJECTS)
	$(AR) $(ARFLAGS) $(LIB_PATH)/libbee.a $(OBJECTS)
	ranlib $(LIB_PATH)/libbee.a 

test: $(EXECUTABLE)
	@echo OK
	
$(EXECUTABLE): $(OBJECTS)
	$(CPP) $(CFLAGS) -o $(BIN_PATH)/$(EXECUTABLE) $(OBJECTS) $(LIBS)
	
##############################################################

clean:
	rm -f *.o
	rm -f $(LIB_PATH)/libbee.a
	rm -f $(BIN_PATH)/$(EXECUTABLE)
	$(MAKE) clean -C pgr-stereo-examples-libdc-2.0.2/pgrlibdcstereo

##############################################################
