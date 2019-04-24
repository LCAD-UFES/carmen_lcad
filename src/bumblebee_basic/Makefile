include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = BUMBLEBEE_BASIC
MODULE_COMMENT = Bumblebee2 Basic Camera Module

LINK = g++

CXXFLAGS += -fopenmp
CFLAGS +=  `pkg-config --cflags opencv` -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libbee/include/ \
 	-I$(CARMEN_HOME)/sharedlib/libbee/pgr-stereo-examples-libdc-2.0.2/pgrlibdcstereo \
 	`$(GTK_CONFIG) --cflags` 

ifeq ($(PROCESSOR),x86_64)
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libbee/triclops_64/include
else
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libbee/triclops_32/include
endif

ifeq ($(PROCESSOR),x86_64)
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libbee/triclops_64/lib
else
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libbee/triclops_32/lib
endif
LFLAGS += -L$(CARMEN_HOME)/sharedlib/libtf/src \
	-L$(CARMEN_HOME)/sharedlib/libbee/pgr-stereo-examples-libdc-2.0.2/pgrlibdcstereo \
	-L$(CARMEN_HOME)/sharedlib/libbee/lib -lparam_interface -lipc -lglobal -lbee -ldc1394 \
	-ltriclops -lpgrlibdcstereo -lpnmutils \
	`pkg-config --libs opencv` -lflycapture \
	-lGL -lGLU -lglut -ltime_profile \
	 `pkg-config --libs opencv` \
	 -lparam_interface -lipc -lglobal -lglobal_graphics -lbumblebee_basic_interface \
	`$(GTK_CONFIG) --libs` 
	

SOURCES = bumblebee_basic_main.cpp bumblebee_basic_interface.c bumblebee_basic_simulator.c bumblebee_basic_flycap_main.cpp bumblebee_basic_view.cpp 
PUBLIC_INCLUDES = bumblebee_basic_messages.h bumblebee_basic_interface.h
PUBLIC_LIBRARIES = libbumblebee_basic_interface.a 
PUBLIC_BINARIES = bumblebee_basic bumblebee_basic_simulator bumblebee_basic_flycap
TARGETS = libbee libbumblebee_basic_interface.a bumblebee_basic bumblebee_basic_simulator bumblebee_basic_flycap
PUBLIC_LIBRARIES_SO = libbumblebee_basic_interface.so
PUBLIC_BINARIES += bumblebee_basic_view 
TARGETS += bumblebee_basic_view 

libbee:
	$(MAKE) build -C $(CARMEN_HOME)/sharedlib/libbee

libbumblebee_basic_interface.a:		bumblebee_basic_interface.o

libbumblebee_basic_interface.so.1:	bumblebee_basic_interface.o

bumblebee_basic_flycap: bumblebee_basic_flycap_main.o $(CARMEN_HOME)/sharedlib/libbee/lib/libbee.a libbumblebee_basic_interface.a

bumblebee_basic:			bumblebee_basic_main.o $(CARMEN_HOME)/sharedlib/libbee/lib/libbee.a libbumblebee_basic_interface.a

bumblebee_basic_view:			bumblebee_basic_view.o libbumblebee_basic_interface.a

bumblebee_basic_simulator:		bumblebee_basic_simulator.o

clean: clean_libbee

clean_libbee:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libbee

include $(CARMEN_HOME)/src/Makefile.rules
