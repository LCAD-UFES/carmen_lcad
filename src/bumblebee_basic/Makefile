include ../Makefile.conf

MODULE_NAME = BUMBLEBEE_BASIC
MODULE_COMMENT = Bumblebee2 Basic Camera Module


LINK = g++

CXXFLAGS += -fopenmp
CFLAGS +=  `pkg-config --cflags opencv` -fopenmp

IFLAGS += -I$(CARMEN_HOME)/sharedlib/libbee/include/ \
 	-I$(CARMEN_HOME)/sharedlib/libbee/pgr-stereo-examples-libdc-2.0.2/pgrlibdcstereo \
 	-I$(CARMEN_HOME)/sharedlib/libtf/src -I/usr/local/include/bullet/ \
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
	-ltf -ltriclops -lpgrlibdcstereo -lpnmutils \
	`pkg-config --libs opencv` -lflycapture \
	-lGL -lGLU -lglut -ltime_profile -lfused_odometry_interface \
	 -llocalize_ackerman_interface -lrddf_interface \
	 `pkg-config --libs opencv` \
	 -lBulletCollision -lBulletDynamics -lBulletSoftBody -lLinearMath \
	 -lboost_thread-mt -lrt -lboost_signals -lboost_system \
	 -lparam_interface -lipc -lglobal -lglobal_graphics -lbumblebee_basic_interface \
	`$(GTK_CONFIG) --libs` 
	

SOURCES = bumblebee_basic_main.cpp bumblebee_basic_interface.c \
 bumblebee_basic_simulator.c bumblebee_basic_flycap_main.cpp bumblebee_basic_view.cpp \
 bumblebee_list_camera_ids.cpp
PUBLIC_INCLUDES = bumblebee_basic_messages.h bumblebee_basic_interface.h
PUBLIC_LIBRARIES = libbumblebee_basic_interface.a 
PUBLIC_BINARIES = bumblebee_basic bumblebee_basic_simulator bumblebee_basic_flycap
TARGETS = libbee libbumblebee_basic_interface.a bumblebee_basic bumblebee_basic_simulator bumblebee_basic_flycap
PUBLIC_LIBRARIES_SO = libbumblebee_basic_interface.so
PUBLIC_BINARIES += bumblebee_basic_view bumblebee_list_camera_ids
TARGETS += bumblebee_basic_view bumblebee_list_camera_ids

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
# rules

bumblebee_list_camera_ids: bumblebee_list_camera_ids.o

include ../Makefile.rules
