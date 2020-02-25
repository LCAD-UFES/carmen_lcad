include ../Makefile.conf

MODULE_NAME = Virtual Scan 2
MODULE_COMMENT = Abstract polar 2D representation of 2D or 3D range scanner data

CC = cc -std=c++0x
CFLAGS += -Wall -std=c++0x
CXXFLAGS += -Wall -std=c++0x
LINK = g++

IFLAGS += \
	-I/usr/include/epik/mtt -I/usr/include/epik/tracker -I/usr/include/epik/utils -I/usr/include/epik/pipeline

LFLAGS += \
	-lgrid_mapping -lmap_io -lmap_interface -lmapper_interface -lparam_interface -lipc -lglobal -lmap_server_interface \
	-lmoving_objects_interface -lvelodyne_interface -lvelodyne_transforms -ltf -lboost_thread-mt -lboost_signals \
	-lboost_system -ldbscan -llocalize_ackerman_interface -lrddf_interface -lcollision_detection -lviso  \
	-lmtt -lopencv_highgui -lopencv_core -lopencv_imgproc

SOURCES = \
	virtual_scan_main.cpp \
	virtual_scan.cpp \
	virtual_scan_interface.cpp \
	virtual_scan_tracker.cpp

PUBLIC_INCLUDES = \
	virtual_scan_interface.h \
	virtual_scan_messages.h \
	virtual_scan_tracker.h

PUBLIC_LIBRARIES = \
	libvirtual_scan_interface.a

PUBLIC_BINARIES = \
	virtual_scan2
	
TARGETS = \
	libvirtual_scan_interface.a libclustering \
	virtual_scan2
	
# If you set graphics support in configure.
ifndef NO_GRAPHICS
#Carmen Global Graphic Library and Gtk+2 Graphics Libraries
IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0`
LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0`
endif

libclustering:
	$(MAKE) -C $(CARMEN_HOME)/sharedlib/libclustering
	
clean: clean_libclustering

clean_libclustering:
	$(MAKE) clean -C $(CARMEN_HOME)/sharedlib/libclustering

libvirtual_scan_interface.a: \
	virtual_scan_interface.o

virtual_scan2: \
	virtual_scan_main.o \
	virtual_scan.o \
	virtual_scan_interface.o \
	virtual_scan_tracker.o

include ../Makefile.rules
