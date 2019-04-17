include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEG MAP UTIL TESTS
MODULE_COMMENT = Tests programs of the libsegmap_util library 

CXXFLAGS += -Wno-sign-compare -Wno-deprecated -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ \
	-I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv` -lopencv_imgcodecs
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization -lglobal -lboost_program_options

LFLAGS += -lsegmap_types -lsegmap_datasets -lsegmap_readers -lsegmap_slam \
	-lsegmap_visualization -lsegmap_readers -lsegmap_motion_models \
	-lutil

DEPS = $(CARMEN_HOME)/src/segmap/libsegmap/types/libsegmap_types.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/datasets/libsegmap_datasets.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsegmap_readers.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/motion_models/libsegmap_motion_models.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/slam/libsegmap_slam.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/visualization/libsegmap_visualization.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsemap_readers.a \
	$(CARMEN_HOME)/src/segmap/util/libutil.a

SOURCES = test_segmap_dataset.cpp test_command_line.cpp
TARGETS = test_command_line

test_segmap_dataset: test_segmap_dataset.o $(DEPS)
test_command_line: test_command_line.o $(DEPS)

include $(CARMEN_HOME)/src/Makefile.rules
