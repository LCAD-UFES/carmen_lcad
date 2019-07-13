include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = SEG MAP PROGRAMS
MODULE_COMMENT = Main programs

LINK = g++ 

CXXFLAGS += -fopenmp
IFLAGS += -fopenmp

CXXFLAGS += -Wno-deprecated -Wno-write-strings -Wno-unused-result -Wno-sign-compare -std=c++11
# Just to prevent the process that generates the Makefile.depend from complaining we're not using C++11
IFLAGS += -std=c++11

IFLAGS += -I /usr/local/include/pcl-1.8 -I /usr/include/eigen3/ -I /usr/include/vtk-5.10/ `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv` -lopencv_imgcodecs
LFLAGS += -L /usr/lib/x86_64-linux-gnu/ -lboost_system
LFLAGS += -L /usr/lib -lvtkIO -lvtkFiltering -lvtkCommon -lvtkRendering -lvtkGraphics
LFLAGS += -L /usr/local/lib -lpcl_common -lpcl_io -lpcl_visualization -lglobal 
LFLAGS += -lboost_program_options -lboost_filesystem

LFLAGS += -lsegmap_types -lsegmap_datasets -lsegmap_readers -lsegmap_slam \
	-lsegmap_visualization -lsegmap_readers -lsegmap_motion_models -lsegmap_initializations \
	-lcarmen_util 

DEPS = $(CARMEN_HOME)/src/segmap/libsegmap/types/libsegmap_types.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/datasets/libsegmap_datasets.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsegmap_readers.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/motion_models/libsegmap_motion_models.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/slam/libsegmap_slam.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/visualization/libsegmap_visualization.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/readers/libsemap_readers.a \
	$(CARMEN_HOME)/src/segmap/libsegmap/initializations/libsegmap_initializations.a \
	$(CARMEN_HOME)/src/utilities/libcarmen_util/libcarmen_util.a

SOURCES = mapper.cpp localizer.cpp velodyne_calibration_save_data.cpp velodyne_calibration_compute_calibration_table.cpp

PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES = 
PUBLIC_BINARIES = 
TARGETS = mapper localizer velodyne_calibration_save_data velodyne_calibration_compute_calibration_table

# The libraries are added as dependencies to make the program be compiled when the libs are changed.
# Without adding them as a dependency, we would have to do a make clean && make whenever we change a .cpp file.
localizer: localizer.o $(DEPS) 
mapper: mapper.o $(DEPS)
velodyne_calibration_save_data: velodyne_calibration_save_data.o $(DEPS)
velodyne_calibration_compute_calibration_table: velodyne_calibration_compute_calibration_table.o $(DEPS)

include $(CARMEN_HOME)/src/Makefile.rules


