include $(CARMEN_HOME)/src/Makefile.conf

MODULE_NAME = CONVERT LOG IMAGES
MODULE_COMMENT = Convert log images from raw format to png

CXXFLAGS += -std=c++11
IFLAGS += `pkg-config --cflags opencv`

LFLAGS += `pkg-config --libs opencv`

SOURCES = to_png_old_log.cpp to_png_new_log.cpp

PUBLIC_BINARIES = to_png_old_log to_png_new_log

TARGETS = to_png_old_log to_png_new_log
 
to_png_old_log: to_png_old_log.o

to_png_new_log: to_png_new_log.o

include $(CARMEN_HOME)/src/Makefile.rules
