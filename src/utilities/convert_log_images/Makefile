include ../../Makefile.conf

CXXFLAGS += -std=c++11
IFLAGS += `pkg-config --cflags opencv`
LFLAGS += `pkg-config --libs opencv`

MODULE_NAME = CONVERT LOG IMAGES
MODULE_COMMENT = 

SOURCES = to_png_old_log.cpp to_png_new_log.cpp
PUBLIC_INCLUDES = 
PUBLIC_LIBRARIES =  
PUBLIC_BINARIES = to_png_old_log to_png_new_log

TARGETS = to_png_old_log to_png_new_log

to_png_old_log: to_png_old_log.o 
to_png_new_log: to_png_new_log.o

include ../../Makefile.rules
