include ../../Makefile.conf

CFLAGS += 
IFLAGS += 
LFLAGS += 

MODULE_NAME = glm
MODULE_COMMENT = 

SOURCES = glm.c glmimg.c glmimg_devil.c glmimg_jpg.c glmimg_png.c glmimg_sdl.c glmimg_sim.c glm_util.c
PUBLIC_INCLUDES = glm.h
PUBLIC_LIBRARIES = libglm.a  
PUBLIC_BINARIES = 

TARGETS = libglm.a

libglm.a: glm.o glmimg.o glmimg_devil.o glmimg_jpg.o glmimg_png.o glmimg_sdl.o glmimg_sim.o glm_util.o

include ../../Makefile.rules
