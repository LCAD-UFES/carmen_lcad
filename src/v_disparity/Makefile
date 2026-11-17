include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

LINK = g++
CFLAGS += -Wall -fopenmp
CXXFLAGS += -fopenmp
IFLAGS += -I/usr/local/include
LFLAGS += -lglobal -lparam_interface -lipc -lGL -lGLU -lglut \
	  -lstereo_interface -lbumblebee_basic_interface \
	  `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv`

MODULE_NAME = V_DISPARITY
MODULE_COMMENT = V-Disparity Library and Viewer

SOURCES = v_disparity.cpp
PUBLIC_INCLUDES = v_disparity.h
 
ifndef NO_CUDA
SOURCES += v_disparity_GPU.cu
endif

PUBLIC_LIBRARIES = libv_disparity.a
PUBLIC_BINARIES =
MAN_PAGES =

TARGETS = libv_disparity.a

PUBLIC_LIBRARIES_SO = 

ifndef NO_PYTHON
TARGETS += 
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
SOURCES += v_disparity_view.cpp
PUBLIC_BINARIES += v_disparity_view
TARGETS += v_disparity_view
endif

ifndef NO_CUDA
libv_disparity.a: v_disparity.o  v_disparity_GPU.o
else
libv_disparity.a: v_disparity.o
endif

v_disparity_view: v_disparity_view.o libv_disparity.a

include ../Makefile.rules

