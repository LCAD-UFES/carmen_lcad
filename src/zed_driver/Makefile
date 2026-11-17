
include ../Makefile.conf

# Migração Ubuntu 26.04: o módulo linkava contra o OpenCV mas nunca pedia os -I dele.
# No Ubuntu 20.04 os headers ficavam em /usr/include/opencv2 (path padrão); o pacote do
# 26.04 põe tudo em /usr/include/opencv4, que exige -I explícito.
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

MODULE_NAME = ZED_CAMERA
MODULE_COMMENT = Zed Camera Driver

LINK = g++
CXXFLAGS = -std=c++11 -Wno-cpp

IFLAGS += -std=c++11 -I/usr/local/zed/include -I/usr/local/cuda/include

LFLAGS += -lparam_interface -lglobal -lipc -lbumblebee_basic_interface `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` -L/usr/local/zed/lib -lsl_zed -L/usr/local/cuda/lib64

SOURCES = zed_driver_main.cpp

PUBLIC_BINARIES = zed_driver

TARGETS = zed_driver

zed_driver: zed_driver_main.o

include ../Makefile.rules

