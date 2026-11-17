include ../Makefile.conf

MODULE_NAME = "ARGOS UNITREE CAMERA"
MODULE_COMMENT = "Camera do ARGOS (Unitree Go2) via unitree_sdk2/DDS, pela rede"

# O unitree_sdk2 e o CycloneDDS C++ exigem C++17.
CXXFLAGS += -std=c++17

# unitree_sdk2 instalado em /usr/local. Outro prefixo:
#     make UNITREE_SDK2_PREFIX=/opt/unitree_robotics
UNITREE_SDK2_PREFIX ?= /usr/local

# O segundo include e' o do CycloneDDS C++ (ddscxx): os headers gerados do IDL do
# Unitree fazem #include "dds/topic/TopicTraits.hpp", que mora em include/ddscxx/dds.
IFLAGS += -I$(UNITREE_SDK2_PREFIX)/include -I$(UNITREE_SDK2_PREFIX)/include/ddscxx
IFLAGS += `pkg-config --cflags opencv4 2>/dev/null || pkg-config --cflags opencv`

LFLAGS += -L$(UNITREE_SDK2_PREFIX)/lib

# A ordem importa: libunitree_sdk2.a e' estatica e depende de ddscxx/ddsc.
LFLAGS += -lparam_interface -lipc -lglobal -lcamera_drivers \
	  -lunitree_sdk2 -lddscxx -lddsc \
	  `pkg-config --libs opencv4 2>/dev/null || pkg-config --libs opencv` \
	  -lstdc++ -pthread -lrt

SOURCES = argos_unitree_camera_main.cpp

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = argos_unitree_camera

TARGETS = argos_unitree_camera

argos_unitree_camera: argos_unitree_camera_main.o

# rules

include ../Makefile.rules
