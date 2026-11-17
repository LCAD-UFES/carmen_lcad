include ../Makefile.conf

MODULE_NAME = "ARGOS UNITREE DRIVER"
MODULE_COMMENT = "Driver do ARGOS (Unitree Go2) via unitree_sdk2/DDS, sem CAN virtual"

# O unitree_sdk2 e o CycloneDDS C++ exigem C++17.
CXXFLAGS += -std=c++17

# unitree_sdk2 instalado em /usr/local (headers em /usr/local/include/unitree,
# libunitree_sdk2.a em /usr/local/lib). Se voce instalou em outro prefixo, aponte
# UNITREE_SDK2_PREFIX na linha de comando:
#     make UNITREE_SDK2_PREFIX=/opt/unitree_robotics
UNITREE_SDK2_PREFIX ?= /usr/local

# O segundo include e' o do CycloneDDS C++ (ddscxx): os headers gerados do IDL do
# Unitree fazem #include "dds/topic/TopicTraits.hpp", que mora em include/ddscxx/dds.
IFLAGS += -I$(UNITREE_SDK2_PREFIX)/include -I$(UNITREE_SDK2_PREFIX)/include/ddscxx
LFLAGS += -L$(UNITREE_SDK2_PREFIX)/lib

# A ordem importa: libunitree_sdk2.a e' estatica e depende de ddscxx/ddsc.
LFLAGS += -lparam_interface -lipc -lglobal -lford_escape_hybrid_interface \
	  -lunitree_sdk2 -lddscxx -lddsc \
	  -lstdc++ -pthread -lrt

SOURCES = argos_unitree_driver_main.cpp

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = argos_unitree_driver

TARGETS = argos_unitree_driver

argos_unitree_driver: argos_unitree_driver_main.o

# rules

include ../Makefile.rules
