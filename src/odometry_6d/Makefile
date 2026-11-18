include ../Makefile.conf

MODULE_NAME = "ODOMETRY 6D"
MODULE_COMMENT = "Odometria 6D por velocidade (linear, angular e aceleracao angular) e o publicador do Unitree Go2"

CXXFLAGS += -std=c++17

SOURCES = odometry_6d_interface.c

PUBLIC_INCLUDES = odometry_6d_messages.h odometry_6d_interface.h
PUBLIC_LIBRARIES = libodometry_6d_interface.a
TARGETS = libodometry_6d_interface.a

libodometry_6d_interface.a: odometry_6d_interface.o

# O publicador do Go2 so' compila onde o unitree_sdk2 existe (a Jetson do ARGOS). Em qualquer
# outra maquina o modulo gera so' a interface, que e' o que o localize_ackerman precisa.
#
# Usa o unitree_sdk2 ORIGINAL (~/unitree_sdk2: headers, libunitree_sdk2.a e o CycloneDDS dele em
# thirdparty/). A copia de /usr/local no ARGOS tem uma libddsc compilada com memoria compartilhada
# (DDS_HAS_SHM): os headers dela nao compilam o Odometry_ e a lib derruba o SDK em tempo de execucao.
# RPATH (--disable-new-dtags), e nao RUNPATH: vence o LD_LIBRARY_PATH do .bashrc do ARGOS, que
# aponta para outras libddsc. Caminhos completos porque o Makefile.conf poe -L/usr/local/lib antes.
UNITREE_SDK2_HOME ?= $(HOME)/unitree_sdk2
UNITREE_ARCH := $(shell uname -m)

ifneq ($(wildcard $(UNITREE_SDK2_HOME)/include/unitree/idl/ros2/Odometry_.hpp),)
IFLAGS += -I$(UNITREE_SDK2_HOME)/include -I$(UNITREE_SDK2_HOME)/thirdparty/include \
	  -I$(UNITREE_SDK2_HOME)/thirdparty/include/ddscxx
LFLAGS += -Wl,--disable-new-dtags -Wl,-rpath,$(UNITREE_SDK2_HOME)/thirdparty/lib/$(UNITREE_ARCH)
UNITREE_LIBS = $(UNITREE_SDK2_HOME)/lib/$(UNITREE_ARCH)/libunitree_sdk2.a \
	  $(UNITREE_SDK2_HOME)/thirdparty/lib/$(UNITREE_ARCH)/libddscxx.so \
	  $(UNITREE_SDK2_HOME)/thirdparty/lib/$(UNITREE_ARCH)/libddsc.so
LFLAGS += -lfused_odometry_interface -llocalize_ackerman_interface -lparam_interface -lipc -lglobal \
	  $(UNITREE_LIBS) -lstdc++ -pthread -lrt

SOURCES += odometry_6d_go2_main.cpp
PUBLIC_BINARIES = odometry_6d_go2
TARGETS += odometry_6d_go2

odometry_6d_go2: odometry_6d_go2_main.o libodometry_6d_interface.a
endif

include ../Makefile.rules
