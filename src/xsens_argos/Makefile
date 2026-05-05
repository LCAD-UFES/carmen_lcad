include ../Makefile.conf

MODULE_NAME = xsens_argos 
MODULE_COMMENT = Driver de IMU do Argos

# --- Caminhos Base ---
UNITREE_SDK_DIR = /opt/unitree_robotics

# --- Compilador e Flags ---
CXX = g++
CFLAGS = -Wall -O3 -std=c++11 -Wno-address-of-packed-member -fPIC

# --- Includes (Onde os arquivos .h estão) ---
IFLAGS += -I$(CARMEN_HOME)/include -I$(UNITREE_SDK_DIR)/include -I$(UNITREE_SDK_DIR)/include/ddscxx

LFLAGS += -L$(UNITREE_SDK_DIR)/lib
# --- Bibliotecas (Onde os arquivos .so e .a estão) ---
LFLAGS += -lunitree_sdk2 -lddscxx -lddsc -lxsens_mtig_interface -lxsens_interface -lparam_interface -lglobal -lipc -lrt -lpthread

        # -L$(CARMEN_HOME)/sharedlib/libcmt/

SOURCES = xsens_argos_main.cpp 

TARGETS = xsens_argos
PUBLIC_BINARIES = xsens_argos 

xsens_argos: xsens_argos_main.o

include ../Makefile.rules
