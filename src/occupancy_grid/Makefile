include ../Makefile.conf

MODULE_NAME = OCCUPANCY GRID
MODULE_COMMENT = 	Trabalho 2 de Robótica Probabilistíca  \
									GRUPO: Romulo, Tiago, Avelino
LINK = g++

LFLAGS += -lglobal -lmap_interface -lrobot_ackerman_interface -lipc -lparam_interface -lsimulator_ackerman_interface -lslam_interface

SOURCES = occupancy_grid_main.cpp occupancy_grid.cpp

PUBLIC_BINARIES = occupancy_grid

TARGETS = occupancy_grid

occupancy_grid: occupancy_grid_main.o occupancy_grid.o

include ../Makefile.rules
