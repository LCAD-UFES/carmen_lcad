include ../Makefile.conf

MODULE_NAME = LEGGACY 500 SOCKET INTERFACE
MODULE_COMMENT = "Send motion commands to be executed by in the hardware by socket"

LINK = g++
CFLAGS += -std=c99

IFLAGS += -I$(CARMEN_HOME)/include/carmen

LFLAGS += -lbase_ackerman_interface -lrobot_ackerman_interface -lparam_interface -lobstacle_avoider -lglobal -lipc -lfused_odometry_interface -lcollision_detection \
-lford_escape_hybrid_interface -lobstacle_avoider_interface

SOURCES = obstacle_avoider_to_socket_client.cpp socket_to_legacy500_server.cpp legacy500_to_socket_server.cpp socket_to_odometry_client.cpp

TARGETS = obstacle_avoider_to_socket_client socket_to_legacy500_server legacy500_to_socket_server socket_to_odometry_client

PUBLIC_BINARIES = obstacle_avoider_to_socket_client socket_to_legacy500_server legacy500_to_socket_server socket_to_odometry_client

obstacle_avoider_to_socket_client:	obstacle_avoider_to_socket_client.o

socket_to_legacy500_server:	socket_to_legacy500_server.o

socket_to_odometry_client: socket_to_odometry_client.o

legacy500_to_socket_server: legacy500_to_socket_server.o

include ../Makefile.rules