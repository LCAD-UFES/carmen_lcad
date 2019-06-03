# -------------------------------------------------
# Project created by QtCreator 2011-05-31T11:30:38
# -------------------------------------------------
QT +=           opengl

TARGET =        imu_viewer

TEMPLATE =      app

SOURCES +=      imu_viewer_main.cpp
SOURCES +=      mainwindow.cpp
SOURCES +=      objectgl.cpp

INCLUDEPATH +=	$(CARMEN_HOME)/include
LIBS +=			-L$(CARMEN_HOME)/lib -lxsens_interface -lparam_interface -lipc -lglobal -lgps_xyz_interface -lrotation_geometry -lxml2 -lz -lm -lpi_imu_interface

HEADERS +=      mainwindow.h
HEADERS +=      objectgl.h

OBJECTS_DIR = ./
MOC_DIR = ./
DESTDIR = ./

