# -------------------------------------------------
# Project created by QtCreator 2011-05-31T11:30:38
# -------------------------------------------------
QT +=           opengl

TARGET =        mpu9250-OpenGl

TEMPLATE =      app

SOURCES +=      src/main.cpp
SOURCES +=      src/mainwindow.cpp
SOURCES +=      src/objectgl.cpp
SOURCES +=      src/MadgwickAHRS.cpp
SOURCES +=      src/rOc_serial.cpp
SOURCES +=      src/rOc_timer.cpp

HEADERS +=      include/mainwindow.h
HEADERS +=      include/objectgl.h
HEADERS +=      include/MadgwickAHRS.h
HEADERS +=      include/rOc_serial.h
HEADERS +=      include/rOc_timer.h

INCLUDEPATH +=  src
INCLUDEPATH +=  include


OBJECTS_DIR = tmp/
MOC_DIR = tmp/
DESTDIR = bin/

