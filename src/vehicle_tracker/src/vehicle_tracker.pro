#-------------------------------------------------
#
# Project created by QtCreator 2016-12-14T18:09:29
#
#-------------------------------------------------

QT       += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vehicle_tracker
TEMPLATE = app


SOURCES += main.cpp\
        vehicle_tracker_gui.cpp \
    canvas.cpp \
    display.cpp

HEADERS  += vehicle_tracker_gui.h \
    canvas.h \
    types.h \
    display.h \
    CircleGraphicsItem.h

FORMS    += vehicle_tracker_gui.ui

unix:!macx: LIBS += -lCGAL_Qt5 -lCGAL -lCGAL_Core -lgmp
