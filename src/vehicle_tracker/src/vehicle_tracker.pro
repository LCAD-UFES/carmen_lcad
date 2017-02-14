#-------------------------------------------------
#
# Project created by QtCreator 2016-12-14T18:09:29
#
#-------------------------------------------------

QT += core gui

greaterThan(QT_MAJOR_VERSION, 4): QT += widgets

TARGET = vehicle_tracker
TEMPLATE = app

SOURCES += \
	main.cpp \
	vehicle_tracker_gui.cpp \
    canvas.cpp \
    display.cpp \
    types.cpp \
    carmen_gateway.cpp \
    difference_scanner.cpp \
    virtual_scan.cpp

HEADERS += \
	vehicle_tracker_gui.h \
    canvas.h \
    types.h \
    display.h \
    CircleGraphicsItem.h \
    carmen_gateway.h \
    difference_scanner.h \
    virtual_scan.h

INCLUDEPATH += \
	$(CARMEN_HOME)/include

FORMS += vehicle_tracker_gui.ui

LIBS += \
	-lCGAL_Qt5 -lCGAL -lCGAL_Core -lgmp \
	-L$(CARMEN_HOME)/lib -llocalize_ackerman_interface -lvirtual_scan_interface -lvelodyne_interface -lparam_interface -lipc -lglobal \
	-L$(CARMEN_HOME)/sharedlib/libtf/src -ltf \
	-lboost_signals -lboost_system
