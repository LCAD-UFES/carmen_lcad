#-------------------------------------------------
#
# Project created by QtCreator 2011-09-16T14:23:49
#
#-------------------------------------------------

QT       += core gui

TARGET = skeleton_module_view
TEMPLATE = app

DESTDIR = $(CARMEN_HOME)/bin

LIBS += -L$(CARMEN_HOME)/lib -lparam_interface -lipc -lglobal -lskeleton_module_sensor_interface -lskeleton_module_filter_interface -lglobal_graphics_qt

INCLUDEPATH += $(CARMEN_HOME)/include

SOURCES += main.cpp\
        skeleton_module_view.cpp

HEADERS  += skeleton_module_view.h

FORMS    += skeleton_module_view.ui
