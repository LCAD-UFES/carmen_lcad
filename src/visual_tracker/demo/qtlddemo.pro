include(qtlddemo.pri)

TEMPLATE = app
QT += core gui opengl
CONFIG += debug_and_release ordered
DEPENDPATH += .
UI_DIR += ./GeneratedFiles
RCC_DIR += ./GeneratedFiles
DESTDIR = ./bin
CONFIG(debug, debug|release) {
    TARGET = qtlddemod
    DEFINES += DEBUG
    INCLUDEPATH += ./include \
        /usr/include/opencv \
        $(CARMEN_HOME)/include \
        ./GeneratedFiles/Debug
    LIBS += -L/usr/lib \
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc \
        -lopencv_video \
        -L$(CARMEN_HOME)/lib \
        -lkinect_interface \
        -lvisual_tracker_interface -lbumblebee_basic_interface \
        -lparam_interface -lz -lipc -lglobal -lglobal_graphics_qt
    MOC_DIR += ./GeneratedFiles/Debug
    OBJECTS_DIR += Debug
} else {
    TARGET = qtlddemo
    LIBS += -L/usr/lib \
        -lopencv_core \
        -lopencv_highgui \
        -lopencv_imgproc \
        -lopencv_video \
        -L$(CARMEN_HOME)/lib \
        -lkinect_interface \
        -lvisual_tracker_interface -lbumblebee_basic_interface \
        -lparam_interface -lz -lipc -lglobal -lglobal_graphics_qt
    INCLUDEPATH += ./include \
        /usr/include/opencv \
        $(CARMEN_HOME)/include \
        ./GeneratedFiles/Release
    MOC_DIR += ./GeneratedFiles/Release
    OBJECTS_DIR += Release
}
