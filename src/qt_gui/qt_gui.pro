TEMPLATE = app
TARGET = ../../bin/QT_GUI
QT += core \
    gui \
    opengl
HEADERS += map_component/rrt_component.h \
    carmen/robot_config.h \
    carmen/map_config.h \
    carmen/carmen_state.h \
    carmen/carmen_thread.h \
    principal_view.h \
    map_item.h \
    map_component/navigator_plan_component.h \
    map_component/car_component.h \
    map_component/goal_component.h \
    map_component/particle_component.h \
    map_component/laser_component.h \
    map_component/map_component.h \
    main_window.h
SOURCES += map_component/rrt_component.cpp \
    carmen/robot_config.cpp \
    carmen/map_config.cpp \
    carmen/carmen_state.cpp \
    carmen/carmen_thread.cpp \
    principal_view.cpp \
    map_item.cpp \
    map_component/navigator_plan_component.cpp \
    map_component/car_component.cpp \
    map_component/goal_component.cpp \
    map_component/particle_component.cpp \
    map_component/laser_component.cpp \
    map_component/map_component.cpp \
    main_window.cpp \
    qt_gui_main.cpp
FORMS += 
RESOURCES += 
INCLUDEPATH += ../../include
LIBS += -L../../lib -lglobal -lipc -lz \
    -lparam_interface -lmap_interface \
    -llocalize_ackerman_interface \
    -lsimulator_ackerman_interface \
    -lobstacle_avoider_interface \
    -lbase_ackerman_interface \
    -lprob_models \
    -llocalize_traf_module_filter_interface \
    -lsimulator_interface -llocalize_interface \
    -llaser_interface -lgrid_mapping_interface \
    -lnavigator_interface \
    -lnavigator_ackerman_interface -lglobal_graphics_qt \
    -lrrt_planner_interface
