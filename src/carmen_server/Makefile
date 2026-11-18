include ../Makefile.conf

MODULE_NAME = "CARMEN SERVER"
MODULE_COMMENT = "Ponte IPC <-> ZMQ para o app remoto (substitui o proccontrol_gui e as interfaces graficas)"

CXXFLAGS += -std=c++11

# So' bibliotecas de interface que ja' existem: o servidor nao muda nenhum modulo.
LFLAGS += -lcamera_drivers -ljpeg -lmapper_interface -lvelodyne_interface -lmoving_objects_interface -lgps_xyz_interface -lfused_odometry_interface -lxsens_interface \
	  -lobstacle_avoider_interface -lfrenet_path_planner_interface -loffroad_planner_interface -luser_app_server_interface \
	  -ltask_manager_interface -lford_escape_hybrid_interface -lbase_ackerman_interface \
	  -lbehavior_selector_interface -lnavigator_ackerman_interface -lroute_planner_interface \
	  -lrddf_interface -lmap_server_interface -llocalize_ackerman_interface \
	  -lproccontrol_interface -lmap_io -lmap_interface -lparam_interface -lipc -lglobal \
	  -lzmq -lz -lstdc++ -lm

SOURCES = carmen_server_main.cpp

PUBLIC_INCLUDES =
PUBLIC_LIBRARIES =
PUBLIC_BINARIES = carmen_server

TARGETS = carmen_server

carmen_server: carmen_server_main.o

# rules

include ../Makefile.rules
