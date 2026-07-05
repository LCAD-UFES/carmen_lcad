include ../Makefile.conf

USE_CASADI = 1

MODULE_NAME = NLP-MATPlan
MODULE_COMMENT = A Non-Linear Programming approch to Trajetory Planning for Multi-Articulated Vehicles

LINK ?= g++
CFLAGS += -fopenmp -Wno-deprecated-declarations
CXXFLAGS += -std=c++11 -Wno-deprecated-declarations
ifdef USE_CASADI
CXXFLAGS += -DUSE_CASADI
endif

IFLAGS += -I$(CARMEN_HOME)/sharedlib/prob_models -I$(CARMEN_HOME)/sharedlib/libcarmodel -I$(CARMEN_HOME)/sharedlib/libcontrol -I/home/lcad/carmen/include
LFLAGS += -L$(CARMEN_HOME)/sharedlib/prob_models -L$(CARMEN_HOME)/sharedlib/libcarmodel -L$(CARMEN_HOME)/sharedlib/libcontrol

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

LFLAGS += $(INTERFACE_LIBS) -lglobal -lipc \
	-lprob_models -lgrid_mapping -lmap_io -lmap_util -lmapper_interface \
	-lparam_interface -lmap_interface -llocalize_ackerman_interface -lsimulator_ackerman_interface \
	-lrobot_ackerman_interface \
	-lmotion_planner_interface -lcollision_detection -lmap_server_interface -lbehavior_selector_interface -lglobal_planning_interface \
	-lobstacle_distance_mapper_interface -lbehavior_selector_interface \
	-lcarmodel -lcarmen_gps_wrapper -lrotation_geometry -lmodel_predictive_planner_optimizer \
	-ltask_manager_interface \
	-lgsl -lgslcblas -lcarmen_util `pkg-config --libs opencv` \

ifdef USE_CASADI
CASADI_DIR=$(shell /usr/bin/env python3 -c "import casadi; print(casadi.__path__[0])")
LFLAGS += -L$(CASADI_DIR) -lcasadi -lstdc++
LFLAGS += -ltrailer_nlp
endif
LFLAGS += -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine
	 
IFLAGS += \
	`pkg-config --cflags gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --cflags gtkglext-1.0`
LFLAGS += \
	-lglobal_graphics \
	`pkg-config --libs gtk+-2.0 gmodule-export-2.0` \
	`pkg-config --libs gtkglext-1.0`

SOURCES = nlp_mat_planner_main.cpp reeds_sheep.cpp \
	offroad_planner_interface.c path_smoothing.cpp nlp_mat_planner_distance_map.cpp path_smoothing_study.cpp \
	nlp_mat_planner_path_planner_astar.cpp trailer_analytical_expansion.cpp \
	send_fake_plan.cpp

PUBLIC_INCLUDES = offroad_planner_interface.h nlp_mat_planner.h nlp_mat_planner_messages.h

PUBLIC_LIBRARIES = liboffroad_planner_interface.a

PUBLIC_BINARIES = nlp_matplan send_fake_plan

ifdef USE_CASADI
TARGETS = liboffroad_planner_interface.a nlp_matplan send_fake_plan
else
TARGETS = liboffroad_planner_interface.a nlp_matplan send_fake_plan # path_smoothing_study
endif

PUBLIC_LIBRARIES_SO = liboffroad_planner_interface.a

ifdef DEBUG
IFLAGS += -DPRINT_TIME -DDRAW_EXPANSION_TREE -DPRINT_DEBUG
endif

# Dar make no NHCM é ruim, demora, e os binários do NHCM não são usados na maioria dos casos ao usar o offroad_planner
nhcm:
	make -C NHCM
	
clean: clean_NHCM

clean_NHCM:
	$(MAKE) clean -C $(CARMEN_HOME)/src/offroad_planner/NHCM

ifdef USE_CASADI
nlp_matplan: nlp_mat_planner_main.o nlp_mat_planner_path_planner_astar.o reeds_sheep.o planning.o nlp_mat_planner_distance_map.o \
	trailer_analytical_expansion.o liboffroad_planner_interface.a trailer_nlp.a
else
nlp_matplan: nlp_mat_planner_main.o nlp_mat_planner_path_planner_astar.o reeds_sheep.o planning.o nlp_mat_planner_distance_map.o \
	trailer_analytical_expansion.o liboffroad_planner_interface.a
endif

send_fake_plan: send_fake_plan.o liboffroad_planner_interface.a

liboffroad_planner_interface.a: offroad_planner_interface.o

#path_smoothing_study: path_smoothing_study.o path_smoothing.o

download:
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_car_cost_matrix_101x36x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_car_cost_matrix_101x36x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1dh9nIVM76IMsVO7eix_eCXPp5Bm6sOU-/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_car_cost_matrix_101x36x1.0.data
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_car_cost_matrix_101x36x1.0.data file already exists. Delete it if you need to update. *)
endif
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_trailer_cost_matrix_81x10x5x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1sPjdHSc2nly2tEPDdVNHq7auA-EIjdoO/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data file already exists. Delete it if you need to update. *)
endif
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1oSySWrFQGajAusoPaWj8fBtLhR_-k00B/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data file already exists. Delete it if you need to update. *)
endif
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1yMKvdAswV88-Nk5Z8H34FEGZ_GLA926g/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano file already exists. Delete it if you need to update. *)
endif
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/18GEzqiHRO16g28iDsVJGrB65J1VlI9aO/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic file already exists. Delete it if you need to update. *)
endif
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_art_cost_matrix_101x36x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1F7QDy4en3szKUUMMOuRL0DAsaXxkSAhf/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data file already exists. Delete it if you need to update. *)
endif

update:
ifneq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_car_cost_matrix_101x36x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Removing offroad_planner_car_cost_matrix_101x36x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	rm $(CARMEN_HOME)/data/offroad_planner_car_cost_matrix_101x36x1.0.data
endif
ifneq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Removing offroad_planner_trailer_cost_matrix_81x10x5x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	rm $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x10x5x1.0.data
endif
ifneq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Removing offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	rm $(CARMEN_HOME)/data/offroad_planner_iara_trailer_cost_matrix_81x10x5x1.0.data
endif
ifneq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Removing offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano )
	$(info ***************************************************************************************************************)
	$(info )
	rm $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano
endif
ifneq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Removing offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic )
	$(info ***************************************************************************************************************)
	$(info )
	rm $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic
endif
ifneq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Removing offroad_planner_art_cost_matrix_101x36x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	rm $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data
endif
	make download
	
axor_suzano:
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1yMKvdAswV88-Nk5Z8H34FEGZ_GLA926g/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_axor_suzano file already exists. Delete it if you need to update. *)
endif

atego_mosaic:
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/18GEzqiHRO16g28iDsVJGrB65J1VlI9aO/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_trailer_cost_matrix_81x9x5x1.0.data_9_de_maio_atego_mosaic file already exists. Delete it if you need to update. *)
endif

atego2430_vale:
ifeq (,$(wildcard $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data))
	$(info )
	$(info **************************************************************************************************************)
	$(info *	Downloading offroad_planner_art_cost_matrix_101x36x1.0.data )
	$(info ***************************************************************************************************************)
	$(info )
	$(CARMEN_HOME)/bin/gdown.pl https://drive.google.com/file/d/1F7QDy4en3szKUUMMOuRL0DAsaXxkSAhf/view?usp=sharing $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data
else
	$(info * The $(CARMEN_HOME)/data/offroad_planner_art_cost_matrix_101x36x1.0.data file already exists. Delete it if you need to update. *)
endif

# rules

include ../Makefile.rules

