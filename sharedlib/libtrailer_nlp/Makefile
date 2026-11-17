include ../../src/Makefile.conf

LINK ?= g++
# Migracao Ubuntu 26.04: caiu o -D_GLIBCXX_USE_CXX11_ABI=0 e subiu para c++14.
#  * O _GLIBCXX_USE_CXX11_ABI=0 forcava o std::string do ABI PRE-C++11. Ele existia para
#    casar com o casadi do pip, construido assim. O casadi do apt no 26.04 usa o ABI NOVO,
#    entao a flag ficou invertida: esta lib passava a pedir _ZN6casadi4OptiC1ERKSs (Ss = ABI
#    antigo) e a libcasadi so' oferece ...NSt7__cxx1112basic_string... -- dava
#    "undefined reference to casadi::Opti::Opti(std::string const&)" ao linkar o offroad_planner.
#  * c++14 pelo mesmo motivo do resto da migracao: o Boost 1.90 exige (item 11 do CHANGELOG).
CXXFLAGS += -std=c++14 -Wno-deprecated-copy

MODULE_NAME = LIBTRAILER_NLP
MODULE_COMMENT = Tractor-trailers Trajectory Planning Library

IFLAGS += -I$(CARMEN_HOME)/include/carmen

#G2O
IFLAGS += -I/usr/include/eigen3 
IFLAGS += -I/usr/include/suitesparse -I/usr/include/eigen3 -I/usr/local/include/EXTERNAL/csparse/ -I/usr/local/include/g2o/

#CASADI
CASADI_DIR=$(shell /usr/bin/env python3 -c "import casadi; print(casadi.__path__[0])")
CXXFLAGS += -Wno-sign-compare -Wno-unused-parameter 
# Migracao Ubuntu 26.04: o pacote apt poe os headers em /usr/include/casadi (path padrao,
# dispensa -I) e o dist-packages do python nao tem mais o subdiretorio include/. O -I fica
# so' como fallback para instalacoes via pip, onde o diretorio existe.
IFLAGS += $(if $(wildcard $(CASADI_DIR)/include),-I$(CASADI_DIR)/include,)
LFLAGS += -L$(CASADI_DIR) -lcasadi -lstdc++ -lglobal_planning_interface -L/usr/lib64/libkml -lkmlbase -lkmldom -lkmlengine -lcarmen_gps_wrapper -ltask_manager_interface -lcarmen_util


SOURCES = trailer_nlp.cpp

PUBLIC_INCLUDES = trailer_nlp.h 

PUBLIC_LIBRARIES = libtrailer_nlp.a

TARGETS = libtrailer_nlp.a

libtrailer_nlp.a: trailer_nlp.o

trailer_nlp.o: trailer_nlp.h

include ../../src/Makefile.rules