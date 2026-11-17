include ../Makefile.conf

#### 
# To use Caffe Net read the README_USAR_SQUEEZENET.txt
#####
MODULE_NAME = TRAFFIC_LIGHT 
MODULE_COMMENT = Mapping, Detection and Inference State of Traffic Lights

LINK = g++

# ---------------------------------------------------------------------------------------
# Migração Ubuntu 26.04: este módulo depende da libwnn (repo do LCAD, sem pacote apt) e,
# para os binários de reconhecimento, de CUDA/Caffe. Só que a `libtraffic_light_interface`
# (wrapper IPC puro) e os headers públicos são necessários para o `rddf`, que faz
# #include <carmen/traffic_light_interface.h> e linka -ltraffic_light_interface. Por isso o
# módulo entrou no PACKAGES do src/Makefile, com tudo que depende da libwnn condicionado à
# presença dela: sem a lib instalada, sai só a interface (que basta para o rddf); com a lib,
# o módulo volta a ser construído inteiro, sem editar nada.
# ---------------------------------------------------------------------------------------
HAS_WNN := $(firstword $(wildcard /usr/local/lib/libwnn.* /usr/lib/libwnn.* /usr/lib/x86_64-linux-gnu/libwnn.*))

CXXFLAGS += -Wno-ignored-qualifiers -fopenmp 
CFLAGS += -Wall -fopenmp 
ifneq ($(HAS_WNN),)
SUBDIRS += TLightStateRecog
endif

IFLAGS +=  -std=c++11 -I/usr/local/include/bullet/ 

LFLAGS += -lpthread -fopenmp -lglobal -lparam_interface -lipc -lGL -lGLU -lglut \
	 -ltime_profile -lbumblebee_basic_interface -lstereo_interface \
	 -llocalize_ackerman_interface -lrddf_interface
ifneq ($(HAS_WNN),)
LFLAGS += -lwnn
endif

LFLAGS +=  -lgomp 

IFLAGS += `pkg-config opencv4 --cflags 2>/dev/null || pkg-config opencv --cflags`
LFLAGS += `pkg-config opencv4 --libs 2>/dev/null || pkg-config opencv --libs`

# `which nvcc` sozinho nao basta como teste de CUDA:
#  1. esta arvore foi configurada com `configure --nocuda`, que grava NO_CUDA=1 no
#     Makefile.vars, e mesmo assim o bloco abaixo era avaliado so' porque o binario
#     do nvcc existe na maquina;
#  2. CUDA_LIBS nunca foi definida em lugar nenhum da arvore, entao `-L$(CUDA_LIBS)`
#     virava um `-L` vazio e o linker nao encontrava libs que existem de fato
#     (curand/cublas/cusolver moram em /usr/local/cuda-*/lib64);
#  3. -lcudnn e' pacote separado do CUDA e pode nao estar instalado -- era ele que
#     derrubava o link mesmo com o -L correto.
HAS_CUDA := $(if $(NO_CUDA),,$(shell which nvcc 2>/dev/null))
ifneq ($(HAS_CUDA),)

CUDA_LIBS ?= $(firstword $(wildcard /usr/local/cuda/lib64 /usr/local/cuda-*/lib64))
LFLAGS += -L$(CUDA_LIBS)
LFLAGS += -lcudart -lcuda -lcurand -lcublas -lcusolver
ifneq (,$(wildcard $(CUDA_LIBS)/libcudnn.so /usr/lib/*-linux-gnu/libcudnn.so))
LFLAGS += -lcudnn
endif

ifdef CAFFE_HOME
IFLAGS += -I$(CAFFE_HOME)/include -I$(CAFFE_HOME)/build/src/ -I/usr/local/cuda/include 
LFLAGS += -L$(CAFFE_HOME)/build/lib -lcaffe -lglog -lprotobuf 

SOURCES += squeezenet_tlight.cpp
PUBLIC_INCLUDES += squeezenet_tlight.h
PUBLIC_LIBRARIES += libsqueezenet_tlight.a
TARGETS += libsqueezenet_tlight.a
CXXFLAGS += -DUSE_CAFFE
endif

ifdef DLIB
IFLAGS += -I$(CARMEN_HOME)/sharedlib/libtf/src 
LFLAGS += -ldlib
CXXFLAGS += -DUSE_DLIB
endif

endif



#SOURCES = traffic_light_main.cpp traffic_light_interface.cpp traffic_light.cpp mapping_traffic_light_main.cpp log_generate_images.cpp
SOURCES = traffic_light_main.cpp traffic_light_interface.cpp traffic_light.cpp log_generate_images.cpp #traffic_light_annotation_tool.cpp tlight_train.cpp tlight_evaluate_train.cpp 

PUBLIC_INCLUDES = traffic_light_interface.h traffic_light_messages.h traffic_light.h

PUBLIC_LIBRARIES = libtraffic_light_interface.a

#PUBLIC_BINARIES = traffic_light mapping_traffic_light log_generate_images
PUBLIC_BINARIES = traffic_light log_generate_images

#TARGETS = libtraffic_light_interface.a  traffic_light mapping_traffic_light log_generate_images
TARGETS = libtraffic_light_interface.a  

ifneq ($(HAS_CUDA),)
TARGETS += log_generate_images #traffic_light_annotation_tool tlight_train tlight_evaluate_train
# O binario traffic_light depende de TLightStateRecog/tlight_vgram.o (regra la' embaixo),
# e tlight_vgram.h faz #include <wnn/VgRamNeuron.h>. Mas o .cpp so' entra em SOURCES sob
# `ifneq ($(HAS_WNN),)`, umas 25 linhas adiante -- ou seja, com nvcc instalado e libwnn
# ausente o make tentava linkar um objeto cuja fonte ele nao compila, e o build inteiro
# parava com "fatal error: wnn/VgRamNeuron.h: No such file or directory".
# A condicao agora e' a mesma dos SOURCES, que e' o que o comentario no topo deste arquivo
# ja' prometia: sem a libwnn sai so' a interface, que e' o que o rddf precisa.
ifneq ($(HAS_WNN),)
TARGETS += traffic_light
endif
endif

PUBLIC_LIBRARIES_SO = libtraffic_light_interface.so

ifndef NO_PYTHON
TARGETS += libtraffic_light_interface.so.1
endif

ifndef NO_GRAPHICS
IFLAGS += `$(GTK_CONFIG) --cflags` 
LFLAGS += -lparam_interface -lipc -lglobal -lglobal_graphics \
	`$(GTK_CONFIG) --libs` 
ifneq ($(HAS_WNN),)
SOURCES += traffic_light_view.cpp
PUBLIC_BINARIES += traffic_light_view
TARGETS += traffic_light_view
endif
endif

#tlight_evaluate_train: tlight_evaluate_train.o tlight_vgram.o
tlight_train: tlight_train.o tlight_vgram.o

libtraffic_light_interface.a:		traffic_light_interface.o traffic_light.o

libtraffic_light_interface.so.1:	traffic_light_interface.o traffic_light.o

ifneq ($(HAS_WNN),)
SOURCES += TLightStateRecog/tlight_vgram.cpp
endif

ifdef CAFFE_HOME
libsqueezenet_tlight.a: squeezenet_tlight.o

traffic_light: traffic_light_main.o libtraffic_light_interface.a TLightStateRecog/tlight_vgram.o squeezenet_tlight.o #tlight_vgram.o
else
traffic_light: traffic_light_main.o libtraffic_light_interface.a TLightStateRecog/tlight_vgram.o #traffic_light_annotation_tool.o tlight_vgram.o 
endif
#mapping_traffic_light:			mapping_traffic_light_main.o libtraffic_light_interface.a

log_generate_images:			log_generate_images.o

traffic_light_view:			traffic_light_view.o libtraffic_light_interface.a

#traffic_light_annotation_tool: traffic_light_annotation_tool.o

# rules
include ../Makefile.rules
