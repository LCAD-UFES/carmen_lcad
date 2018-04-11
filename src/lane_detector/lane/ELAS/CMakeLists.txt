cmake_minimum_required(VERSION 2.8)
project(LaneResearch)
find_package(OpenCV REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
#set(ENV{PKG_CONFIG_PATH} "/usr/local/opencv-2.4.12.3/lib:/usr/include/opencv:$ENV{PKG_CONFIG_PATH}")

set(SRC_FILES_DIR "Source Files")

# CODE
set(CODE_HEADERS
	PreProcessamento.h
	nieto.h 
	Kalman.h KalmanHoughs.h AnaliseDasHoughs.h Houghs.h
	MapaDeEvidencias.h Mapas.h
	FiltroDeParticulas.h FiltroDeParticulasHough.h 
	LMTDetector.h SinalizacaoHorizontal.h
	FaixasAdjacentes.h
)
set(CODE_SOURCE
	PreProcessamento.cpp
	nieto.cpp 
	Kalman.cpp KalmanHoughs.cpp AnaliseDasHoughs.cpp Houghs.cpp
	MapaDeEvidencias.cpp Mapas.cpp
	FiltroDeParticulas.cpp  FiltroDeParticulasHough.cpp
	LMTDetector.cpp SinalizacaoHorizontal.cpp
	FaixasAdjacentes.cpp
)

include_directories(~/carmen_lcad/include)

# ELAS CARMEN
set(ELAS_CARMEN_HEADER ELAS.h ELAS_ALL.hpp 
	pre_processing.h feature_map_generation.h horizontal_signalisation.h lane_estimation.h lmt_classification.h adjacent_lanes.h
	SinalizacaoHorizontal.h AnaliseDasHoughs.h Kalman.h 
)
set(ELAS_CARMEN_SOURCE ELAS.cpp 
	pre_processing.cpp feature_map_generation.cpp horizontal_signalisation.cpp lane_estimation.cpp lmt_classification.cpp adjacent_lanes.cpp
	SinalizacaoHorizontal.cpp AnaliseDasHoughs.cpp Kalman.cpp
)
FILE(GLOB CODE_UTILS utils/*.h utils/*.cpp)

# TinyXML 2
set(LIB_TINYXML2_H libs/tinyxml2/tinyxml2.h)
set(LIB_TINYXML2_SRC libs/tinyxml2/tinyxml2.cpp)

IF (NOT BUILD_CARMEN)
	source_group(${SRC_FILES_DIR}\\utils FILES ${CODE_UTILS})
	
	# TinyXML 2
	add_library(tinyxml2 ${LIB_TINYXML2_H} ${LIB_TINYXML2_SRC})

	# VanishingPoint LIB
	FILE(GLOB LIB_VP_H libs/vanishingPoint/*.h)
	FILE(GLOB LIB_VP_SRC libs/vanishingPoint/*.cpp)
	add_library(vanishingPoint ${LIB_VP_H} ${LIB_VP_SRC})

	add_executable(CODE main.cpp ${CODE_SOURCE} ${CODE_HEADERS} ${ELAS_CARMEN_SOURCE} ${ELAS_CARMEN_HEADER} ${CODE_UTILS})
	# LINK
	target_link_libraries(CODE ${OpenCV_LIBS} tinyxml2 vanishingPoint)
ELSE (NOT BUILD_CARMEN)	
	add_library(ELAS ${LIB_TINYXML2_H} ${LIB_TINYXML2_SRC} ${CODE_SOURCE} ${CODE_HEADERS} ${ELAS_CARMEN_SOURCE} ${ELAS_CARMEN_HEADER} ${CODE_UTILS})
ENDIF (NOT BUILD_CARMEN)
