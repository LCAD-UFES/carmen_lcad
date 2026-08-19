cmake_minimum_required(VERSION 2.8.3)
project(lio_sam)

set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -pthread")

find_package(catkin REQUIRED COMPONENTS
  tf
  roscpp
  rospy
  cv_bridge
  # pcl library
  pcl_conversions
  # msgs
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

find_package(OpenMP REQUIRED)
set(PCL_DIR "/usr/lib/x86_64-linux-gnu/cmake/pcl" CACHE PATH "PCL cmake dir" FORCE)
# Ubuntu 26.04: o VTK 9.5 do apt define VTK::jsoncpp apontando para JsonCpp::JsonCpp,
# mas nao chama find_package(JsonCpp) quando o PCL o carrega -> "target was not found"
# no generate. Carregar o jsoncpp antes cria o alvo. Ver ~/ros_src/NOTAS.md.
find_package(jsoncpp QUIET)
find_package(PCL REQUIRED QUIET)
find_package(OpenCV REQUIRED QUIET)
find_package(GTSAM REQUIRED QUIET)

add_message_files(
  DIRECTORY msg
  FILES
  cloud_info.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
  nav_msgs
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  DEPENDS PCL GTSAM

  CATKIN_DEPENDS 
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  message_runtime 
  message_generation
)

# include directories
include_directories(
	include
	${catkin_INCLUDE_DIRS}
	${PCL_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
	${GTSAM_INCLUDE_DIR}
)

# link directories
link_directories(
	include
	${PCL_LIBRARY_DIRS}
  ${OpenCV_LIBRARY_DIRS}
  ${GTSAM_LIBRARY_DIRS}
)

###############################
## Optional GPU map search   ##
###############################
# Downsample + batched k-NN search used by mapOptmization to replace the
# hottest items in perf (flann kdtree search ~28%, VoxelGrid ~2.5%).
# Falls back to a CPU stub (always returns false -> mapOptmization uses the
# original PCL VoxelGrid/KdTreeFLANN path) if no CUDA toolkit is found, so
# the package still builds on machines without a GPU (e.g. a dev laptop).
find_package(CUDA QUIET)

if(CUDA_FOUND)
  message(STATUS "CUDA ${CUDA_VERSION} found -- building GPU map search (cuda_map_search)")

  # Cada arquitetura só existe a partir de uma certa versão do toolkit --
  # pedir sm_86 (Ampere/RTX 30) num nvcc mais velho que 11.1 dá
  # "nvcc fatal: Unsupported gpu architecture 'compute_86'" na hora do build.
  # Por isso a lista de -gencode é montada de acordo com CUDA_VERSION
  # detectado pelo find_package(CUDA) acima, em vez de fixa.
  #   sm_52 -> GTX 900 series (Maxwell)      -- CUDA >= 6.5
  #   sm_61 -> GTX 10 series, ex 1060/1070/1080 (Pascal) -- CUDA >= 8.0
  #   sm_75 -> RTX 20 series (Turing)        -- CUDA >= 10.0
  #   sm_86 -> RTX 30 series (Ampere)        -- CUDA >= 11.1
  # Se a frota ainda tiver placa Kepler (GTX 600/700), adicione:
  #   -gencode arch=compute_35,code=sm_35
  set(_CMS_GENCODE_FLAGS -O3 --std=c++14)
  set(_CMS_HIGHEST_ARCH "")

  if(CUDA_VERSION VERSION_GREATER_EQUAL "6.5")
    list(APPEND _CMS_GENCODE_FLAGS -gencode arch=compute_52,code=sm_52)
    set(_CMS_HIGHEST_ARCH "52")
  endif()
  if(CUDA_VERSION VERSION_GREATER_EQUAL "8.0")
    list(APPEND _CMS_GENCODE_FLAGS -gencode arch=compute_61,code=sm_61)
    set(_CMS_HIGHEST_ARCH "61")
  endif()
  if(CUDA_VERSION VERSION_GREATER_EQUAL "10.0")
    list(APPEND _CMS_GENCODE_FLAGS -gencode arch=compute_75,code=sm_75)
    set(_CMS_HIGHEST_ARCH "75")
  endif()
  if(CUDA_VERSION VERSION_GREATER_EQUAL "11.1")
    list(APPEND _CMS_GENCODE_FLAGS -gencode arch=compute_86,code=sm_86)
    set(_CMS_HIGHEST_ARCH "86")
  endif()

  # PTX fallback (forward compatibility): -gencode ...,code=sm_XX só embute
  # binário SASS pra aquela arch exata. Se a GPU real for MAIS NOVA que
  # tudo que o toolkit sabe gerar (ex: GPU Ampere/Ada com toolkit <11.1),
  # o driver não acha SASS compatível E não tem PTX pra compilar em JIT ->
  # "cudaErrorInvalidDeviceFunction: invalid device function" em runtime,
  # mesmo com o build passando. Foi exatamente esse o erro visto no log.
  # Embutir o virtual-arch mais alto suportado também como PTX
  # (code=compute_XX, não code=sm_XX) resolve: SASS não tem compat forward,
  # PTX tem.
  if(_CMS_HIGHEST_ARCH)
    list(APPEND _CMS_GENCODE_FLAGS -gencode arch=compute_${_CMS_HIGHEST_ARCH},code=compute_${_CMS_HIGHEST_ARCH})
  endif()

  message(STATUS "cuda_map_search: gencode flags for CUDA ${CUDA_VERSION} -> ${_CMS_GENCODE_FLAGS}")
  set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS}; ${_CMS_GENCODE_FLAGS})

  cuda_include_directories(include)
  cuda_add_library(cuda_map_search src/cuda_map_search.cu)
  add_definitions(-DWITH_CUDA_MAP_SEARCH)
else()
  message(WARNING "CUDA not found -- mapOptmization will build with the CPU-only stub "
                   "(cuda_map_search_stub.cpp); it will use the original PCL kdtree/VoxelGrid path")
  add_library(cuda_map_search src/cuda_map_search_stub.cpp)
endif()

###########
## Build ##
###########

# Range Image Projection
add_executable(${PROJECT_NAME}_imageProjection src/imageProjection.cpp)
add_dependencies(${PROJECT_NAME}_imageProjection ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Feature Association
add_executable(${PROJECT_NAME}_featureExtraction src/featureExtraction.cpp)
add_dependencies(${PROJECT_NAME}_featureExtraction ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_link_libraries(${PROJECT_NAME}_featureExtraction ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

# Mapping Optimization
add_executable(${PROJECT_NAME}_mapOptmization 
  src/mapOptmization.cpp
  src/Scancontext.cpp
)
add_dependencies(${PROJECT_NAME}_mapOptmization ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_generate_messages_cpp)
target_compile_options(${PROJECT_NAME}_mapOptmization PRIVATE ${OpenMP_CXX_FLAGS})
target_link_libraries(${PROJECT_NAME}_mapOptmization ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} ${OpenMP_CXX_FLAGS} gtsam cuda_map_search)

# IMU Preintegration
add_executable(${PROJECT_NAME}_imuPreintegration src/imuPreintegration.cpp)
target_link_libraries(${PROJECT_NAME}_imuPreintegration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} gtsam)