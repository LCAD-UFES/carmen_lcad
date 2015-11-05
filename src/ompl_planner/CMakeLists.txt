cmake_minimum_required(VERSION 2.8)
project(ompl_demo CXX C)

# set the default build type
if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Debug)
endif()

#find_package(OMPL REQUIRED)
#include_directories(${OMPL_INCLUDE_DIRS})

find_package(Boost COMPONENTS date_time thread serialization filesystem system program_options unit_test_framework REQUIRED)
include_directories(${Boost_INCLUDE_DIR})

option(OMPL_BUILD_DEMOS "Build OMPL demos" ON)

if (OMPL_BUILD_DEMOS)

  macro(add_ompl_demo demo_name)
    add_executable(${ARGV})
    target_link_libraries(${demo_name}
      ompl
      ${Boost_FILESYSTEM_LIBRARY}
      ${Boost_SYSTEM_LIBRARY}
      ${Boost_THREAD_LIBRARY}
      ${Boost_DATE_TIME_LIBRARY}
      ${Boost_PROGRAM_OPTIONS_LIBRARY})
  endmacro(add_ompl_demo)

  add_ompl_demo(ompl_planner ompl_planner_main.cpp)

endif(OMPL_BUILD_DEMOS)
