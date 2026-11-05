# Prefer the package's own exported target. Ubuntu's 4.2.0+dfsg-6 package
# exports an absent CppUnitLite archive, which prevents even non-test consumers
# from loading GTSAMConfig.cmake. Keep that workaround local to this project.
find_path(GTSAM_DIR GTSAMConfig.cmake
  PATHS /usr/local /usr
  PATH_SUFFIXES lib/cmake/GTSAM lib/${CMAKE_LIBRARY_ARCHITECTURE}/cmake/GTSAM)

set(_gtsam_broken_test_export FALSE)
if(GTSAM_DIR STREQUAL "/usr/lib/cmake/GTSAM"
   AND EXISTS "${GTSAM_DIR}/GTSAM-exports-relwithdebinfo.cmake")
  file(READ "${GTSAM_DIR}/GTSAM-exports-relwithdebinfo.cmake" _gtsam_exports)
  if(_gtsam_exports MATCHES "libCppUnitLite\\.a"
     AND NOT EXISTS "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/libCppUnitLite.a")
    set(_gtsam_broken_test_export TRUE)
  endif()
endif()

if(_gtsam_broken_test_export)
  # Only support the known system package layout/version in this fallback.
  include("${GTSAM_DIR}/GTSAMConfigVersion.cmake")
  if(NOT PACKAGE_VERSION VERSION_EQUAL "4.2.0")
    message(FATAL_ERROR "Broken GTSAM test export in an unsupported version: ${PACKAGE_VERSION}")
  endif()
  find_path(GTSAM_APT_INCLUDE_DIR gtsam/config.h PATHS /usr/include NO_DEFAULT_PATH)
  find_library(GTSAM_APT_LIBRARY gtsam
    PATHS /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE} NO_DEFAULT_PATH)
  set(GTSAM_INCLUDE_DIR "${GTSAM_APT_INCLUDE_DIR}")
  set(GTSAM_LIBRARY "${GTSAM_APT_LIBRARY}")
  if(NOT GTSAM_INCLUDE_DIR OR NOT GTSAM_LIBRARY)
    message(FATAL_ERROR "The system GTSAM headers or shared library are missing")
  endif()
  find_package(Boost 1.67 CONFIG REQUIRED COMPONENTS
    serialization filesystem thread program_options date_time timer chrono regex)
  find_package(Eigen3 REQUIRED)
  set(_gtsam_dependencies "")
  # libgtsam-dev does not require libmetis-dev/libsuitesparse-dev. The
  # versioned runtime libraries are sufficient for this binary consumer.
  set(_gtsam_metis_soname libmetis.so.5)
  set(_gtsam_ccolamd_soname libccolamd.so.3)
  foreach(_dependency tbb tbbmalloc metis ccolamd)
    find_library(GTSAM_${_dependency}_LIBRARY
      NAMES ${_dependency} ${_gtsam_${_dependency}_soname})
    if(NOT GTSAM_${_dependency}_LIBRARY)
      message(FATAL_ERROR "Missing GTSAM dependency: ${_dependency}")
    endif()
    list(APPEND _gtsam_dependencies "${GTSAM_${_dependency}_LIBRARY}")
  endforeach()
  add_library(gtsam SHARED IMPORTED)
  set_target_properties(gtsam PROPERTIES
    IMPORTED_LOCATION "${GTSAM_LIBRARY}"
    INTERFACE_INCLUDE_DIRECTORIES "${GTSAM_INCLUDE_DIR}"
    INTERFACE_COMPILE_FEATURES cxx_std_11
    INTERFACE_LINK_LIBRARIES "Boost::serialization;Boost::filesystem;Boost::thread;Boost::program_options;Boost::date_time;Boost::timer;Boost::chrono;Boost::regex;${_gtsam_dependencies};Eigen3::Eigen")
  include("${GTSAM_DIR}/gtsam_extra.cmake")
  set(GTSAM_FOUND TRUE)
  message(STATUS "GTSAM 4.2: using system library without the missing CppUnitLite test export")
else()
  find_package(GTSAM 4.2 REQUIRED CONFIG)
endif()

if(DEFINED GTSAM_VERSION_NUMERIC AND NOT GTSAM_VERSION_NUMERIC LESS 40300)
  message(FATAL_ERROR "This project targets GTSAM 4.2; select its APT installation with GTSAM_DIR")
endif()

# Variables consumed by catkin_package(DEPENDS GTSAM).
set(GTSAM_INCLUDE_DIRS "${GTSAM_INCLUDE_DIR}")
set(GTSAM_LIBRARIES gtsam)
