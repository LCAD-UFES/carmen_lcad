###########################################################
#                  Find OpenCV Library
# See http://sourceforge.net/projects/opencvlibrary/
#----------------------------------------------------------
#
## 1: Setup:
# The following variables are optionally searched for defaults
#  OpenCV_DIR:            Base directory of OpenCv tree to use.
#
## 2: Variable
# The following are set after configuration is done: 
#  
#  OpenCV_FOUND
#  OpenCV_LIBS
#  OpenCV_INCLUDE_DIR
#  OpenCV_VERSION (OpenCV_VERSION_MAJOR, OpenCV_VERSION_MINOR, OpenCV_VERSION_PATCH)
#
#
# Deprecated variable are used to maintain backward compatibility with
# the script of Jan Woetzel (2006/09): www.mip.informatik.uni-kiel.de/~jw
#  OpenCV_INCLUDE_DIRS
#  OpenCV_LIBRARIES
#  OpenCV_LINK_DIRECTORIES
# 
## 3: Version
#
# 2010/04/07 Benoit Rat, Correct a bug when OpenCVConfig.cmake is not found.
# 2010/03/24 Benoit Rat, Add compatibility for when OpenCVConfig.cmake is not found.
# 2010/03/22 Benoit Rat, Creation of the script.
#
#
# tested with:
# - OpenCV 2.1:  MinGW, MSVC2008
# - OpenCV 2.0:  MinGW, MSVC2008, GCC4
#
#
## 4: Licence:
#
# LGPL 2.1 : GNU Lesser General Public License Usage
# Alternatively, this file may be used under the terms of the GNU Lesser

# General Public License version 2.1 as published by the Free Software
# Foundation and appearing in the file LICENSE.LGPL included in the
# packaging of this file.  Please review the following information to
# ensure the GNU Lesser General Public License version 2.1 requirements
# will be met: http://www.gnu.org/licenses/old-licenses/lgpl-2.1.html.
# 
#----------------------------------------------------------

# paulozog(2011,06,06)
find_file(OpenCV_CMAKE_INCLUDE "OpenCVConfig.cmake" 
  HINTS "/usr/local/share/opencv" "/usr/share/opencv" "/usr/local/share/OpenCV" 
  DOC "Directory containing OpenCVConfig.cmake")

if(EXISTS ${OpenCV_CMAKE_INCLUDE})

  include(${OpenCV_CMAKE_INCLUDE}) #That's it.  We're done. Why did this guy write this entire script?
  set (OpenCV_FOUND true)
  set (OpenCV_INCLUDE_DIR "${OpenCV_INCLUDE_DIRS}")

else(EXISTS ${OpenCV_CMAKE_INCLUDE})

  #If there is no OpenCVConfig.cmake, then do what Ayoung does
  find_path(OpenCV_DIR "OpenCVConfig.cmake" 
    DOC "Root directory of OpenCV")

  # ayoung(2011.04.29 when using opencv from synaptic, no OpenCVconfig.cmake will be provided)
  set(OpenCV_SYNAPTIC false)
  if (NOT EXISTS "${OpenCV_DIR}")

    # paulozog(2011.06.06 Don't forget that the user might not even have opencv at all!
    find_library(OpenCV_SYNAPTIC_LIB "cv")
    if (EXISTS ${OpenCV_SYNAPTIC_LIB})
      set(OpenCV_SYNAPTIC 1)
      set(OpenCV_DIR "/usr" CACHE STRING "" FORCE)
    endif(EXISTS ${OpenCV_SYNAPTIC_LIB})

    #message ("Opencv WARNING: No OpenCVConfig.cmake found \n OpenCV_DIR has been set to /usr assuming opencv installed using synaptic.\n If you're using synaptic version opencv do [e] then [c] again to compile.\n\n Or you should set OpenCV_DIR to change for the different version")
  endif (NOT EXISTS "${OpenCV_DIR}")

  ##====================================================
  ## Find OpenCV libraries
  ##----------------------------------------------------
  if(EXISTS "${OpenCV_DIR}" OR ${OpenCV_SYNAPTIC})

    #When its possible to use the Config script use it.
    if(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

      ## Include the standard CMake script
      include("${OpenCV_DIR}/OpenCVConfig.cmake")

      ## Search for a specific version
      set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}")

      #Otherwise try to guess it.
    else(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

      set(OPENCV_LIB_COMPONENTS cxcore cv ml highgui cvaux)
      #find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "")
      find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}/include/opencv" DOC "")

      #Find OpenCV version by looking at cvver.h
      file(STRINGS ${OpenCV_INCLUDE_DIR}/cvver.h OpenCV_VERSIONS_TMP REGEX "^#define CV_[A-Z]+_VERSION[ \t]+[0-9]+$")
      string(REGEX REPLACE ".*#define CV_MAJOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MAJOR ${OpenCV_VERSIONS_TMP})
      string(REGEX REPLACE ".*#define CV_MINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_MINOR ${OpenCV_VERSIONS_TMP})
      string(REGEX REPLACE ".*#define CV_SUBMINOR_VERSION[ \t]+([0-9]+).*" "\\1" OpenCV_VERSION_PATCH ${OpenCV_VERSIONS_TMP})
      set(OpenCV_VERSION ${OpenCV_VERSION_MAJOR}.${OpenCV_VERSION_MINOR}.${OpenCV_VERSION_PATCH} CACHE STRING "" FORCE)
      if (OpenCV_SYNAPTIC)
        set(CVLIB_SUFFIX "") # ayoung(2011.04.29 when using opencv from synaptic, no prefix is provded to find libraries)
      else ()
        set(CVLIB_SUFFIX "${OpenCV_VERSION_MAJOR}${OpenCV_VERSION_MINOR}${OpenCV_VERSION_PATCH}") # ayoung(2011.04.29 when using opencv from synaptic, no prefix is provded to find libraries)
      endif ()
      
    endif(EXISTS "${OpenCV_DIR}/OpenCVConfig.cmake")

    
    

    ## Initiate the variable before the loop
    set(OpenCV_LIBS "")
    set(OpenCV_FOUND_TMP true)

    ## Loop over each components
    foreach(__CVLIB ${OPENCV_LIB_COMPONENTS})

      find_library(OpenCV_${__CVLIB}_LIBRARY_DEBUG NAMES "${__CVLIB}${CVLIB_SUFFIX}d" "lib${__CVLIB}${CVLIB_SUFFIX}d" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)
      find_library(OpenCV_${__CVLIB}_LIBRARY_RELEASE NAMES "${__CVLIB}${CVLIB_SUFFIX}" "lib${__CVLIB}${CVLIB_SUFFIX}" PATHS "${OpenCV_DIR}/lib" NO_DEFAULT_PATH)

      #Remove the cache value
      set(OpenCV_${__CVLIB}_LIBRARY "" CACHE STRING "" FORCE)
      
      #both debug/release
      if(OpenCV_${__CVLIB}_LIBRARY_DEBUG AND OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY debug ${OpenCV_${__CVLIB}_LIBRARY_DEBUG} optimized ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)
        #only debug
      elseif(OpenCV_${__CVLIB}_LIBRARY_DEBUG)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_DEBUG}  CACHE STRING "" FORCE)
        #only release
      elseif(OpenCV_${__CVLIB}_LIBRARY_RELEASE)
        set(OpenCV_${__CVLIB}_LIBRARY ${OpenCV_${__CVLIB}_LIBRARY_RELEASE}  CACHE STRING "" FORCE)
        #no library found
      else()
        set(OpenCV_FOUND_TMP false)
      endif()

      #Add to the general list
      if(OpenCV_${__CVLIB}_LIBRARY)
        set(OpenCV_LIBS ${OpenCV_LIBS} ${OpenCV_${__CVLIB}_LIBRARY})
      endif(OpenCV_${__CVLIB}_LIBRARY)
      
    endforeach(__CVLIB)


    set(OpenCV_FOUND ${OpenCV_FOUND_TMP} CACHE BOOL "" FORCE)


  else(EXISTS "${OpenCV_DIR}" OR ${OpenCV_SYNAPTIC})
    set(ERR_MSG "Please specify OpenCV directory using OpenCV_DIR env. variable")
  endif(EXISTS "${OpenCV_DIR}" OR ${OpenCV_SYNAPTIC})
  ##====================================================


  ##====================================================
  ## Print message
  ##----------------------------------------------------
  if(NOT OpenCV_FOUND)
    # make FIND_PACKAGE friendly
    if(NOT OpenCV_FIND_QUIETLY)
      if(OpenCV_FIND_REQUIRED)
        message(FATAL_ERROR "OpenCV required but some headers or libs not found. ${ERR_MSG}")
      else(OpenCV_FIND_REQUIRED)
        message(STATUS "WARNING: OpenCV was not found. ${ERR_MSG}")
      endif(OpenCV_FIND_REQUIRED)
    endif(NOT OpenCV_FIND_QUIETLY)
  endif(NOT OpenCV_FOUND)
  ##====================================================


  ##====================================================
  ## Backward compatibility
  ##----------------------------------------------------
  if(OpenCV_FOUND)
    option(OpenCV_BACKWARD_COMPA "Add some variable to make this script compatible with the other version of FindOpenCV.cmake" false)
    if(OpenCV_BACKWARD_COMPA)
      #find_path(OpenCV_INCLUDE_DIRS "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory") 
      #find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}" PATH_SUFFIXES "include" "include/opencv" DOC "Include directory")
      find_path(OpenCV_INCLUDE_DIRS "cv.h" PATHS "${OpenCV_DIR}/include/opencv" DOC "Include directory") 
      find_path(OpenCV_INCLUDE_DIR "cv.h" PATHS "${OpenCV_DIR}/include/opencv" DOC "Include directory")
      set(OpenCV_LIBRARIES "${OpenCV_LIBS}" CACHE STRING "" FORCE)
    endif(OpenCV_BACKWARD_COMPA)
  endif(OpenCV_FOUND)
  ##====================================================
endif(EXISTS ${OpenCV_CMAKE_INCLUDE})

##====================================================
## Hide from ccmake GUI
##----------------------------------------------------
MARK_AS_ADVANCED(
  OpenCV_BACKWARD_COMPA
  OpenCV_cv_LIBRARY
  OpenCV_cv_LIBRARY_DEBUG
  OpenCV_cv_LIBRARY_RELEASE
  OpenCV_cvaux_LIBRARY
  OpenCV_cvaux_LIBRARY_DEBUG
  OpenCV_cvaux_LIBRARY_RELEASE
  OpenCV_cxcore_LIBRARY
  OpenCV_cxcore_LIBRARY_DEBUG
  OpenCV_cxcore_LIBRARY_RELEASE
  OpenCV_highgui_LIBRARY
  OpenCV_highgui_LIBRARY_DEBUG
  OpenCV_highgui_LIBRARY_RELEASE
  OpenCV_ml_LIBRARY
  OpenCV_ml_LIBRARY_DEBUG
  OpenCV_ml_LIBRARY_RELEASE
  OpenCV_DIR
  OpenCV_FOUND
  OpenCV_INCLUDE_DIR
  OpenCV_VERSION
  OpenCV_CMAKE_INCLUDE
  OpenCV_SYNAPTIC_LIB
  )
##====================================================
