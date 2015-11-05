/**
   \file  Robots/LoBot/misc/LoExcept.C
   \brief Various exceptions thrown by different Robolocust subsystems.
*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoExcept.C $
// $Id: LoExcept.C 13967 2010-09-18 08:00:07Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/misc/LoExcept.H"

// Standard C++ headers
#include <map>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- ERROR MESSAGES -----------------------------

/*
 ************************************************************************
 *                                                                      *
 * DEVNOTE: Steps for adding a new error                                *
 * -------------------------------------                                *
 *    1. Add an enum for the new error code in LoExcept.H.              *
 *                                                                      *
 *    2. Add a #defined string for the new error in this section of     *
 *       this file.                                                     *
 *                                                                      *
 *    3. Add an initializer for the new error in the constructor of the *
 *       private messages class defined in the next section of this     *
 *       file.                                                          *
 *                                                                      *
 *    4. If required, define a new exception struct derived from        *
 *       lobot::uhoh in LoExcept.H and define its constructor in the    *
 *       EXCEPTION CLASSES section of this file.                        *
 *                                                                      *
 ************************************************************************
*/

// Library/dependency issues
#ifndef LOEM_MISSING_PTHREAD
   #define LOEM_MISSING_PTHREAD \
              "sorry, need the POSIX threads library for multithreading"
#endif
#ifndef LOEM_MISSING_LIBDC1394
   #define LOEM_MISSING_LIBDC1394 "sorry, need libdc1394 for capturing " \
                                  "from FireWire cameras"
#endif
#ifndef LOEM_MISSING_FFMPEG
   #define LOEM_MISSING_FFMPEG "sorry, need FFmpeg libraries for " \
                               "encoding/decoding movie files"
#endif
#ifndef LOEM_MISSING_OPENCV
   #define LOEM_MISSING_OPENCV "sorry, need OpenCV for Horn-Schunck " \
                               "optical flow algorithm"
#endif
#ifndef LOEM_MISSING_OPENGL
   #define LOEM_MISSING_OPENGL "sorry, need OpenGL and GLUT for " \
                               "laser range finder visualization"
#endif
#ifndef LOEM_MISSING_GLEW
   #define LOEM_MISSING_GLEW "sorry, off-screen rendering requires " \
                             "OpenGL extension wrangler library GLEW"
#endif
#ifndef LOEM_MISSING_LIBURG
   #define LOEM_MISSING_LIBURG "sorry, need the URG library for " \
                               "interfacing with laser range finder"
#endif
#ifndef LOEM_MISSING_LIBDEVIL
   #define LOEM_MISSING_LIBDEVIL "sorry, need OpenIL/DevIL library for " \
                                 "saving screen capture frames to disk"
#endif
#ifndef LOEM_MISSING_LIBGSL
   #define LOEM_MISSING_LIBGSL "sorry, need the GSL library for ICP, etc."
#endif

// Thread related errors
#ifndef LOEM_THREAD_CREATION_FAILURE
   #define LOEM_THREAD_CREATION_FAILURE "unable to create new thread"
#endif
#ifndef LOEM_MUTEX_INIT_ERROR
   #define LOEM_MUTEX_INIT_ERROR "unable to initialize mutex"
#endif
#ifndef LOEM_COND_INIT_ERROR
   #define LOEM_COND_INIT_ERROR "unable to initialize condition variable"
#endif
#ifndef LOEM_RWLOCK_INIT_ERROR
   #define LOEM_RWLOCK_INIT_ERROR "unable to initialize reader-writer lock"
#endif
#ifndef LOEM_RWLOCK_RDLOCK_FAILED
   #define LOEM_RWLOCK_RDLOCK_FAILED "unable to read-lock reader-writer lock"
#endif
#ifndef LOEM_RWLOCK_WRLOCK_FAILED
   #define LOEM_RWLOCK_WRLOCK_FAILED "unable to write-lock reader-writer lock"
#endif

// Bus errors
#ifndef LOEM_INIT_PROBLEM
   #define LOEM_INIT_PROBLEM "unable to acquire raw 1394 handle"
#endif
#ifndef LOEM_NO_CAMERAS
   #define LOEM_NO_CAMERAS "no cameras connected to FireWire bus"
#endif
#ifndef LOEM_HIGHEST_NODE
   #define LOEM_HIGHEST_NODE "cannot work with camera that is "\
                             "highest numbered node on FireWire bus"
#endif

// Camera node errors
#ifndef LOEM_CAMERA_NODES_FREED
   #define LOEM_CAMERA_NODES_FREED "sorry, camera nodes have been freed"
#endif
#ifndef LOEM_BAD_CAMERA_NODE_INDEX
   #define LOEM_BAD_CAMERA_NODE_INDEX "camera node index out of range"
#endif

// Camera initialization errors
#ifndef LOEM_SETUP_FAILED
   #define LOEM_SETUP_FAILED "unable to setup camera for capture"
#endif
#ifndef LOEM_START_TRANSMISSION_FAILED
   #define LOEM_START_TRANSMISSION_FAILED "unable to start ISO transmission"
#endif

// Video I/O errors
#ifndef LOEM_MPEG_FILES_NOT_FOUND
   #define LOEM_MPEG_FILES_NOT_FOUND \
              "MPEG files specified for playback could not be found"
#endif
#ifndef LOEM_NO_VIDEOSTREAM_SOURCE
   #define LOEM_NO_VIDEOSTREAM_SOURCE "video stream source missing"
#endif
#ifndef LOEM_NO_COMPOSITOR_SOURCES
   #define LOEM_NO_COMPOSITOR_SOURCES "no compositor sources specified"
#endif

// Laser range finder errors
#ifndef LOEM_LRF_CONNECTION_FAILURE
   #define LOEM_LRF_CONNECTION_FAILURE \
              "unable to connect to laser range finder device"
#endif
#ifndef LOEM_LRF_DATA_RETRIEVAL_FAILURE
   #define LOEM_LRF_DATA_RETRIEVAL_FAILURE \
              "unable to retrieve data from laser range finder"
#endif
#ifndef LOEM_NO_LRF_SOURCE
   #define LOEM_NO_LRF_SOURCE "laser range finder is not the input source"
#endif

// I/O errors
#ifndef LOEM_SERIAL_PORT_INIT_ERROR
   #define LOEM_SERIAL_PORT_INIT_ERROR "unable to initialize serial port"
#endif
#ifndef LOEM_SERIAL_PORT_BAD_ARG
   #define LOEM_SERIAL_PORT_BAD_ARG \
              "bad argument to serial port read/write call"
#endif
#ifndef LOEM_SERIAL_PORT_READ_ERROR
   #define LOEM_SERIAL_PORT_READ_ERROR "unable to read from serial port"
#endif
#ifndef LOEM_SERIAL_PORT_WRITE_ERROR
   #define LOEM_SERIAL_PORT_WRITE_ERROR "unable to write to serial port"
#endif

// Motor errors
#ifndef LOEM_MOTOR_READ_FAILURE
   #define LOEM_MOTOR_READ_FAILURE "unable to read from motor serial port"
#endif
#ifndef LOEM_IN_PLACE_TURNS_NOT_SUPPORTED
   #define LOEM_IN_PLACE_TURNS_NOT_SUPPORTED \
      "sorry, selected robot platform does not support in-place turns"
#endif

// Incorrect command line options and configuration settings
#ifndef LOEM_BAD_OPTION
   #define LOEM_BAD_OPTION "command line parse error"
#endif
#ifndef LOEM_NO_SUCH_CONFIG_FILE
   #define LOEM_NO_SUCH_CONFIG_FILE "unable to open specified config file"
#endif
#ifndef LOEM_CONFIG_FILE_SYNTAX_ERROR
   #define LOEM_CONFIG_FILE_SYNTAX_ERROR "syntax error in config file"
#endif
#ifndef LOEM_CONFIG_FILE_MEMORY_ERROR
   #define LOEM_CONFIG_FILE_MEMORY_ERROR \
              "ran out of memory while parsing config file"
#endif
#ifndef LOEM_BAD_GEOMETRY_SPEC
   #define LOEM_BAD_GEOMETRY_SPEC "bad drawable geometry specification"
#endif
#ifndef LOEM_MISSING_CMDLINE_ARGS
   #define LOEM_MISSING_CMDLINE_ARGS "required command line arguments missing"
#endif
#ifndef LOEM_BAD_RESULTS_SPECS
   #define LOEM_BAD_RESULTS_SPECS "bad results file and dir specs"
#endif

// Unknown model/algorithms
#ifndef LOEM_UNKNOWN_LOCUST_MODEL
   #define LOEM_UNKNOWN_LOCUST_MODEL "unknown locust model"
#endif
#ifndef LOEM_UNKNOWN_BEHAVIOR
   #define LOEM_UNKNOWN_BEHAVIOR "unknown behavior"
#endif
#ifndef LOEM_UNKNOWN_ROBOT_PLATFORM
   #define LOEM_UNKNOWN_ROBOT_PLATFORM "unknown robot platform"
#endif

// Arbiter errors
#ifndef LOEM_ARBITER_NOT_RUNNING
   #define LOEM_ARBITER_NOT_RUNNING "DAMN arbiter is no longer running"
#endif
#ifndef LOEM_UNSUPPORTED_TURN_DIRECTION
   #define LOEM_UNSUPPORTED_TURN_DIRECTION \
              "turn arbiter does not support supplied direction"
#endif

// Behaviour errors
#ifndef LOEM_MOTOR_SYSTEM_MISSING
   #define LOEM_MOTOR_SYSTEM_MISSING \
              "motor subsystem required by LGMD module or an active behaviour"
#endif
#ifndef LOEM_LASER_RANGE_FINDER_MISSING
   #define LOEM_LASER_RANGE_FINDER_MISSING \
              "behaviour/visualizer needs laser range finder"
#endif
#ifndef LOEM_MAPPING_DISABLED
   #define LOEM_MAPPING_DISABLED \
              "selected functionality requires mapping to be enabled"
#endif
#ifndef LOEM_BOGUS_LOW_LEVEL_ODOMETRY
   #define LOEM_BOGUS_LOW_LEVEL_ODOMETRY \
              "bogus low-level odometry ==> low-level berserk or bad config"
#endif
#ifndef LOEM_NO_GOALS
   #define LOEM_NO_GOALS "behaviour requires at least one goal"
#endif

// Assorted errors
#ifndef LOEM_DANGER_ZONE_LRF_NOT_SETUP
   #define LOEM_DANGER_ZONE_LRF_NOT_SETUP \
              "danger zone object requires LRF prior to instantiation"
#endif
#ifndef LOEM_NO_INPUT_SOURCE
   #define LOEM_NO_INPUT_SOURCE "no input source for locust LGMD model"
#endif
#ifndef LOEM_NOT_ENOUGH_LOCUSTS
   #define LOEM_NOT_ENOUGH_LOCUSTS \
              "need at least 1 locust for visualization and 2 for an EMD"
#endif
#ifndef LOEM_IDC_CONVERGENCE_FAILURE
   #define LOEM_IDC_CONVERGENCE_FAILURE \
              "IDC scan matching algorithm failed to converge"
#endif
#ifndef LOEM_PARTICLE_INDEX_OUT_OF_BOUNDS
   #define LOEM_PARTICLE_INDEX_OUT_OF_BOUNDS \
              "invalid index for accessing particle state variable"
#endif
#ifndef LOEM_OPENGL_FBO_INIT_ERROR
   #define LOEM_OPENGL_FBO_INIT_ERROR \
              "unable to create OpenGL off-screen rendering buffer"
#endif
#ifndef LOEM_GLEW_INIT_ERROR
   #define LOEM_GLEW_INIT_ERROR \
              "unable to initialize OpenGL extension wrangler library GLEW"
#endif
#ifndef LOEM_OPENCV_IMAGE_INIT_ERROR
   #define LOEM_OPENCV_IMAGE_INIT_ERROR \
              "there was a problem creating some OpenCV image"
#endif
#ifndef LOEM_SIGNALS_MASK_SETUP_FAILURE
   #define LOEM_SIGNALS_MASK_SETUP_FAILURE \
              "unable to setup signals mask for shutdown thread"
#endif
#ifndef LOEM_SIGWAIT_FAILED
   #define LOEM_SIGWAIT_FAILED \
              "sigwait() system function failed in shutdown thread"
#endif
#ifndef LOEM_LOGIC_ERROR
   #define LOEM_LOGIC_ERROR \
              "buggy code! some kind of impossible case or other logic error"
#endif
#ifndef LOEM_BROKEN_FEATURE
   #define LOEM_BROKEN_FEATURE "attempt to use a broken feature"
#endif

//------------- MAPPING BETWEEN ERROR CODES AND MESSAGES ----------------

// Forward declarations
static const char* error_msg(int code) ;

// Quick helper to map error codes to appropriate message strings
class messages {
   typedef std::map<int, const char*> msg_map ;
   msg_map m_map ;
   messages() ;
   friend const char* error_msg(int code) ;
} ;

/*
 *************************************************************************
 *                                                                       *
 * DEVNOTE: Steps for adding a new error                                 *
 * -------------------------------------                                 *
 *    1. Add an enum for the new error code in LoExcept.H.               *
 *                                                                       *
 *    2. Add a #defined string for the new error to the previous section *
 *       of this file.                                                   *
 *                                                                       *
 *    3. Add an initializer for the new error in the following           *
 *       constructor.                                                    *
 *                                                                       *
 *    4. If required, define a new exception struct derived from         *
 *       lobot::uhoh in LoExcept.H and define its constructor in the     *
 *       next section of this file.                                      *
 *                                                                       *
 *************************************************************************
*/
messages::messages()
{
   m_map[MISSING_PTHREAD]   = LOEM_MISSING_PTHREAD ;
   m_map[MISSING_LIBDC1394] = LOEM_MISSING_LIBDC1394 ;
   m_map[MISSING_FFMPEG]    = LOEM_MISSING_FFMPEG ;
   m_map[MISSING_OPENCV]    = LOEM_MISSING_OPENCV ;
   m_map[MISSING_OPENGL]    = LOEM_MISSING_OPENGL ;
   m_map[MISSING_GLEW]      = LOEM_MISSING_GLEW ;
   m_map[MISSING_LIBURG]    = LOEM_MISSING_LIBURG ;
   m_map[MISSING_LIBDEVIL]  = LOEM_MISSING_LIBDEVIL ;
   m_map[MISSING_LIBGSL]    = LOEM_MISSING_LIBGSL ;

   m_map[THREAD_CREATION_FAILURE] = LOEM_THREAD_CREATION_FAILURE ;
   m_map[MUTEX_INIT_ERROR]        = LOEM_MUTEX_INIT_ERROR ;
   m_map[COND_INIT_ERROR]         = LOEM_COND_INIT_ERROR ;
   m_map[RWLOCK_INIT_ERROR]       = LOEM_RWLOCK_INIT_ERROR ;
   m_map[RWLOCK_RDLOCK_FAILED]    = LOEM_RWLOCK_RDLOCK_FAILED ;
   m_map[RWLOCK_WRLOCK_FAILED]    = LOEM_RWLOCK_WRLOCK_FAILED ;

   m_map[INIT_PROBLEM] = LOEM_INIT_PROBLEM ;
   m_map[NO_CAMERAS]   = LOEM_NO_CAMERAS ;
   m_map[HIGHEST_NODE] = LOEM_HIGHEST_NODE ;

   m_map[SETUP_FAILED] = LOEM_SETUP_FAILED ;
   m_map[START_TRANSMISSION_FAILED] = LOEM_START_TRANSMISSION_FAILED ;

   m_map[MPEG_FILES_NOT_FOUND]  = LOEM_MPEG_FILES_NOT_FOUND ;
   m_map[NO_VIDEOSTREAM_SOURCE] = LOEM_NO_VIDEOSTREAM_SOURCE ;
   m_map[NO_COMPOSITOR_SOURCES] = LOEM_NO_COMPOSITOR_SOURCES ;

   m_map[LRF_CONNECTION_FAILURE]     = LOEM_LRF_CONNECTION_FAILURE ;
   m_map[LRF_DATA_RETRIEVAL_FAILURE] = LOEM_LRF_DATA_RETRIEVAL_FAILURE ;
   m_map[NO_LRF_SOURCE]              = LOEM_NO_LRF_SOURCE ;

   m_map[SERIAL_PORT_INIT_ERROR]  = LOEM_SERIAL_PORT_INIT_ERROR ;
   m_map[SERIAL_PORT_BAD_ARG]     = LOEM_SERIAL_PORT_BAD_ARG ;
   m_map[SERIAL_PORT_READ_ERROR]  = LOEM_SERIAL_PORT_READ_ERROR ;
   m_map[SERIAL_PORT_WRITE_ERROR] = LOEM_SERIAL_PORT_WRITE_ERROR ;

   m_map[MOTOR_READ_FAILURE]           = LOEM_MOTOR_READ_FAILURE ;
   m_map[IN_PLACE_TURNS_NOT_SUPPORTED] = LOEM_IN_PLACE_TURNS_NOT_SUPPORTED ;

   m_map[BAD_OPTION]               = LOEM_BAD_OPTION ;
   m_map[NO_SUCH_CONFIG_FILE]      = LOEM_NO_SUCH_CONFIG_FILE ;
   m_map[CONFIG_FILE_SYNTAX_ERROR] = LOEM_CONFIG_FILE_SYNTAX_ERROR ;
   m_map[CONFIG_FILE_MEMORY_ERROR] = LOEM_CONFIG_FILE_MEMORY_ERROR ;
   m_map[BAD_GEOMETRY_SPEC]        = LOEM_BAD_GEOMETRY_SPEC ;
   m_map[MISSING_CMDLINE_ARGS]     = LOEM_MISSING_CMDLINE_ARGS ;
   m_map[BAD_RESULTS_SPECS]        = LOEM_BAD_RESULTS_SPECS ;

   m_map[UNKNOWN_LOCUST_MODEL]      = LOEM_UNKNOWN_LOCUST_MODEL ;
   m_map[UNKNOWN_BEHAVIOR]          = LOEM_UNKNOWN_BEHAVIOR ;
   m_map[UNKNOWN_ROBOT_PLATFORM]    = LOEM_UNKNOWN_ROBOT_PLATFORM ;

   m_map[ARBITER_NOT_RUNNING]        = LOEM_ARBITER_NOT_RUNNING ;
   m_map[UNSUPPORTED_TURN_DIRECTION] = LOEM_UNSUPPORTED_TURN_DIRECTION ;

   m_map[MOTOR_SYSTEM_MISSING]       = LOEM_MOTOR_SYSTEM_MISSING ;
   m_map[LASER_RANGE_FINDER_MISSING] = LOEM_LASER_RANGE_FINDER_MISSING ;
   m_map[MAPPING_DISABLED]           = LOEM_MAPPING_DISABLED ;
   m_map[BOGUS_LOW_LEVEL_ODOMETRY]   = LOEM_BOGUS_LOW_LEVEL_ODOMETRY ;
   m_map[NO_GOALS]                   = LOEM_NO_GOALS ;

   m_map[DANGER_ZONE_LRF_NOT_SETUP]    = LOEM_DANGER_ZONE_LRF_NOT_SETUP ;
   m_map[NO_INPUT_SOURCE]              = LOEM_NO_INPUT_SOURCE ;
   m_map[NOT_ENOUGH_LOCUSTS]           = LOEM_NOT_ENOUGH_LOCUSTS ;
   m_map[PARTICLE_INDEX_OUT_OF_BOUNDS] = LOEM_PARTICLE_INDEX_OUT_OF_BOUNDS ;
   m_map[OPENGL_FBO_INIT_ERROR]        = LOEM_OPENGL_FBO_INIT_ERROR ;
   m_map[GLEW_INIT_ERROR]              = LOEM_GLEW_INIT_ERROR ;
   m_map[OPENCV_IMAGE_INIT_ERROR]      = LOEM_OPENCV_IMAGE_INIT_ERROR ;
   m_map[SIGNALS_MASK_SETUP_FAILURE]   = LOEM_SIGNALS_MASK_SETUP_FAILURE ;
   m_map[SIGWAIT_FAILED]               = LOEM_SIGWAIT_FAILED ;
   m_map[LOGIC_ERROR]                  = LOEM_LOGIC_ERROR ;
   m_map[BROKEN_FEATURE]               = LOEM_BROKEN_FEATURE ;
}

// The following function returns the error message corresponding to the
// specified error code. In order to work correctly, this function will
// need an instance of the above map. The simplest thing to do is to
// instantiate it as a local static variable.
//
// DEVNOTE: Using a global static instance might not work if there are
// other such static global/member variables in other translation units
// (no thanks to the static initialization dependency problem).
static const char* error_msg(int code)
{
   static const char* unknown = "unknown error" ;
   static const messages M ;

   messages::msg_map::const_iterator it = M.m_map.find(code) ;
   if (it == M.m_map.end())
      return unknown ;
   return it->second ;
}

//------------------------- EXCEPTION CLASSES ---------------------------

// Base for all lobot exceptions
uhoh::uhoh(int code, const std::string& msg)
   : runtime_error(msg), m_code(code)
{}

// Missing libraries
missing_libs::missing_libs(int code)
   : uhoh(code, error_msg(code))
{}

// Thread related errors
thread_error::thread_error(int code)
   : uhoh(code, error_msg(code))
{}

// 1394 bus errors
bus_error::bus_error(int code)
   : uhoh(code, error_msg(code))
{}

// Camera errors
camera_error::camera_error(int code)
   : uhoh(code, error_msg(code))
{}

// Video I/O errors
vstream_error::vstream_error(int code)
   : uhoh(code, error_msg(code))
{}

// Laser range finder errors
lrf_error::lrf_error(int code)
   : uhoh(code, error_msg(code))
{}

// I/O errors
io_error::io_error(int code)
   : uhoh(code, error_msg(code))
{}

// Motor errors
motor_error::motor_error(int code)
   : uhoh(code, error_msg(code))
{}

// Incorrect command line options and configuration settings
customization_error::customization_error(int code)
   : uhoh(code, error_msg(code))
{}

// Unknown locust models, integration algorithms, etc.
unknown_model::unknown_model(int code)
   : uhoh(code, error_msg(code))
{}

// Arbiter errors
arbiter_error::arbiter_error(int code)
   : uhoh(code, error_msg(code))
{}

// Behavior errors
behavior_error::behavior_error(int code)
   : uhoh(code, error_msg(code))
{}

// Miscellaneous errors
misc_error::misc_error(int code)
   : uhoh(code, error_msg(code))
{}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
