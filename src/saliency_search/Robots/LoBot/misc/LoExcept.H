/**
   \file Robots/LoBot/misc/LoExcept.H

   \brief Various exceptions thrown by different pieces of
   Lobot/Robolocust.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/misc/LoExcept.H $
// $Id: LoExcept.H 13967 2010-09-18 08:00:07Z mviswana $
//

#ifndef LOBOT_EXCEPTIONS_DOT_H
#define LOBOT_EXCEPTIONS_DOT_H

//------------------------------ HEADERS --------------------------------

// Standard C++ headers
#include <stdexcept>
#include <string>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//---------------------------- ERROR CODES ------------------------------

/*
 ************************************************************************
 *                                                                      *
 * DEVNOTE: Steps for adding a new error                                *
 * -------------------------------------                                *
 *    1. Add an enum for the new error code here.                       *
 *                                                                      *
 *    2. Add a #defined string for the new error in LoExcept.C.         *
 *                                                                      *
 *    3. Add an initializer for the new error in the constructor of the *
 *       private messages class defined in LoExcept.C.                  *
 *                                                                      *
 *    4. If required, add a new exception struct in the next section of *
 *       this file and define its constructor in LoExcept.C.            *
 *                                                                      *
 ************************************************************************
*/
enum
{
   // Dependency/library issues
   MISSING_PTHREAD = 1,
   MISSING_LIBDC1394,
   MISSING_FFMPEG,
   MISSING_OPENCV,
   MISSING_OPENGL,
   MISSING_GLEW,
   MISSING_LIBURG,
   MISSING_LIBDEVIL,
   MISSING_LIBGSL,

   // Thread related errors
   THREAD_CREATION_FAILURE,
   MUTEX_INIT_ERROR,
   COND_INIT_ERROR,
   RWLOCK_INIT_ERROR,
   RWLOCK_RDLOCK_FAILED,
   RWLOCK_WRLOCK_FAILED,

   // FireWire bus problems
   INIT_PROBLEM,
   NO_CAMERAS,
   HIGHEST_NODE,

   // Camera problems
   CAMERA_NODES_FREED,
   BAD_CAMERA_NODE_INDEX,
   SETUP_FAILED,
   START_TRANSMISSION_FAILED,

   // Video I/O
   MPEG_FILES_NOT_FOUND,
   NO_VIDEOSTREAM_SOURCE,
   NO_COMPOSITOR_SOURCES,

   // Laser range finder errors
   LRF_CONNECTION_FAILURE,
   LRF_DATA_RETRIEVAL_FAILURE,
   NO_LRF_SOURCE,

   // I/O errors
   SERIAL_PORT_INIT_ERROR,
   SERIAL_PORT_BAD_ARG,
   SERIAL_PORT_READ_ERROR,
   SERIAL_PORT_WRITE_ERROR,

   // Motor errors
   MOTOR_READ_FAILURE,
   IN_PLACE_TURNS_NOT_SUPPORTED,

   // Incorrect command line options and configuration settings
   BAD_OPTION,
   NO_SUCH_CONFIG_FILE,
   CONFIG_FILE_SYNTAX_ERROR,
   CONFIG_FILE_MEMORY_ERROR,
   BAD_GEOMETRY_SPEC,
   MISSING_CMDLINE_ARGS,
   BAD_RESULTS_SPECS,

   // Unknown models/algorithms
   UNKNOWN_LOCUST_MODEL,
   UNKNOWN_BEHAVIOR,
   UNKNOWN_ROBOT_PLATFORM,

   // DAMN arbiter errors
   ARBITER_NOT_RUNNING,
   UNSUPPORTED_TURN_DIRECTION,

   // Behaviour related errors
   MOTOR_SYSTEM_MISSING,
   LASER_RANGE_FINDER_MISSING,
   MAPPING_DISABLED,
   BOGUS_LOW_LEVEL_ODOMETRY,
   NO_GOALS,

   // Assorted errors
   DANGER_ZONE_LRF_NOT_SETUP,
   NO_INPUT_SOURCE,
   NOT_ENOUGH_LOCUSTS,
   PARTICLE_INDEX_OUT_OF_BOUNDS,
   OPENGL_FBO_INIT_ERROR,
   GLEW_INIT_ERROR,
   OPENCV_IMAGE_INIT_ERROR,
   SIGNALS_MASK_SETUP_FAILURE,
   SIGWAIT_FAILED,
   LOGIC_ERROR,
   BROKEN_FEATURE,
} ;

//------------------------- EXCEPTION CLASSES ---------------------------

// Base for all lobot exceptions
class uhoh : public std::runtime_error {
protected:
   int m_code ;
   uhoh(int code, const std::string& msg) ;
public:
   int code() const throw() {return m_code ;}
} ;

// Missing libraries
struct missing_libs : public uhoh {
   missing_libs(int code)  ;
} ;

// Thread related errors
struct thread_error : public uhoh {
   thread_error(int code) ;
} ;

// 1394 bus errors
struct bus_error : public uhoh {
   bus_error(int code) ;
} ;

// Camera init errors
struct camera_error : public uhoh {
   camera_error(int code) ;
} ;

// Video I/O errors
struct vstream_error : public uhoh {
   vstream_error(int code) ;
} ;

// Laser range finder errors
struct lrf_error : public uhoh {
   lrf_error(int code) ;
} ;

// I/O errors
struct io_error : public uhoh {
   io_error(int code) ;
} ;

// Motor errors
struct motor_error : public uhoh {
   motor_error(int code) ;
} ;

// Incorrect command line options and configuration settings
struct customization_error : public uhoh {
   customization_error(int code) ;
} ;

// Unknown locust models, integration algorithms, etc.
struct unknown_model : public uhoh {
   unknown_model(int code) ;
} ;

// Arbiter errors
struct arbiter_error : public uhoh {
   arbiter_error(int code) ;
} ;

// Behaviour errors
struct behavior_error : public uhoh {
   behavior_error(int code) ;
} ;

// Miscellaneous errors
struct misc_error : public uhoh {
   misc_error(int code) ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
