/**
   \file  Robots/LoBot/config/LoConfigHelpers.H
   \brief Helper/convenience functions for accessing the lobot
   configuration database.

   This file defines several functions that can be used to ease the
   interface to lobot's configuration database. Thus, clients may, for
   instance, simply use the get_conf() function and not have to worry
   about invoking Configuration::get(); so on and so forth.

   Basically, this file provides a bunch of helpers that result less
   typing and less ceremony in client code for using the configuration
   database.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/config/LoConfigHelpers.H $
// $Id: LoConfigHelpers.H 13445 2010-05-21 06:12:57Z mviswana $
//

#ifndef LOBOT_CONFIG_HELPERS_DOT_H
#define LOBOT_CONFIG_HELPERS_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/config/LoConfig.H"
#include "Robots/LoBot/config/LoDefaults.H"

#include "Robots/LoBot/misc/LoTypes.H"
#include "Robots/LoBot/util/LoString.H"
#include "Robots/LoBot/util/triple.hh"
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <string>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//---------------------- GENERAL CONFIG HELPERS -------------------------

/// A convenience routine to allow clients to not have to type the whole
/// Configuration::get call...
template<typename T>
inline T get_conf(const std::string& section, const std::string& key,
                  const T& default_value = T())
{
   return Configuration::get(section, key, default_value) ;
}

/// A convenience routine to return an RGB color from the specified
/// section of the config file.
template<>
PixelType get_conf(const std::string& section, const std::string& key,
                   const PixelType& defval)
{
   int color[3] ;
   const int default_color[] = {defval.red(), defval.green(), defval.blue()} ;
   Configuration::get(section, key, color, default_color, 3) ;
   return PixelType(color[0], color[1], color[2]) ;
}

/// Convenience routine to return a pair from the specified section of
/// the config file.
template<typename T>
std::pair<T, T>
get_conf(const std::string& section, const std::string& key,
         const std::pair<T, T>& defval = std::pair<T, T>(T(), T()))
{
   T pair[2] ;
   const T defaults[] = {defval.first, defval.second} ;
   Configuration::get<T>(section, key, pair, defaults, 2) ;
   return std::pair<T, T>(pair[0], pair[1]) ;
}

/// Convenience routine to return a triple from the specified section of
/// the config file.
template<typename T>
triple<T, T, T>
get_conf(const std::string& section, const std::string& key,
         const triple<T, T, T>& defval = make_triple(T(), T(), T()))
{
   T triple[3] ;
   const T defaults[] = {defval.first, defval.second, defval.third} ;
   Configuration::get<T>(section, key, triple, defaults, 3) ;
   return make_triple(triple[0], triple[1], triple[2]) ;
}

/// Convenience routine to return a range<T> from the specified section
/// of the config file.
template<typename T>
range<T> get_conf(const std::string& section, const std::string& key,
                  const range<T>& defval = range<T>(T(), T()))
{
   std::pair<T, T> r =
      get_conf<T>(section, key, std::make_pair(defval.min(), defval.max())) ;
   return make_range(r.first, r.second) ;
}

/// Retrieve settings from the global section of the config file
template<typename T>
inline T global_conf(const std::string& key, const T& default_value = T())
{
   return Configuration::get_global<T>(key, default_value) ;
}

/// Retrieve settings from the secret internal section of the
/// configuration database.
template<typename T>
inline T internal_conf(const std::string& key, const T& default_value = T())
{
   return get_conf<T>(LOCD_INTERNAL, key, default_value) ;
}

//---------------------- ROBOT PLATFORM HELPERS -------------------------

/// Retrieve settings from the motor section of the config file
template<typename T>
inline T robot_conf(const std::string& key, const T& default_value = T())
{
   return get_conf<T>("robot", key, default_value) ;
}

/// Does the user want to enable the physical robot?
inline bool robot_enabled()
{
   return robot_conf("use_robot", true) ;
}

/// Which robot platform will Robolocust be running on?
inline std::string robot_platform()
{
   return downstring(robot_conf<std::string>("platform", "roomba_cm")) ;
}

/// Is the lobot controller running on an iRobot Create/Roomba?
bool robot_platform_is_roomba() ;

//---------------------------- I/O HELPERS ------------------------------

/// Check what input source to use for the locust LGMD spikes
inline std::string locust_input()
{
   return downstring(global_conf<std::string>("locust_input", "laser")) ;
}

/// Check if cameras are being used to generate LGMD spikes
inline bool video_input()
{
   return locust_input() == "video" ;
}

/// Check if the laser range finder is being used to generate LGMD spikes
inline bool laser_input()
{
   return locust_input() == "laser" ;
}

/// Retrieve settings from the laser section of the config file
template<typename T>
inline T laser_conf(const std::string& key, const T& default_value = T())
{
   return get_conf<T>("laser", key, default_value) ;
}

/// Check which LGMD model is being used
inline std::string locust_model()
{
   return downstring(global_conf<std::string>("locust_model",
                                              LOBOT_DEFAULT_LOCUST_MODEL)) ;
}

//---------------------------- UI HELPERS -------------------------------

/// Retrieve settings from the ui section of the config file
template<typename T>
inline T ui_conf(const std::string& key, const T& default_value = T())
{
   return get_conf<T>("ui", key, default_value) ;
}

/// Check if a drawable (such as a behaviour, an arbiter, LRF visualizer,
/// etc.) is configured to be visualized or not. By default,
/// visualizations are all turned off; the user must explicitly turn them
/// on.
inline bool visualize(const std::string& section)
{
   return get_conf(section, "visualization", false) ;
}

//----------------------- VIDEO RELATED HELPERS -------------------------

/// Retrieve settings from the video section of the config file
template<typename T>
inline T video_conf(const std::string& key, const T& default_value = T())
{
   return get_conf<T>("video", key, default_value) ;
}

///  Retrieve settings from the optical flow section of the config file
template<typename T>
inline T optical_flow_conf(const std::string& key, const T& default_value)
{
   return get_conf<T>("optical_flow", key, default_value) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
