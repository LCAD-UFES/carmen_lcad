/**
   \file  Robots/LoBot/config/LoConfig.H
   \brief Robolocust/lobot configuration database.

   This file defines a class that reads the Robolocust/lobot config file
   (~/.lobotrc by default or as specified by the --config-file option)
   and then populates a two-level map of key-value pairs that other parts
   of the program can use to query whatever parameters they need.

   Robolocust/lobot config files are simplified INI files. See
   LoIniFileLexer.l and LoIniFileParser.y for the syntax rules and an
   illustrative example of what is allowed.

   The first level of the two-level map alluded to above corresponds to
   sections in the INI file and the second level is for the key-value
   pairs of that section. Thus, the first level maps INI file section
   names to maps of key-value pairs. These "inner" maps map key names to
   their corresponding values.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/config/LoConfig.H $
// $Id: LoConfig.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_CONFIGURATION_API_DOT_H
#define LOBOT_CONFIGURATION_API_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/config/LoLexYaccDefs.H"

#include "Robots/LoBot/misc/singleton.hh"
#include "Robots/LoBot/util/LoString.H"

// Standard C++ headers
#include <algorithm>
#include <map>
#include <vector>
#include <string>

//------------------------------ DEFINES --------------------------------

/// The Robolocust config file is divided into multiple sections with
/// each section consisting of key-value pairs. There is a top-level or
/// global section as well that appears before any other sections in the
/// config file. In actuality, this global section is implemented as a
/// specially named section.
///
/// In addition to the above-mentioned global section of the config file,
/// we also have an internal section that is not meant to be used
/// directly by users. Rather, it acts as a sort of scratch space via
/// which different Robolocust modules can exchange bits of data without
/// having to explicitly know each other (which greatly simplifies the
/// overall design).
///
/// This internal section is never defined explicitly in the config file.
/// Instead, Robolocust sets it up internally.
#define LOCD_INTERNAL "__SECRET_INTERNAL_SECTION__"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::ConfigDB
   \brief Robolocust/lobot configuration database.

   This class implements a dictionary of key-value pairs. Both keys and
   values are strings.

   NOTE: This class does not have any public members. It is not meant to
   be used directly clients, which should instead use the API provided by
   lobot::Configuration.
*/
class Configuration ; // forward declaration
class ConfigDB : public singleton<ConfigDB> {
   /// The Robolocust/lobot config file consists of several sections.
   /// Each section has key-value statements in it. The configuration
   /// database mirrors this structure with a two-level map, i.e., a map
   /// of maps. The top-level map maps section names to the collection of
   /// key-value definitions in that section. The key-value maps for a
   /// section map key names to their corresponding values.
   typedef std::map<std::string, std::string> KeyValuePairs ;
   typedef std::map<std::string, KeyValuePairs> ConfigMap ;

   ConfigMap m_config_db ;

   /// Private constructor because the lobot config database is a
   /// singleton object.
   ConfigDB() ;
   friend class singleton<ConfigDB> ;

   /// This method inserts the supplied key-value pair into the specified
   /// section of the configuration database.
   void insert(const std::string& section,
               const std::string& key, const std::string& value) ;

   /// This method returns the value corresponding to the specified key
   /// from the specified section of the configuration database. If the
   /// key is not defined, the default value supplied by the client will
   /// be returned.
   std::string retrieve(const std::string& section,
                        const std::string& key,
                        const std::string& default_value) const ;

   /// Clean-up
   ~ConfigDB() ;

   // The configuration database is not accessed directly by clients.
   // Instead they use the API provided by the Configuration class, which
   // has full access to this object.
   friend class Configuration ;
} ;

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::Configuration
   \brief A more user-friendly API for the ConfigDB.

   lobot::ConfigDB is an "internal" class related to the lobot config
   system and is not meant to be used directly by other objects in the
   lobot system. Instead, client modules should use the API provided by
   this class for loading and retrieving configuration settings.
*/
class Configuration {
   // prevent instantiation, copy and assignment
   Configuration() ;
   Configuration(const Configuration&) ;
   Configuration& operator=(const Configuration&) ;

public:
   /// This method loads the configuration settings from the specified
   /// file.
   static void load(const std::string& config_file) ;

   /// This method inserts the supplied key-value pair into the specified
   /// section of the configuration database.
   static void set(const std::string& section,
                   const std::string& key, const std::string& value) ;

   /// This method inserts the supplied key-value pair into the
   /// unnamed/anonymous global/top-level scope of the configuration
   /// database.
   static void set_global(const std::string& key, const std::string& value) ;

   /// This method inserts the supplied key-value pair into the (secret)
   /// internal section of the configuration database.
   static void set_internal(const std::string& key, const std::string& value) ;

   /// This method retrieves the value corresponding to the specified key
   /// from the specified section of the configuration database. If the
   /// key is not defined, the default value supplied by the client will
   /// be returned.
   template<typename T>
   static T get(const std::string& section, const std::string& key,
                const T& default_value = T()) ;

   /// The configuration database simply retains key-value pairs as
   /// strings. But some settings can be lists (e.g., the value
   /// corresponding to some key may be a list of numbers). This method
   /// "breaks up" the value portion of the specified key and section
   /// into a list and returns the result via an array of type T.
   template<typename T>
   static void get(const std::string& section, const std::string& key,
                   T* target, const T* defaults, unsigned int n) ;

   /// This method retrieves the value corresponding to the specified key
   /// from the unnamed/anonymous global/top-level scope section of the
   /// configuration database. If the key is not defined in the top-level
   /// section, the default value supplied by the client will be
   /// returned.
   template<typename T>
   static T get_global(const std::string& key, const T& defval = T()) ;

   /// Development and debugging support
   static void dump() ;
} ;

/// Retrieve the value corresponding to the specified key from the
/// specified section of the config file and return it as an instance of
/// type T.
template<typename T>
T Configuration::get(const std::string& section, const std::string& key,
                     const T& default_value)
{
   std::string value = ConfigDB::instance().retrieve(section, key, "") ;
   if (value == "") //key-value pair not defined in specified sec. of INI file
      return default_value ;
   return from_string<T>(value) ;
}

/// Partial specialization of above method for retrieving on/off/yes/no
/// flags from the config file.
template<>
bool Configuration::get(const std::string& section, const std::string& key,
                        const bool& default_value)
{
   std::string flag =
      downstring(ConfigDB::instance().retrieve(section, key, "")) ;
   if (flag == "") // not defined in config file
      return default_value ;
   if (flag == "f" || flag == "false" || flag == "no" || flag == "off" ||
       flag == "0" || flag == "disabled")
      return false ;
   return true ;
}

/// Retrieve the values corresponding to the specified key from the
/// specified section of the config file and return them via an array of
/// type T.
template<typename T>
void Configuration::get(const std::string& section, const std::string& key,
                        T* target, const T* defaults, unsigned int n)
{
   std::copy(defaults, defaults + n, target) ;
   std::vector<T> config(string_to_vector<T>(get<std::string>(section, key))) ;
   if (config.size() > n)
      config.resize(n) ;
   std::copy(config.begin(), config.end(), target) ;
}

/// This method retrieves the value corresponding to the specified key
/// from the unnamed/anonymous global/top-level scope section of the
/// configuration database. If the key is not defined in the top-level
/// section, the default value supplied by the client will be returned.
template<typename T>
T Configuration::get_global(const std::string& key, const T& defval)
{
   return get<T>(LOCD_TOP_LEVEL, key, defval) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
