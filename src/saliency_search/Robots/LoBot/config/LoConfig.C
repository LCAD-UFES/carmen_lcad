/**
   \file  Robots/LoBot/config/LoConfig.C
   \brief Robolocust/lobot configuration database.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/config/LoConfig.C $
// $Id: LoConfig.C 13912 2010-09-10 07:24:52Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/config/LoConfig.H"
#include "Robots/LoBot/config/LoIniFileLexer.H"
#include "Robots/LoBot/config/LoIniFileParser.H"
#include "Robots/LoBot/misc/LoExcept.H"

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <algorithm>
#include <utility>

// Standard C headers
#include <stdio.h>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------- INITIALIZATION AND CLEAN-UP ----------------------

ConfigDB::ConfigDB(){}
ConfigDB::~ConfigDB(){}

void Configuration::load(const std::string& config_file)
{
   FILE* file = fopen(config_file.c_str(), "r") ;
   if (! file)
      throw customization_error(NO_SUCH_CONFIG_FILE) ;

   yyin = file ;
   int parse_ok = yyparse() ;
   yyin = stdin ;
   fclose(file) ;

   switch (parse_ok) {
      case 0: // all okay
         break ;
      case 1: // syntax error
         throw customization_error(CONFIG_FILE_SYNTAX_ERROR) ;
      case 2: // memory error
         throw customization_error(CONFIG_FILE_MEMORY_ERROR) ;
   }
}

//----------------- CONFIGURATION DATABASE OPERATIONS -------------------

// Adding configuration settings to the database
void ConfigDB::insert(const std::string& section,
                      const std::string& key, const std::string& value)
{
   KeyValuePairs& key_value_map = m_config_db[section] ;
   key_value_map[key] = value ;
}

void Configuration::set(const std::string& section,
                        const std::string& key, const std::string& value)
{
   ConfigDB::instance().insert(section, key, value) ;
}

void Configuration::set_global(const std::string& key, const std::string& val)
{
   set(LOCD_TOP_LEVEL, key, val) ;
}

void Configuration::set_internal(const std::string& key,
                                 const std::string& val)
{
   set(LOCD_INTERNAL, key, val) ;
}

// Retrieving configuration settings from the database
std::string ConfigDB::retrieve(const std::string& section,
                               const std::string& key,
                               const std::string& default_value) const
{
   ConfigMap::const_iterator i = m_config_db.find(section) ;
   if (i == m_config_db.end())
      return default_value ;

   KeyValuePairs::const_iterator j = i->second.find(key) ;
   if (j == i->second.end())
      return default_value ;
   return j->second ;
}

//--------------------------- DEBUG SUPPORT -----------------------------

namespace {

typedef std::map<std::string, std::string> KV ;
typedef std::map<std::string, KV> DB ;

class dump_kvmap {
   const std::string& section ;
public:
   dump_kvmap(const std::string& sec) : section(sec) {}
   void operator()(const KV::value_type& kv) const {
      LERROR("%-15s %-25s %s",
             section.c_str(), kv.first.c_str(), kv.second.c_str()) ;
   }
} ;

void dump_section(const DB::value_type& i)
{
   std::for_each(i.second.begin(), i.second.end(), dump_kvmap(i.first)) ;
}

}

void Configuration::dump()
{
   ConfigDB& db = ConfigDB::instance() ;
   std::for_each(db.m_config_db.begin(), db.m_config_db.end(), dump_section) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
