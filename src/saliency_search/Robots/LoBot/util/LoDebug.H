/**
   \file  Robots/LoBot/util/LoDebug.H
   \brief Debug functions.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoDebug.H $
// $Id: LoDebug.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_DEBUG_UTILITIES_DOT_H
#define LOBOT_DEBUG_UTILITIES_DOT_H

//------------------------------ HEADERS --------------------------------

// INVT utilities
#include "Util/log.H"

// Standard C++ headers
#include <sstream>
#include <ostream>
#include <string>
#include <algorithm>
#include <map>
#include <vector>
#include <iterator>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------------- DEBUG SUPPORT -----------------------------

/// Debugging support: dump an STL container using LERROR
///
/// DEVNOTE: Mostly suitable for small containers, e.g., vector of 10
/// numbers or something like that.
template<typename container>
void dump(const container& C, const std::string& caller,
          const std::string& container_name = "container")
{
   std::ostringstream str ;
   std::copy(C.begin(), C.end(),
             std::ostream_iterator<typename container::value_type>(str, " ")) ;
   LERROR("from %s ==> %s[0x%lx]: size = %d, contents = %s",
          caller.c_str(), container_name.c_str(),
          reinterpret_cast<long int>(&C), static_cast<int>(C.size()),
          str.str().c_str()) ;
}

/// Dump an array of type T given pointers to its first and last elements
template<typename T>
void dump(const T* begin, const T* end, const std::string& caller,
          const std::string& container_name = "array")
{
   std::vector<T> vec(begin, end) ;
   dump(vec, caller, container_name) ;
}

/// Function object to dump an std::pair to the given output stream
template<typename T1, typename T2>
class dump_pair {
   std::ostream& os ;
public:
   dump_pair(std::ostream& s) : os(s) {}
   void operator()(const std::pair<T1, T2>& p) const {
      os << '[' << p.first << ' ' << p.second << "] " ;
   }
} ;

/// The earlier version of the dump() function for dumping containers
/// will only work with containers of built-in/standard types such as
/// vector<int>, list<float>, etc. because it uses std::ostream_iterator,
/// which, annoyingly enough, doesn't work with user-defined types.
/// Therefore, the earlier dump() function won't dump std::map containers
/// (because their value type is an std::pair).
///
/// We work around this problem by providing this overloaded version of
/// dump() especially for std::maps.
template<typename key_type, typename value_type>
void dump(const std::map<key_type, value_type>& C,
          const std::string& caller,
          const std::string& container_name = "container")
{
   std::ostringstream str ;
   std::for_each(C.begin(), C.end(),
                 dump_pair<key_type, value_type>(str)) ;
   LERROR("from %s ==> %s[0x%lx]: size = %d, contents = %s",
          caller.c_str(), container_name.c_str(),
          reinterpret_cast<long int>(&C), static_cast<int>(C.size()),
          str.str().c_str()) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
