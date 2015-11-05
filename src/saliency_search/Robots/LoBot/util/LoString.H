/**
   \file  Robots/LoBot/util/LoString.H
   \brief String functions.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoString.H $
// $Id: LoString.H 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_STRING_UTILITIES_DOT_H
#define LOBOT_STRING_UTILITIES_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/util/range.hh"

// Standard C++ headers
#include <sstream>
#include <string>
#include <list>
#include <deque>
#include <vector>
#include <iterator>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//--------------------- BASIC STRING CONVERSIONS ------------------------

/// Convenient (but perhaps not the most efficient) helper to convert
/// various data types to strings.
///
/// DEVNOTE: Works as long as type T defines an operator << that writes
/// to an ostream.
template<typename T>
std::string to_string(const T& t)
{
   std::ostringstream str ;
   str << t ;
   return str.str() ;
}

/// to_string() specialization for converting ranges to strings.
template<typename T>
std::string to_string(const range<T>& R)
{
   std::ostringstream str ;
   str << R.min() << ' ' << R.max() ;
   return str.str() ;
}

/// Read from string. As above, works as long as type T defines an
/// operator >> that reads from an istream.
template<typename T>
T from_string(const std::string& s, const T& defval = T())
{
   T t(defval) ;
   std::istringstream str(s) ;
   str >> t ;
   return t ;
}

/// from_string() specialization for strings. If the client wants a
/// string from the input string, we just return the input string. If we
/// were to apply the default version of this template function, we would
/// end up parsing the input string as a whitespace separated string
/// stream and only return the first string from this stream.
template<>
std::string from_string(const std::string& s, const std::string&)
{
   return s ;
}

//-------------------------- STRING PARSING -----------------------------

/// Converts a whitespace separated string of T into a vector of T
template<typename T>
std::vector<T> string_to_vector(const std::string& s)
{
   std::istringstream str(s) ;
   return std::vector<T>(std::istream_iterator<T>(str),
                         std::istream_iterator<T>()) ;
}

/// Converts a whitespace separated string of T into a deque of T
template<typename T>
std::deque<T> string_to_deque(const std::string& s)
{
   std::istringstream str(s) ;
   return std::deque<T>(std::istream_iterator<T>(str),
                        std::istream_iterator<T>()) ;
}

/// Converts a whitespace separated string of T into a list of T
template<typename T>
std::list<T> string_to_list(const std::string& s)
{
   std::istringstream str(s) ;
   return std::list<T>(std::istream_iterator<T>(str),
                       std::istream_iterator<T>()) ;
}

//--------------------------- MISCELLANEOUS -----------------------------

/// String case conversion
//@{
std::string upstring(std::string) ;
std::string downstring(std::string) ;
//@}

/// Duplicate the given string n times and return the result in a new
/// string.
std::string dup_string(const std::string& s, int n) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
