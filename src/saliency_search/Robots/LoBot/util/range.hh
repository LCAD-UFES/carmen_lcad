/**
   \file  range.hh
   \brief A convenient structure for holding a min-max range.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/range.hh $
// $Id: range.hh 13037 2010-03-23 01:00:53Z mviswana $
//

#ifndef LOBOT_RANGE_DOT_H
#define LOBOT_RANGE_DOT_H

//------------------------------ HEADERS --------------------------------

// Standard C++ headers
#include <algorithm>
#include <limits>
#include <utility>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/// A convenient means of specifying a min-max range.
template<typename T>
class range {
   /// We store the min and max in an STL pair.
   std::pair<T, T> m_range ;

public:
   /// The type of the range's values.
   typedef T value_type ;

   /// Construct a min-max range.
   range(value_type, value_type) ;

   /// Copy constructor.
   range(const range&) ;

   /// Assignment operator.
   range& operator=(const range&) ;

   /// Return the min value of the range.
   const value_type& min() const {return m_range.first ;}

   /// Return the max value of the range.
   const value_type& max() const {return m_range.second ;}

   /// Return the range's mid value.
   value_type mid() const {return (min() + max())/2 ;}

   /// Return the size of the range, i.e., max - min.
   ///
   /// NOTE: This function returns max - min + 1 for integral types.
   value_type size() const ;

   /// Set the range's minimum value.
   void min(value_type m) {m_range.first = m ;}

   /// Set the range's maximum value.
   void max(value_type m) {m_range.second = m ;}

   /// Reset the range using the specified minimum and maixmum values.
   void reset(value_type min, value_type max) {
      m_range.first  = std::min(min, max) ;
      m_range.second = std::max(min, max) ;
   }

   /// Test whether a given value is in this range or not.
   bool in(value_type v) const {
      return m_range.first <= v && v <= m_range.second ;
   }
} ;

// Range constructor
template<typename T>
range<T>::range(T min, T max)
   : m_range(std::min(min, max), std::max(min, max))
{}

// Copy constructor
template<typename T>
range<T>::range(const range& R)
   : m_range(R.m_range)
{}

// Assignment
template<typename T>
range<T>& range<T>::operator=(const range& R)
{
   if (&R != this)
      m_range = R.m_range ;
   return *this ;
}

// Range size
template<typename T>
T range<T>::size() const
{
   if (std::numeric_limits<T>::is_integer)
      return m_range.second - m_range.first + 1 ;
   return m_range.second - m_range.first ;
}

//----------------------- CONVENIENCE FUNCTIONS -------------------------

/// Returns a range using the min and max values supplied to it. This
/// function allows clients to construct a range without having to
/// explicitly specify the type as would be required with a class
/// instantiation. Instead, clients can take advantage of type deduction
/// for template functions and simply provide the parameters, letting the
/// compiler figure out the correct types.
template<typename T>
inline range<T> make_range(T min, T max)
{
   return range<T>(min, max) ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
