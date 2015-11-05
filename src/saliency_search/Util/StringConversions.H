/*!@file Util/StringConversions.H Convert to and from std::string */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/StringConversions.H $
// $Id: StringConversions.H 11934 2009-11-06 03:03:46Z itti $
//

#ifndef STRINGCONVERSIONS_H_DEFINED
#define STRINGCONVERSIONS_H_DEFINED

#include <string>
#include <typeinfo>

#include "rutz/time.h"  // for rutz::time

struct in_addr;

/// Type of exception that is thrown from throwBadConversion
/** The real type of the exception is a derived type that is hidden
    from this header file -- that's so that we can encapsulate the
    implementation. */
class conversion_error : public std::exception
{
protected:
  conversion_error() throw();
  virtual ~conversion_error() throw();

public:
  virtual const char* what() const throw() = 0;

  /// Raise an exception for an invalid string conversion to the given type
  /** The exception thrown will be derived from conversion_error. */
  template <class T>
  static inline void raise(const std::string& str,
                           const std::string& extrainfo = "")
  {
    raise_impl(typeid(T), str, extrainfo);
  }

private:
  static void raise_impl(const std::type_info& type,
                         const std::string& str,
                         const std::string& extrainfo);
};

/*! \defgroup strcvthelp String conversion helpers

   We define a number of string conversion helpers here just so that
   we don't require each type that we may want to instantiate into a
   ModelParam derivative to define operator<< and operator>>. The main
   reason is that this definition requires #including <sstream>
   (especially if the class is all-inline like Point2D<int>), which would
   largely increase the compile time if we did it for all our simple
   types like Dims, Point2D<int>, etc.  But the code below may some day
   migrate into proper definitions of operator<< and operator>> for
   those data types. */
//@{

/*! @name Basic type string conversion functions */
//@{
//! int -> string via stringstream
std::string convertToString(const int& val);
//! string -> int via stringstream
void convertFromString(const std::string& str, int& val);

//! long int -> string via stringstream
std::string convertToString(const long int& val);
//! string -> long int via stringstream
void convertFromString(const std::string& str, long int& val);

//! long long int -> string via stringstream
std::string convertToString(const long long int& val);
//! string -> long long int via stringstream
void convertFromString(const std::string& str, long long int& val);

//! short int -> string via stringstream
std::string convertToString(const short int& val);
//! string -> short int via stringstream
void convertFromString(const std::string& str, short int& val);

//! unsigned int -> string via stringstream
std::string convertToString(const unsigned int& val);
//! string -> unsigned int via stringstream
void convertFromString(const std::string& str, unsigned int& val);

//! unsigned long int -> string via stringstream
std::string convertToString(const unsigned long int& val);
//! string -> unsigned long int via stringstream
void convertFromString(const std::string& str, unsigned long int& val);

//! unsigned long long int -> string via stringstream
std::string convertToString(const unsigned long long int& val);
//! string -> unsigned long long int via stringstream
void convertFromString(const std::string& str, unsigned long long int& val);

//! unsigned short int -> string via stringstream
std::string convertToString(const unsigned short int& val);
//! string -> unsigned short int via stringstream
void convertFromString(const std::string& str, unsigned short int& val);

//! unsigned char -> string via stringstream
std::string convertToString(const unsigned char& val);
//! string -> unsigned char via stringstream
void convertFromString(const std::string& str, unsigned char& val);

//! float -> string via stringstream
std::string convertToString(const float& val);
//! string -> float via stringstream
void convertFromString(const std::string& str, float& val);

//! double -> string via stringstream
std::string convertToString(const double& val);
//! string -> double stringstream
void convertFromString(const std::string& str, double& val);

//! std::string overload
std::string convertToString(const std::string& val);
//! std::string overload
void convertFromString(const std::string& str, std::string& val);

//! bool overload: format is "true|false"
std::string convertToString(const bool& val);
//! bool overload
/*! Anything with a first letter 't', T', 'y', 'Y', or '1' will
  convert to true; anything with a first letter 'f', F', 'n', 'N', or
  '0' will convert to false */
void convertFromString(const std::string& str, bool& val);

//@}

/*! @name Overloaded implementations of string conversion functions */
//@{

//! in_addr overload: format is "<int>.<int>.<int>.<int>"
std::string convertToString(const in_addr& val);
//! in_addr overload: format is "<int>.<int>.<int>.<int>"
void convertFromString(const std::string& str, in_addr& val);

//! rutz::time overload: format is "<h>:<m>:<s>.<ms>"
std::string convertToString(const rutz::time& val);
//! rutz::time overload: format is "<h>:<m>:<s>.<ms>"
void convertFromString(const std::string& str, rutz::time& val);

//@}

/*! @name Easier-syntax functions to convert to/from string */
//@{

//! converts val into a string and returns the string
template <class T>
inline std::string toStr(const T& val);

//! converts str into a T value and returns the T value
template <class T>
inline T fromStr(const std::string& str);
//@}


// ######################################################################
// inline functions
// ######################################################################

// ######################################################################
// let this function stay inline so that in case somebody tries to run
// toStr() for a type for which there is no convertToString(), we will
// get an error at compile time instead of at link time
template <class T>
inline std::string toStr(const T& val)
{ return convertToString(val); }

// ######################################################################
// let this function stay inline so that in case somebody tries to run
// fromStr() for a type for which there is no convertFromString(), we
// will get an error at compile time instead of at link time
template <class T>
T fromStr(const std::string& str)
{
  T val;
  convertFromString(str, val);
  return val;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
