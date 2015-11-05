/*!@file Util/TypeTraits.H Template approach to getting compile-time
 information about types.*/

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/TypeTraits.H $
// $Id: TypeTraits.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef TYPETRAITS_H_DEFINED
#define TYPETRAITS_H_DEFINED

//! The generic TypeTraits implementation.
/*! This template struct can be specialized or partially specialized
    for other user-defined types. In general, the approach should be
    that the generic version makes the most conservative
    specifications, and that specializations can make more aggressive
    specifications. That way, if we forget to specialize for a given
    type, we might risk missing some optimization, but won't risk
    breaking the program. */
template <class T>
struct TypeTraits
{
  //! Whether the type is trivial (either builtin or a plain-old-data struct).
  /*! This will be used to determine whether objects of the type can
      safely be handled as raw memory; that is, whether they can be
      initialized with memset(), copied with memcpy(), and don't have
      to have their destructors run, etc. Any of the builtin types
      meet these criteria, as well as any user-defined plain-old-data
      (POD) type. A POD type must have members that are all either
      builtin types or other POD types, and must not have a
      user-defined destqructor, copy constructor, or assignment
      operator. It <b>can</b> have user-defined constructors. */
  enum { isTrivial = 0 };
};

template <> struct TypeTraits<bool>           { enum { isTrivial = 1 }; };

template <> struct TypeTraits<char>           { enum { isTrivial = 1 }; };
template <> struct TypeTraits<unsigned char>  { enum { isTrivial = 1 }; };
template <> struct TypeTraits<short>          { enum { isTrivial = 1 }; };
template <> struct TypeTraits<unsigned short> { enum { isTrivial = 1 }; };
template <> struct TypeTraits<int>            { enum { isTrivial = 1 }; };
template <> struct TypeTraits<unsigned int>   { enum { isTrivial = 1 }; };
template <> struct TypeTraits<long>           { enum { isTrivial = 1 }; };
template <> struct TypeTraits<unsigned long>  { enum { isTrivial = 1 }; };

template <> struct TypeTraits<float>          { enum { isTrivial = 1 }; };
template <> struct TypeTraits<double>         { enum { isTrivial = 1 }; };
template <> struct TypeTraits<long double>    { enum { isTrivial = 1 }; };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !TYPETRAITS_H_DEFINED
