/*!@file Util/safecopy.H Provides a "safe" replacement for memcpy:
  when INVT_MEM_DEBUG is not defined, we just delegate to memcpy(),
  however when INVT_MEM_DEBUG is defined we use a generic iterator
  version, which allows the use of a range-checked iterator (such as
  CheckedIterator) for tracking down memory-access related bugs.
 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/safecopy.H $
// $Id: safecopy.H 5736 2005-10-18 17:30:29Z rjpeters $
//

#ifndef SAFECOPY_H_DEFINED
#define SAFECOPY_H_DEFINED

#include <cstring>

#include "Util/TypeTraits.H"

//! Element-by-element copy a series forward iterators.
template <class Iter, class ConstIter>
inline void genericcopy(Iter dest, ConstIter src, unsigned int count)
{
  for (unsigned int i = 0; i < count; ++i)
    *dest++ = *src++;
}

//! A generic version of safecopy() for iterators.
template <class Iter, class ConstIter>
inline void safecopy(Iter dest, ConstIter src, unsigned int count)
{
  genericcopy(dest, src, count);
}

//! Helper template struct for pointer version of safecopy().
template <typename T, bool isTrivial = TypeTraits<T>::isTrivial>
struct SafeCopyHelper
{
  static void copy(T* dest, const T* src, unsigned int n)
  { genericcopy(dest, src, n); }
};

//! Specialized implementation for trivial types.
template <typename T>
struct SafeCopyHelper<T, true>
{
  static void copy(T* dest, const T* src, unsigned int n)
  { memcpy(dest, src, n * sizeof(T)); }
};

//! A specialized version of safecopy() for pointers.
/*! This version uses memcpy() internally for maximum efficiency if T is a
    "trivial" type, otherwise it uses genericcopy(). <b>Note however</b>
    that safecopy's third parameter is number-of-elements, <b>NOT</b>
    number of bytes (i.e., unlike memcpy). */
template <class T>
inline void safecopy(T* dest, const T* src, unsigned int count)
{
  SafeCopyHelper<T>::copy(dest, src, count);
}

#endif // !SAFECOPY_H_DEFINED

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
