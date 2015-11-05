/*!@file Image/CheckedIterator.H A range-checked iterator */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/CheckedIterator.H $
// $Id: CheckedIterator.H 6352 2006-03-07 19:54:28Z rjpeters $
//

#ifndef CHECKEDITERATOR_H_DEFINED
#define CHECKEDITERATOR_H_DEFINED

#include <assert.h>
#include <stddef.h>

#include <iterator>

namespace CheckedIteratorAux
{
  void ck_range_helper(const void* ptr, size_t offset,
                       const void* start, const void* stop,
                       size_t size);
}

typedef unsigned long diff_t;

//! A range-checked iterator class for memory debugging.
/*! A CheckedIterator<T> object exposes essentially the same interface
  as a T*, but stores a start/stop range that should bound the
  iterator. This range is checked with a call to ck_range() every time
  the iterator is derefenced, by either operator*() or
  operator->(). */

template<class T>
class CheckedIterator
{
public:
  //! This typedef is used by STL functions
  typedef T value_type;

  //! This typedef is used by STL functions
  typedef T* pointer;

  //! This typedef is used by STL functions
  typedef T& reference;

  //! This typedef is used by STL functions
  typedef ptrdiff_t difference_type;

  //! This typedef is used by STL functions
  typedef std::random_access_iterator_tag iterator_category;

  //! Default constructor
  /*! This builds an iterator with an empty valid range, so a new value
      must be assigned to the iterator before it can be
      dereferenced. However, providing a default constructor helps keep
      source compatibility with non-debug iterators (i.e., raw
      pointers). */
  inline CheckedIterator();
  //! Constructor
  inline CheckedIterator(T* p, T* a, T* b);
  //! Constructor
  template<class UU> inline CheckedIterator(const CheckedIterator<UU>& other);

  /*! @name Pointer arithmetic operators */
  //@{

  //! Pre-increment operator.
  inline CheckedIterator<T>& operator++();

  //! Post-increment operator.
  inline CheckedIterator<T> operator++(int);

  //! Pre-decrement operator.
  inline CheckedIterator<T>& operator--();

  //! Post-decrement operator.
  inline CheckedIterator<T> operator--(int);

  inline void operator+=(diff_t d);
  inline void operator-=(diff_t d);

  inline CheckedIterator<T> operator+(diff_t d) const;
  inline CheckedIterator<T> operator-(diff_t d) const;

  inline diff_t operator-(CheckedIterator<T> other) const;

  //@}

  /*! @name Comparison operators */
  //@{

  template <class U> inline bool operator==(const CheckedIterator<U>& other) const
  { return this->ptr == other.ptr; }
  template <class U> inline bool operator!=(const CheckedIterator<U>& other) const
  { return this->ptr != other.ptr; }
  template <class U> inline bool operator<(const CheckedIterator<U>& other) const
  { return this->ptr < other.ptr; }
  template <class U> inline bool operator>(const CheckedIterator<U>& other) const
  { return this->ptr > other.ptr; }
  template <class U> inline bool operator<=(const CheckedIterator<U>& other) const
  { return this->ptr <= other.ptr; }
  template <class U> inline bool operator>=(const CheckedIterator<U>& other) const
  { return this->ptr >= other.ptr; }

  //@}

  /*! @name Dereferencing operators */
  //@{

  inline T* operator->() const;
  inline T& operator*() const;
  inline T& operator[](diff_t d) const;

  //@}

  // ##### data members:

  //! Pointer to the data
  T* ptr;

  //! Pointer to the first element we can iterate on
  T* start;

  //! Pointer to the last element we can iterete on
  T* stop;
  inline void ck_range(const diff_t offset) const;
};

// ######################################################################
// ######################################################################

// ######################################################################
template<class T> inline
CheckedIterator<T>::CheckedIterator() :
  ptr(0), start(0), stop(0)
{}

// ######################################################################
template<class T> inline
CheckedIterator<T>::CheckedIterator(T* p, T* a, T* b) :
  ptr(p), start(a), stop(b)
{}

// ######################################################################
template<class T> template<class UU> inline
CheckedIterator<T>::CheckedIterator(const CheckedIterator<UU>& other) :
  ptr(other.ptr), start(other.start), stop(other.stop)
{}

// ######################################################################
template<class T> inline
CheckedIterator<T>& CheckedIterator<T>::operator++()
{ ++ptr; return *this; }

// ######################################################################
template<class T> inline
CheckedIterator<T> CheckedIterator<T>::operator++(int)
{ return CheckedIterator(ptr++, start, stop); }

// ######################################################################
template<class T> inline
CheckedIterator<T>& CheckedIterator<T>::operator--()
{ --ptr; return *this; }

// ######################################################################
template<class T> inline
CheckedIterator<T> CheckedIterator<T>::operator--(int)
{ return CheckedIterator(ptr--, start, stop); }

// ######################################################################
template<class T> inline
void CheckedIterator<T>::operator+=(diff_t d)
{ ptr += d; }

// ######################################################################
template<class T> inline
void CheckedIterator<T>::operator-=(diff_t d)
{ ptr -= d; }

// ######################################################################
template<class T> inline
CheckedIterator<T> CheckedIterator<T>::operator+(diff_t d) const
{ return CheckedIterator(ptr+d, start, stop); }

// ######################################################################
template<class T> inline
CheckedIterator<T> CheckedIterator<T>::operator-(diff_t d) const
{ return CheckedIterator(ptr-d, start, stop); }

// ######################################################################
template<class T> inline
diff_t CheckedIterator<T>::operator-(CheckedIterator<T> other) const
{ return ptr - other.ptr; }

// ######################################################################
template<class T> inline
T* CheckedIterator<T>::operator->() const
{ ck_range(0); return ptr; }

// ######################################################################
template<class T> inline
T& CheckedIterator<T>::operator*() const
{ ck_range(0); return *ptr; }

// ######################################################################
template<class T> inline
T& CheckedIterator<T>::operator[](diff_t d) const
{ ck_range(d); return ptr[d]; }

// ######################################################################
template<class T> inline
void CheckedIterator<T>::ck_range(const diff_t offset) const
{ CheckedIteratorAux::ck_range_helper(ptr, offset,
                                      start, stop, sizeof(T)); }

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
