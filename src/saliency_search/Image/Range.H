/*!@file Image/Range.H Manage min/max range information */

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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Range.H $
// $Id: Range.H 14664 2011-03-31 23:21:21Z dberg $
//

#ifndef IMAGE_RANGE_H_DEFINED
#define IMAGE_RANGE_H_DEFINED

#include <limits> // for std::numeric_limits
#include <string>

//! A class for managing min/max range information.
/*! This class is primarily intended to work with algorithms that
    manipulate the dynamic range of Image<T>'s. For example, it can help to
    specify how an image should have its current dynamic range remapped to
    a new range (see remapRange() in Image_MathOps.H). In addition, the
    Range<T> class provides an operator() that allows it to be used as a
    functor in generic algorithms for accumulating the min/max over a
    sequence of elements (as in rangeOf() in Image_MathOps.H). */
template <class T>
class Range
{
private:
  T mini, maxi;

public:
  //! Default constructor.
  /*! We initialize the object's min element with type T's <b>max</b>
      value, and we initialize the object's max element with type T's
      <b>min</b> value. This seeming paradoxical situation is in fact "The
      Right Thing" if we want to accumulate the min/max over a range of
      elements. The starting condition of the Range object should make the
      weakest possible assumption about the values that will be observed;
      and this weakest assumption is precisely to start with the min being
      as large as possible, and the max being as small as possible. */
  inline Range();

  //! Construct with given min and max values.
  inline Range(const T& mi, const T& ma);

  //! Get the current min value.
  const T& min() const { return mini; }

  //! Get the current max value.
  const T& max() const { return maxi; }

  //! Get the range (i.e. the difference of max minus min).
  T range() const { return maxi - mini; }

  //! Set the range's min value.
  void setMin(const T& mi) { mini = mi; }

  //! Set the range's max value.
  void setMax(const T& ma) { maxi = ma; }

  //! Update our min value if val is less than the current max.
  void mergeMin(const T& v);

  //! Update our max value if val is greater than the current max.
  void mergeMax(const T& val);

  //! Update our min or max value if val falls outside of the current range.
  void merge(const T& val);

  //! This operator() allows straightforward use in generic algorithms.
  /*! In particular, this is useful for accumulating the min/max over a
      range of elements. For example, rangeOf() could be implemented like
      this:

      \code
      Image<T> img;
      Range<T> rng = std::for_each(img.begin(), img.end(), Range<T>());
      \endcode

      We construct a default Range<T> object, which is copied and passed to
      for_each(), which in turn calls operator() on that range object for
      each element in the iterator sequence that is visited.
  */
  void operator()(const T& v);

  //! Merge other's min and max values into ours.
  void merge(const Range<T>& other);

  //! See if two Range are the same.
  bool operator==(const Range& x) const throw();

  //! See if two Range are different.
  bool operator!=(const Range& x) const throw();
};

//convert from a Range<T> to a string
template<typename T> std::string convertToString(const Range<T>& val);

//convert from a string to a Range<T>
template<typename T> void convertFromString(const std::string& str, Range<T>& val);


// ######################################################################
template <class T> inline
Range<T>::Range() :
  mini(std::numeric_limits<T>::max()),
  maxi(std::numeric_limits<T>::min())
{
  if (maxi > 0.0) maxi = -std::numeric_limits<T>::max();
}

// ######################################################################
template <class T> inline
Range<T>::Range(const T& mi, const T& ma) :
  mini(mi), maxi(ma)
{}

// ######################################################################
template <class T> inline
void Range<T>::mergeMin(const T& v) { if (v < mini) mini = v; }

// ######################################################################
template <class T> inline
void Range<T>::mergeMax(const T& v) { if (v > maxi) maxi = v; }

// ######################################################################
template <class T> inline
void Range<T>::merge(const T& v)
{
  mergeMin(v);
  mergeMax(v);
}

// ######################################################################
template <class T> inline
void Range<T>::operator()(const T& v)
{
  merge(v);
}

// ######################################################################
template <class T> inline
void Range<T>::merge(const Range<T>& other)
{
  mergeMin(other.min());
  mergeMax(other.max());
}

// ######################################################################
template <class T> inline 
bool Range<T>::operator==(const Range<T>& rhs) const throw()
{
  return ( (min() == rhs.min()) && (max() == rhs.max()) );
}

// ######################################################################
template <class T> inline 
bool Range<T>::operator!=(const Range<T>& rhs) const throw()
{
  return !operator==(rhs);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_RANGE_H_DEFINED
