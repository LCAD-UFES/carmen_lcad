/*!@file Image/NamedImage.H An Image with a name */

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
// Primary maintainer for this file: Laurent Itti <itti@pollux.usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/NamedImage.H $
// $Id: NamedImage.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef IMAGE_NAMEDIMAGE_H_DEFINED
#define IMAGE_NAMEDIMAGE_H_DEFINED

#include "Image/Image.H"
#include "rutz/shared_ptr.h"
#include <string>
//! NamedImage is an Image with a name. It inherits all functionality of Image.
/*! A copy of the name string is made at construction, and is held in
    a shared_ptr, so that further copying NamedImage objects arround
    will not copy again the name string. */
template <class T> class NamedImage : public Image<T> {
public:
  //! Construct from an existing Image, possibly converting along the way
  template <class T2> inline NamedImage(const Image<T2>& img, const std::string& name);

  //! Construct from an existing Image, possibly converting along the way
  template <class T2> inline NamedImage(const Image<T2>& img, const char *name);

  //! Construct an empty (0-by-0) image (useful for arrays of Images)
  inline NamedImage(const std::string& name);

  //! Construct an empty (0-by-0) image (useful for arrays of Images)
  inline NamedImage(const char *name = "");

  //! Conversion and copy constructor
  template <class T2> inline NamedImage(const NamedImage<T2>& A);

  //! Conversion and assigment operator
  template <class T2> inline NamedImage<T>& operator=(const NamedImage<T2>& A);

  //! Assign from an image, preserving our name and doing a clamped convert
  /*! This is potentially dangerous, and mostly provided so that we
    can inherit all of the Image free functions that return images
    (e.g., those of Image/MathOps.H). */
  template <class TT> inline NamedImage<T>& operator=(const Image<TT>& img);

  //! Destructor
  virtual inline ~NamedImage();

  //! Get the name
  inline const std::string& name() const;

  //! Set the name post hoc
  inline void setName(const std::string& name);

  //! Set the name post hoc
  inline void setName(const char *name);

  //! Access our pointer to our name
  inline const rutz::shared_ptr<std::string>& namePtr() const;

private:
  rutz::shared_ptr<std::string> itsName;
};

// ######################################################################
// Inline function implementation
// ######################################################################

template <class T> template <class T2> inline
NamedImage<T>::NamedImage(const Image<T2>& img, const std::string& name) :
  Image<T>(img), itsName(new std::string(name))
{ }

template <class T> template <class T2> inline
NamedImage<T>::NamedImage(const Image<T2>& img, const char *name) :
  Image<T>(img), itsName(new std::string(name))
{ }

template <class T> inline
NamedImage<T>::NamedImage(const std::string& name) :
  Image<T>(), itsName(new std::string(name))
{ }

template <class T> inline
NamedImage<T>::NamedImage(const char *name) :
  Image<T>(), itsName(new std::string(name))
{ }

template <class T> template <class T2> inline
NamedImage<T>::NamedImage(const NamedImage<T2>& A) :
  Image<T>(A), itsName(A.namePtr())
{ }

template <class T> template <class T2> inline
NamedImage<T>& NamedImage<T>::operator=(const NamedImage<T2>& A)
{
  Image<T>::operator=(A);  // copy the image data
  itsName = A.namePtr(); // get a handle to the name
  return *this;
}

template <class T> template <class TT> inline
NamedImage<T>& NamedImage<T>::operator=(const Image<TT>& img)
{
  // our name does not change
  Image<T>::operator=(img); return *this;
}

template <class T> inline
NamedImage<T>::~NamedImage()
{ }

template <class T> inline
const std::string& NamedImage<T>::name() const
{ return *(itsName.get()); }

template <class T> inline
void NamedImage<T>::setName(const std::string& name)
{ itsName.reset(new std::string(name)); }

template <class T> inline
void NamedImage<T>::setName(const char *name)
{ itsName.reset(new std::string(name)); }

template <class T> inline
const rutz::shared_ptr<std::string>& NamedImage<T>::namePtr() const
{ return itsName; }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_NAMEDIMAGE_H_DEFINED
