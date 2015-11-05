/*!@file Image/Conversions.H convert between Image and other types
  (e.g. std::vector) */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Conversions.H $
// $Id: Conversions.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef IMAGE_CONVERSIONS_H_DEFINED
#define IMAGE_CONVERSIONS_H_DEFINED

#include "Util/Assert.H"
#include "Image/Image.H"

#include <vector>


//! Conversion assigment operator.
/*! e.g.:
  \code
  Image<byte> im1; std::vector im2; vectorToImage(im2, im1);
  \endcode

*/

template <class T>
inline Image<T> vectorToImage(const std::vector<std::vector<T> >& A);

template <class T>
inline Image<T> vectorToImage(const std::vector<std::vector<T>* >& A);

template <class T>
inline Image<T> vectorToImage(const std::vector<std::vector<T*> >& A);

//! return a double nested vector that represents data (see also =operator)
template <class T>
inline void getVector(const Image<T>& src,
                      std::vector<std::vector<T> > *V);

//! return a double nested vector that represents data (see also =operator)
template <class T>
inline void getVector(const Image<T>& src,
                      std::vector<std::vector<T*> > *V);

//! return a single row of an image as a vector
template <class T>
inline void getVectorRow(const Image<T>& src,
                         std::vector<T> *V, int row);

//! return a single row of an image as a vector
template <class T>
inline void getVectorRow(const Image<T>& src,
                         std::vector<T*> *V, int row);

//! return a single column of an image as a vector
template <class T>
inline void getVectorColumn(const Image<T>& src,
                            std::vector<T> *V, int column);

//! return a single column of an image as a vector
template <class T>
inline void getVectorColumn(const Image<T>& src,
                            std::vector<T*> *V, int column);

// ######################################################################
// ##### Inline function definitions
// ######################################################################

// ######################################################################
template <class T> inline
Image<T> vectorToImage(const std::vector<std::vector<T> >& src)
{
  Image<T> dst((signed)src[0].size(),(signed)src.size(), NO_INIT);

  typename Image<T>::iterator aptr = dst.beginw();
  typename std::vector<std::vector<T> >::const_iterator vptr = src.begin();
  typename std::vector<std::vector<T> >::const_iterator vstp = src.end();
  while(vptr != vstp)
  {
    typename std::vector<T>::const_iterator vvptr = vptr->begin();
    typename std::vector<T>::const_iterator vvstp = vptr->end();
    while(vvptr != vvstp)
      *aptr++ = *vvptr++;
    ++vptr;
  }

  return dst;
}

// ######################################################################
template <class T> inline
Image<T> vectorToImage(const std::vector<std::vector<T>* >& src)
{
  Image<T> dst((signed)src[0]->size(),(signed)src.size(), NO_INIT);

  typename Image<T>::iterator aptr = dst.beginw();
  for(int ii = 0; ii < (signed)src.size(); ii++)
  {
    //std::vector<T>* temp = src[ii];
    typename std::vector<T>::iterator vvptr = src[ii]->begin();
    typename std::vector<T>::iterator vvstp = src[ii]->end();
    while(vvptr != vvstp)
      *aptr++ = *vvptr++;
  }

  return dst;
}

// ######################################################################
template <class T> inline
Image<T> vectorToImage(const std::vector<std::vector<T*> >& src)
{
  Image<T> dst((signed)src[0].size(),(signed)src.size(), NO_INIT);

  typename Image<T>::iterator aptr = dst.beginw();
  typename std::vector<std::vector<T*> >::const_iterator vptr = src.begin();
  typename std::vector<std::vector<T*> >::const_iterator vstp = src.end();
  while(vptr != vstp)
  {
    typename std::vector<T*>::iterator vvptr = vptr->begin();
    typename std::vector<T*>::iterator vvstp = vptr->end();
    while(vvptr != vvstp)
      *aptr++ = **vvptr++;
    ++vptr;
  }

  return dst;
}

// ######################################################################
template <class T> inline
void getVector(const Image<T>& src, std::vector<std::vector<T> > *V)
{
  ASSERT(V->size() >= (unsigned int)src.getHeight());
  ASSERT(V->at(0).size() >= (unsigned int)src.getWidth());
  typename Image<T>::const_iterator aptr = src.begin();
  typename std::vector<std::vector<T> >::iterator vptr = V->begin();
  typename std::vector<std::vector<T> >::iterator vstp = V->end();
  while(vptr != vstp)
    {
      typename std::vector<T>::iterator vvptr = vptr->begin();
      typename std::vector<T>::iterator vvstp = vptr->end();
      while(vvptr != vvstp)
        *vvptr++ = *aptr++;
    }
}

// ######################################################################
template <class T> inline
void getVector(const Image<T>& src, std::vector<std::vector<T*> > *V)
{
  ASSERT(V->size() >= (unsigned int)src.getHeight());
  ASSERT(V->at(0).size() >= (unsigned int)src.getWidth());
  typename Image<T>::const_iterator aptr = src.begin();
  typename std::vector<std::vector<T*> >::iterator vptr = V->begin();
  typename std::vector<std::vector<T*> >::iterator vstp = V->end();
  while(vptr != vstp)
    {
      typename std::vector<T*>::iterator vvptr = vptr->begin();
      typename std::vector<T*>::iterator vvstp = vptr->end();
      while(vvptr != vvstp)
        **vvptr++ = *aptr++;
    }
}

// ######################################################################
template <class T> inline
void getVectorRow(const Image<T>& src, std::vector<T> *V, int row)
{
  ASSERT(V->size() >= (unsigned int)src.getWidth());
  typename Image<T>::const_iterator aptr =
    src.begin() + (row * src.getWidth());
  typename std::vector<T>::iterator vptr = V->begin();
  typename std::vector<T>::iterator vstp = V->end();
  while(vptr != vstp)
    {
      *vptr++ = *aptr++;
    }
}

// ######################################################################
template <class T> inline
void getVectorRow(const Image<T>& src, std::vector<T*> *V, int row)
{
  ASSERT(V->size() >= (unsigned int)src.getWidth());
  typename Image<T>::const_iterator aptr =
    src.begin() + (row * src.getWidth());

  typename std::vector<T*>::iterator vptr = V->begin();
  typename std::vector<T*>::iterator vstp = V->end();
  while(vptr != vstp)
    {
      **vptr++ = *aptr++;
    }
}

// ######################################################################
template <class T> inline
void getVectorColumn(const Image<T>& src, std::vector<T> *V, int column)
{
  ASSERT(V->size() >= (unsigned int)src.getHeight());
  typename std::vector<T>::iterator vptr = V->begin();
  typename std::vector<T>::iterator vstp = V->end();
  int current = 0;
  while(vptr != vstp)
    {
      *vptr++ = src.begin()[column + current*src.getWidth()];
      current++;
    }
}

// ######################################################################
template <class T> inline
void getVectorColumn(const Image<T>& src, std::vector<T*> *V, int column)
{
  ASSERT(V->size() >= (unsigned int)src.getHeight());
  typename std::vector<T*>::iterator vptr = V->begin();
  typename std::vector<T*>::iterator vstp = V->end();
  int current = 0;
  while(vptr != vstp)
    {
      **vptr++ = src.begin()[column + current*src.getWidth()];
      current++;
    }
}


#endif // !IMAGE_CONVERSIONS_H_DEFINED
