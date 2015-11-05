/*!@file Image/ImageCache.C implements an image cache with running average */

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
// Primary maintainer for this file: Dirk Walther <walther@caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ImageCache.C $
// $Id: ImageCache.C 5736 2005-10-18 17:30:29Z rjpeters $
//

#include "Image/ImageCache.H"

#include "Util/Assert.H"
#include "Image/MathOps.H"
#include "Image/ColorOps.H"
#include "Image/Pixels.H"
#include "Image/PixelsInst.H"      // for PIX_INST macro

// ######################################################################
// ##### Implementation of ImageCache<T>
// ######################################################################
template <class T>
ImageCache<T>::ImageCache(uint maxSize)
  : itsMaxSize(maxSize)
{}

// ######################################################################
template <class T>
ImageCache<T>::~ImageCache()
{}

// ######################################################################
template <class T>
void ImageCache<T>::setMaxSize(const uint maxSize)
{
  itsMaxSize = maxSize;
  // truncate if necessary:
  popOffOld();
}

// ######################################################################
template <class T>
uint ImageCache<T>::getMaxSize() const
{ return itsMaxSize; }

// ######################################################################
template <class T>
void ImageCache<T>::push_back(const Image<T>& img)
{
  doWhenAdd(img);
  itsCache.push_back(img);
  popOffOld();
  return;
}

// ######################################################################
template <class T>
void ImageCache<T>::popOffOld()
{
  // now pop off old images
  if (itsMaxSize == 0) return;

  while (itsCache.size() > itsMaxSize)
    pop_front();

  return;
}

// ######################################################################
template <class T>
Image<T> ImageCache<T>::pop_front()
{
  ASSERT(!itsCache.empty());
  Image<T> ret = itsCache.front();
  doWhenRemove(ret);
  itsCache.pop_front();
  return ret;
}

// ######################################################################
template <class T>
const Image<T>& ImageCache<T>::back() const
{
  ASSERT(!itsCache.empty());
  return itsCache.back();
}

// ######################################################################
template <class T>
const Image<T>& ImageCache<T>::front() const
{
  ASSERT(!itsCache.empty());
  return itsCache.front();
}

// ######################################################################
template <class T>
const Image<T>& ImageCache<T>::getImage(const uint lev) const
{
  ASSERT(lev < itsCache.size());
  return itsCache[lev];
}

// ######################################################################
template <class T>
const Image<T>& ImageCache<T>::operator[](const uint lev) const
{ return getImage(lev); }

// ######################################################################
template <class T>
uint ImageCache<T>::size() const
{ return itsCache.size(); }

// ######################################################################
template <class T>
bool ImageCache<T>::empty() const
{ return itsCache.empty(); }

// ######################################################################
template <class T>
void ImageCache<T>::clear()
{
  // hopefully, overloads of pop_front() will take care of updating any
  // additional data members (like the sliding average image in
  // ImageCache):
  while(size()) this->pop_front();
}

// ######################################################################
template <class T>
void ImageCache<T>::doWhenAdd(const Image<T>& img)
{ }

// ######################################################################
template <class T>
void ImageCache<T>::doWhenRemove(const Image<T>& img)
{ }

// ######################################################################
// ##### Implementation of ImageCacheAvg<T>
// ######################################################################
template <class T>
ImageCacheAvg<T>::ImageCacheAvg()
  : ImageCache<T>(0)
{}

// ######################################################################
template <class T>
ImageCacheAvg<T>::ImageCacheAvg(uint maxSize)
  : ImageCache<T>(maxSize)
{}

// ######################################################################
template <class T>
Image<T> ImageCacheAvg<T>::mean() const
{
  return Image<T> (this->itsSumImg / this->itsCache.size());
}

// ######################################################################
template <class T>
Image<T> ImageCacheAvg<T>::absDiffMean(const Image<T>& img) const
{
  return absDiff(mean(), img);
}

// ######################################################################
template <class T>
Image<T> ImageCacheAvg<T>::clampedDiffMean(const Image<T>& img) const
{
  return clampedDiff(img, mean());
}

// ######################################################################
template <class T>
Image<typename promote_trait<T,float>::TP> ImageCacheAvg<T>::sum() const
{
  return itsSumImg;
}

// ######################################################################
template <class T>
void ImageCacheAvg<T>::doWhenAdd(const Image<T>& img)
{
  if (itsSumImg.initialized())
    {
      ASSERT(itsSumImg.isSameSize(img));
      itsSumImg += img;
   }
  else
    {
      itsSumImg = img;
    }
}

// ######################################################################
template <class T>
void ImageCacheAvg<T>::doWhenRemove(const Image<T>& img)
{
  ASSERT(itsSumImg.initialized());
  itsSumImg -= img;
}


// ######################################################################
// ##### Implementation of ImageCacheMinMax<T>
// ######################################################################
template <class T>
ImageCacheMinMax<T>::ImageCacheMinMax()
  : ImageCache<T>(0)
{}

// ######################################################################
template <class T>
ImageCacheMinMax<T>::ImageCacheMinMax(uint maxSize)
  : ImageCache<T>(maxSize)
{}

// ######################################################################
template <class T>
Image<T> ImageCacheMinMax<T>::getMax() const
{
  if (this->itsCache.size() == 0) return Image<T>(); // empty
  typename std::deque< Image<T> >::const_iterator
    itr = this->itsCache.begin(), end = this->itsCache.end();
  Image<T> ret = *itr++;
  while(itr != end) ret = takeMax(ret, *itr++);
  return ret;
}

// ######################################################################
template <class T>
Image<T> ImageCacheMinMax<T>::getMin() const
{
  if (this->itsCache.size() == 0) return Image<T>(); // empty
  typename std::deque< Image<T> >::const_iterator
    itr = this->itsCache.begin(), end = this->itsCache.end();
  Image<T> ret = *itr++;
  while(itr != end) ret = takeMin(ret, *itr++);
  return ret;
}

// ######################################################################
template <class T>
void ImageCacheMinMax<T>::doWhenAdd(const Image<T>& img)
{ }

// ######################################################################
template <class T>
void ImageCacheMinMax<T>::doWhenRemove(const Image<T>& img)
{ }

// ######################################################################

// See PixelsTypesDefine.H for information about PIX_INST

#ifdef INVT_INST_BYTE
PIX_INST(ImageCache, byte);
PIX_INST(ImageCacheAvg, byte);
template class ImageCacheMinMax<byte>;
#endif

#ifdef INVT_INST_INT16
PIX_INST(ImageCache, int16);
PIX_INST(ImageCacheAvg, int16);
template class ImageCacheMinMax<int16>;
#endif

#ifdef INVT_INST_INT32
PIX_INST(ImageCache, int32);
PIX_INST(ImageCacheAvg, int32);
template class ImageCacheMinMax<int32>;
#endif

#ifdef INVT_INST_FLOAT
PIX_INST(ImageCache, float);
PIX_INST(ImageCacheAvg, float);
template class ImageCacheMinMax<float>;
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
