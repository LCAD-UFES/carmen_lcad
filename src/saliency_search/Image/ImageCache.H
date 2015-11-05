/*!@file Image/ImageCache.H implements an image cache with running average
 */
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/ImageCache.H $
// $Id: ImageCache.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef IMAGECACHE_H_DEFINED
#define IMAGECACHE_H_DEFINED

#include "Image/Image.H"
#include "Util/Promotions.H"

#include <deque>


// ######################################################################
//! base class for image caches that do computations on the fly
/*! This base class has no op doWhenAdd and doWhenRemove functions
  that should be overridden in classes derived from this one.*/
template <class T>
class ImageCache
{
public:
  //! Constructor
  /*! @param maxSize the maximum size of the cache. If this size is exceeded,
    images are popped off the front of the cache and disregarded for the
    computation of the mean. If maxSize = 0, the cache is not limited.*/
  ImageCache(uint maxSize = 0);

  //! Destructor
  virtual ~ImageCache();

  //! Set maximum number of images in the cache
  /*! If the cache currently has more images than the new specified
    maximum, it will be truncated to that maximum. */
  virtual void setMaxSize(const uint maxSize);

  //! Get maximum number of images in the cache
  virtual uint getMaxSize() const;

  //! add image to cache - if the cache gets to big, old images are popped off
  virtual void push_back(const Image<T>& img);

  //! pop the front Image (oldest) off the cache and return it
  virtual Image<T> pop_front();

  //! access the last Image (newest) in the queue
  virtual const Image<T>& back() const;

  //! access the first Image (oldest) in the queue
  virtual const Image<T>& front() const;

  //! Get image from a given level.
  virtual const Image<T>& getImage(const uint lev) const;

  //! Get image from a given level (shorthand for getImage()).
  virtual const Image<T>& operator[](const uint lev) const;

  //! return the current size of the cache
  /*! This may be smaller than the maximum size specified at
    construction or with setMaxSize(), if the cache is not full yet. */
  virtual uint size() const;

  //! true if the cache contains no elements
  virtual bool empty() const;

  //! clear the cache, preserving its maxSize
  virtual void clear();

protected:
  //! checks whether the cache has become too big and removes old entries
  virtual void popOffOld();

  //! called when an image is added - override in your derived classes!
  /*! in ImageCache, this function is no op*/
  virtual void doWhenAdd(const Image<T>& img);

  //! called when an image is removed - override in your derived classes
  /*! in ImageCache, this function is no op*/
  virtual void doWhenRemove(const Image<T>& img);

  //! the maximum size of images to be stored
  uint itsMaxSize;

  //! the cache of the images
  std::deque< Image<T> > itsCache;
};

// ######################################################################
//! image cache to compute the running average
template <class T>
class ImageCacheAvg : public ImageCache<T>
{
public:
  //! Uninitialized constructor
  /*! By default, cache size is not limited. A limit can be set later
    using setMaxSize(). */
  ImageCacheAvg();

  //! Constructor
  /*! @param maxSize the maximum size of the cache. If this size is exceeded,
    images are popped off the front of the cache and disregarded for the
    computation of the mean. If maxSize = 0, the cache is not limited.*/
  ImageCacheAvg(uint maxSize);

  //! return the mean of the images currently in the cache
  Image<T> mean() const;

  //! return abs(img - mean)
  Image<T> absDiffMean(const Image<T>& img) const;

  //! return (img - mean), clamped to zero where negative
  Image<T> clampedDiffMean(const Image<T>& img) const;

  //! returns the sum of all cached images
  /*! This will be of type Image<int> if T = byte, int16 or int32;
    of type float if T = float; of type Image< PixRGB<int> >
    if T = PixRGB<byte>, PixRGB<int16> or PixRGB<int32>;
    and of type Image< PixRGB<float> > if T = PixRGB<float> */
  Image<typename promote_trait<T,float>::TP> sum() const;

protected:
  //! called when an image is added - add image to the sum
  virtual void doWhenAdd(const Image<T>& img);

  //! called when an image is removed - subtract image from the sum
  virtual void doWhenRemove(const Image<T>& img);

  //! the sum of all the cached images
  Image<typename promote_trait<T,float>::TP> itsSumImg;
};

// ######################################################################
//! image cache to compute a running min/max
/*! In this implementation, all the computations are done when
  getMax() or getMin() are called. That is, the cache is fairly dump
  and does not attempt to keep an updated min and max image as new
  images get added or removed. This differs substantially from the
  ImageCacheAvg implementation, where all computations are done as images
  are added/removed and getting the mean is a very cheap operation. */
template <class T>
class ImageCacheMinMax : public ImageCache<T>
{
public:
  //! Uninitialized constructor
  /*! By default, cache size is not limited. A limit can be set later
    using setMaxSize(). */
  ImageCacheMinMax();

  //! Constructor
  /*! @param maxSize the maximum size of the cache. If this size is exceeded,
    images are popped off the front of the cache and disregarded for the
    computation of the mean. If maxSize = 0, the cache is not limited.*/
  ImageCacheMinMax(uint maxSize);

  //! return the max of the images currently in the cache
  Image<T> getMax() const;

  //! return the max of the images currently in the cache
  Image<T> getMin() const;

protected:
  //! called when an image is added - here it's a nop-op
  virtual void doWhenAdd(const Image<T>& img);

  //! called when an image is removed - here it's a no-op
  virtual void doWhenRemove(const Image<T>& img);
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
