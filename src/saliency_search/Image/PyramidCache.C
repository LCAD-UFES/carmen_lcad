/*!@file Image/PyramidCache.C */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PyramidCache.C $
// $Id: PyramidCache.C 8199 2007-03-30 17:58:46Z rjpeters $
//

#ifndef IMAGE_PYRAMIDCACHE_C_DEFINED
#define IMAGE_PYRAMIDCACHE_C_DEFINED

#include "Image/PyramidCache.H"

#include "rutz/mutex.h"

// ######################################################################
template <class T>
PyramidCache<T>::PyramidCache()
  :
  gaussian5(),
  laplacian9()
{}

// ######################################################################
template <class T>
PyramidCache<T>::Item::Item()
  : itsImg(), itsPyr()
{
  pthread_mutex_init(&itsLock, NULL);
}

// ######################################################################
template <class T>
PyramidCache<T>::Item::~Item()
{
  pthread_mutex_destroy(&itsLock);
}

// ######################################################################
template <class T>
bool PyramidCache<T>::Item::beginSet(const Image<T>& img,
                                     rutz::mutex_lock_class* l)
{
  ASSERT(!l->is_locked());

  rutz::mutex_lock_class l2(&itsLock);

  if (itsPyr.size() != 0 && itsImg.hasSameData(img))
    // we already have a pyramid cached for this image, so there is no
    // need for the caller to regenerate it:
    return false;

  // ok, we don't have a cached pyramid for the given image, so pass
  // the lock to the caller and let them regenerate it
  l->swap(l2);
  ASSERT(l->is_locked());
  return true;
}

// ######################################################################
template <class T>
void PyramidCache<T>::Item::endSet(const Image<T>& img, const ImageSet<T>& pyr,
                                   rutz::mutex_lock_class* l)
{
  ASSERT(l->is_locked());

  itsImg = img;
  itsPyr = pyr;

  // release the lock:
  {
    rutz::mutex_lock_class l2;
    l2.swap(*l);
    l2.unlock();
  }

  ASSERT(!l->is_locked());
}

// ######################################################################
template <class T>
const ImageSet<T>* PyramidCache<T>::Item::get(const Image<T>& img) const
{
  GVX_MUTEX_LOCK(&itsLock);

  if (itsPyr.size() == 0 || !itsImg.hasSameData(img))
    return 0;

  return &itsPyr;
}

// ######################################################################
// Explicit instantiations:

template class PyramidCache<PixRGB<byte> >;
template class PyramidCache<PixRGB<float> >;
template class PyramidCache<byte>;
template class PyramidCache<float>;
template class PyramidCache<int>;

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_PYRAMIDCACHE_C_DEFINED
