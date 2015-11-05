/*!@file Image/PyramidCache.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/PyramidCache.H $
// $Id: PyramidCache.H 8199 2007-03-30 17:58:46Z rjpeters $
//

#ifndef IMAGE_PYRAMIDCACHE_H_DEFINED
#define IMAGE_PYRAMIDCACHE_H_DEFINED

#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/Pixels.H"

#include <pthread.h>

namespace rutz { class mutex_lock_class; }

template <class T>
class PyramidCache
{
public:
  PyramidCache();

  struct Item
  {
    Item();
    ~Item();

    /// Acquire an exclusive lock on the mutex so that we can set a new pyramid for the given image
    /** If we already have a pyramid for the given image, then the
        return value here will be false, indicating that there is no
        need to set the pyramid again.

        If we do need a new pyramid for the given image, then the
        return value will be true and the Lock will be filled in with
        lock information that must be passed back to endSet().

        Typical usage would look like this:

        \code
        rutz::mutex_lock_class lock;
        if (cache->gaussian5.beginSet(img, &lock))
        {
          ImageSet<float> pyr = doSomethingToComputePyramid(img);
          cache->gaussian5.endSet(img, pyr, &lock);
        }
        \endcode
    */
    bool beginSet(const Image<T>& img, rutz::mutex_lock_class* l);

    /// Install a new image/pyramid pair using a lock previous acquired from beginSet()
    void endSet(const Image<T>& img, const ImageSet<T>& pyr,
                rutz::mutex_lock_class* l);

    /// Returns a pointer to the cached pyramid for img, or null if there is no such cached pyramid
    const ImageSet<T>* get(const Image<T>& img) const;

  private:
    Item(const Item&);
    Item& operator=(const Item&);

    mutable pthread_mutex_t itsLock;
    Image<T> itsImg;
    ImageSet<T> itsPyr;
  };

  Item gaussian5;
  Item laplacian9;

private:
  PyramidCache(const PyramidCache&);
  PyramidCache& operator=(const PyramidCache&);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_PYRAMIDCACHE_H_DEFINED
