/*!@file Image/MorphOps.C functions for binary morphology (dilate, erode, open, close) */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/MorphOps.C $
// $Id: MorphOps.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_MORPHOPS_C_DEFINED
#define IMAGE_MORPHOPS_C_DEFINED

#include "Image/MorphOps.H"

#include "Image/Image.H"
#include "rutz/trace.h"

// ######################################################################
template <class T>
Image<T> dilateImg(const Image<T>& img, const Image<T>& se, Point2D<int> origin)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img.initialized() && se.initialized());

  int iw = img.getWidth(); int ih = img.getHeight();
  int sw =  se.getWidth(); int sh =  se.getHeight();

  if (origin == Point2D<int>(-1,-1))
    {
      origin.i = (sw - 1) / 2;
      origin.j = (sh - 1) / 2;
    }
  ASSERT((origin.i < sw) && (origin.j < sh));

  Image<T> result(iw,ih,ZEROS);

  typename Image<T>::const_iterator iptr, sptr1, sptr2;
  typename Image<T>::iterator rptr1, rptr2;
  iptr = img.begin();

  for (int iy = 0; iy < ih; ++iy)
    {
      // initialize height
      int rYstart = iy - origin.j;
      int rYend = rYstart + sh;
      int sYstart = 0;
      if (rYstart < 0)
        {
          sYstart -= rYstart;
          rYstart = 0;
        }
      if (rYend > ih) rYend = ih;

      for (int ix = 0; ix < iw; ++ix)
        {
          // do we have to do anything?
          if (*iptr != T())
            {
              //initialize pointers and width
              int rXstart = ix - origin.i;
              int rXend = rXstart + sw;
              int sXstart = 0;
              if (rXstart < 0)
                {
                  sXstart -= rXstart;
                  rXstart = 0;
                }
              if (rXend > iw) rXend = iw;

              rptr1 = result.beginw() + rYstart * iw + rXstart;
              sptr1 = se.begin() + sYstart * sw + sXstart;

              for (int ry = rYstart; ry < rYend; ++ry)
                {
                  rptr2 = rptr1; sptr2 = sptr1;
                  for (int rx = rXstart; rx < rXend; ++rx)
                    {
                      *rptr2 = std::max(*rptr2,*sptr2);
                      ++rptr2; ++sptr2;
                    }
                  rptr1 += iw; sptr1 += sw;
                }
            }  // end: we have to do something
          ++iptr;
        }  // end: for ix
    }  // end: for iy
  return result;
}

// ######################################################################
template <class T>
Image<T> erodeImg(const Image<T>& img, const Image<T>& se, Point2D<int> origin)
{
GVX_TRACE(__PRETTY_FUNCTION__);
  ASSERT(img.initialized() && se.initialized());

  int iw = img.getWidth(); int ih = img.getHeight();
  int sw =  se.getWidth(); int sh =  se.getHeight();

  if (origin == Point2D<int>(-1,-1))
    {
      origin.i = (sw - 1) / 2;
      origin.j = (sh - 1) / 2;
    }
  ASSERT((origin.i < sw) && (origin.j < sh));

  Image<T> result(iw,ih,ZEROS);

  typename Image<T>::const_iterator iptr1, iptr2, sptr1, sptr2;
  typename Image<T>::iterator rptr = result.beginw();
  T se_orig_val = se.getVal(origin);

  //loop over result X
  for (int ry = 0; ry < ih; ++ry)
    {
      // initialize height
      int iYstart = ry - origin.j;
      int iYend = iYstart + sh;
      int sYstart = 0;
      if (iYstart < 0)
        {
          sYstart -= iYstart;
          iYstart = 0;
        }
      if (iYend > ih) iYend = ih;


      // loop over result Y
      for (int rx = 0; rx < iw; ++rx)
        {
          int iXstart = rx - origin.i;
          int iXend = iXstart + sw;
          int sXstart = 0;
          if (iXstart < 0)
            {
              sXstart -= iXstart;
              iXstart = 0;
            }
          if (iXend > iw) iXend = iw;

          bool flag = true;   // reset the flag
          iptr1 = img.begin() + iYstart * iw + iXstart;
          sptr1 = se.begin() + sYstart * sw + sXstart;

          //loop over the image covered by the structuring element
          for (int iy = iYstart; iy < iYend; ++iy)
            {

              iptr2 = iptr1; sptr2 = sptr1;
              for (int ix = iXstart; ix < iXend; ++ ix)
                {
                  if ((*sptr2 != 0) && (*iptr2 == 0))
                    {
                      flag = false;
                      break;
                    } // end: if (*iptr2 == 0)
                  ++iptr2; ++sptr2;
                } // end: for ix

              if (!flag) break;
              iptr1 += iw; sptr1 += sw;

            } // end: for iy

          // should we set the pixel?
          if (flag) *rptr = std::max(*rptr, se_orig_val);

          ++rptr;

        } // end: for rx
    } // end: for ry
  return result;
}

// ######################################################################
template <class T>
Image<T> openImg(const Image<T>& img, const Image<T>& se, Point2D<int> origin)
{
  return dilateImg(erodeImg(img,se,origin),se,origin);
}

// ######################################################################
template <class T>
Image<T> closeImg(const Image<T>& img, const Image<T>& se, Point2D<int> origin)
{
  return erodeImg(dilateImg(img,se,origin),se,origin);
}

// Include the explicit instantiations
#include "inst/Image/MorphOps.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_MORPHOPS_C_DEFINED
