/*!@file ModelNeuron/SVFilter.C*/

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
// Primary maintainer for this file: David J. Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/ModelNeuron/SVFilter.C $

#include "ModelNeuron/SVFilter.H"

#include "Image/Image.H"
#include "Image/Pixels.H"
#include "rutz/trace.h"

// ######################################################################
template <class T>
static Image<typename promote_trait<T, float>::TP>
xFilterSV(const Image<T>& src, const float* hFilt, const int hfs)
{
  GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();

  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (hfs == 0) return source;  // no filter

  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  const int hfs2 = (hfs - 1) / 2;

  // flip the filter to accelerate convolution:
  float *hFilter = new float[hfs]; float sumh = 0.0f;
  for (int x = 0; x < hfs; x ++)
  { sumh += hFilt[x]; hFilter[hfs-1-x] = hFilt[x]; }

  // *** horizontal pass ***
  if (w < hfs)  // special function for very small images
  {
    for (int j = 0; j < h; j ++)
      for (int i = 0; i < w; i ++)
      {
        TF val = TF(); float sum = 0.0F;
        for (int k = 0; k < hfs; k ++)
          if (i + k - hfs2 >= 0 && i + k - hfs2 < w)
          {
            val += sptr[k - hfs2] * hFilter[k];
            sum += hFilter[k];
          }
        *dptr++ = val * sumh / sum; sptr ++;
      }
  }
  else  // function for reasonably large images
    for (int jj = 0; jj < h; jj ++)
    {
 
      //left boarder
      for (int j = hfs2; j > 0; --j)
      {
        TF val = TF(); float sum = 0.0F;
        for (int k = j; k < hfs; ++k)
        {
          val += sptr[k - j] * hFilter[k];
          sum += hFilter[k];
        }
        *dptr++ = val * sumh / sum;
      }

      // bulk (far from the borders):
      for (int i = 0; i < w - hfs + 1; i ++)
      {
        TF val = TF();
        for (int k = 0; k < hfs; k ++) val += sptr[k] * hFilter[k];
        *dptr++ = val; sptr++;
      }

      // right boarder
      for (int j = hfs - 1; j > hfs2; --j)
      {
        TF val = TF(); float sum = 0.0F;
        for (int k = 0; k < j; ++k)
        {
          val += sptr[k] * hFilter[k];
          sum += hFilter[k];
        }
        *dptr++ = val * sumh / sum;
        sptr++;
      }

      sptr += hfs2;  // sptr back to same as dptr (start of next line)
    }
  delete [] hFilter;
  return result;
}

// ######################################################################
template <class T>
static Image<typename promote_trait<T, float>::TP>
yFilterSV(const Image<T>& src, const float* vFilt, const int vfs)
{
  GVX_TRACE(__PRETTY_FUNCTION__);
  const int w = src.getWidth(), h = src.getHeight();
  // promote the source image to float if necessary, so that we do the
  // promotion only once for all, rather than many times as we access
  // the pixels of the image; if no promotion is necessary, "source"
  // will just point to the original data of "src" through the
  // copy-on-write/ref-counting behavior of Image:
  typedef typename promote_trait<T, float>::TP TF;
  const Image<TF> source = src;
  if (vfs == 0) return source;  // no filter

  Image<TF> result(w, h, NO_INIT);
  typename Image<TF>::const_iterator sptr = source.begin();
  typename Image<TF>::iterator dptr = result.beginw();

  const int vfs2 = (vfs - 1) / 2;

  // flip the filter to accelerate convolution:
  float *vFilter = new float[vfs]; float sumv = 0.0f;
  for (int x = 0; x < vfs; x ++)
  { sumv += vFilt[x]; vFilter[vfs-1-x] = vFilt[x]; }

  int ww[vfs];
  for (int i = 0; i < vfs; i ++) ww[i] = w * i; // speedup precompute

  if (h < vfs)  // special function for very small images
  {
    for (int j = 0; j < h; j ++)
      for (int i = 0; i < w; i ++)
      {
        TF val = TF(); float sum = 0.0;
        for (int k = 0; k < vfs; k ++)
          if (j + k - vfs2 >= 0 && j + k - vfs2 < h)
          {
            val += sptr[w * (k - vfs2)] * vFilter[k];
            sum += vFilter[k];
          }
        *dptr++ = val * sumv / sum; sptr ++;
      }
  }
  else  // function for reasonably large images
  {
    //top point
    for (int j = vfs2; j > 0; --j)
    {
      for (int i = 0; i < w; ++i)  
      {
        TF val = TF(); float sum = 0.0;

        //first current part of hemifield
        for (int k = j; k < vfs; k ++)
        {
          val += sptr[ww[k - j]] * vFilter[k];
          sum += vFilter[k];
        }

        //from oposite hemifield
        for (int k = 0; k < j; k ++)
        {
          val += sptr[w - 1 - i + ww[k]] * vFilter[vfs2 - 1 - k];
          sum += vFilter[vfs2 - 1 - k];
        }

        *dptr++ = val * sumv / sum;
        sptr++;
      }
      sptr -= w;   // back to top-left corner
    }

    // bulk (far from edges):
    for (int j = 0; j < h - vfs + 1; j ++)
      for (int i = 0; i < w; i ++)
      {
        TF val = TF();
        for (int k = 0; k < vfs; k ++) val += sptr[ww[k]] * vFilter[k];
        *dptr++ = val; sptr ++;
      }

    //botom points
    for (int j = vfs - 1; j > vfs2; --j)
      for (int i = 0; i < w; ++i)
      {
        TF val = TF(); float sum = 0.0;

        //from current hemifield
        for (int k = 0; k < j; ++k)
        {
          val += sptr[ww[k]] * vFilter[k];
          sum += vFilter[k];
        }

        //from opposite hemifield
        for (int k = j; k < vfs; ++k)
        {
          val += sptr[w - 1 - i + ww[vfs - k]] * vFilter[k];
          sum += vFilter[k];
        }      

        *dptr++ = val * sumv / sum;
        sptr ++;
      }
  }

  delete [] vFilter;
  return result;
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
sepFilterSV(const Image<T>& src, const Image<float>& hFilter,
            const Image<float>& vFilter)
{
  ASSERT(hFilter.is1D() || hFilter.getSize() == 0);
  ASSERT(vFilter.is1D() || vFilter.getSize() == 0);
  return sepFilterSV(src, hFilter.getArrayPtr(), vFilter.getArrayPtr(), hFilter.getSize(), vFilter.getSize());
}

// ######################################################################
template <class T>
Image<typename promote_trait<T, float>::TP>
sepFilterSV(const Image<T>& src, const float* hFilt, const float* vFilt,
            const int hfs, const int vfs)
{
  Image<typename promote_trait<T, float>::TP> result = src;
  if (hfs > 0) result = xFilterSV(result, hFilt, hfs);
  if (vfs > 0) result = yFilterSV(result, vFilt, vfs);
  return result;
}

// Include the explicit instantiations
#include "inst/ModelNeuron/SVFilter.I"

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
