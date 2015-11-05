/*!@file Image/Normalize.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Normalize.C $
// $Id: Normalize.C 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef IMAGE_NORMALIZE_C_DEFINED
#define IMAGE_NORMALIZE_C_DEFINED

#include "Image/Normalize.H"

#include "Image/ColorOps.H"
#include "Image/CutPaste.H"
#include "Image/DrawOps.H"
#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/Pixels.H"
#include "Image/Range.H"
#include "Util/sformat.H"

// ######################################################################
Image<float> normalizeFloat(const Image<float>& src, const int flags)
{
  if (flags == 0)
    return src;

  const Range<float> srcrng = rangeOf(src);
  Range<float> dstrng = srcrng;

  Image<float> nsrc = src;
  if (flags & FLOAT_NORM_0_255)
    {
      inplaceNormalize(nsrc, 0.0f, 255.0f);
      dstrng = Range<float>(0.0f, 255.0f);
    }

  Image<float> result = nsrc;

  if (flags & FLOAT_NORM_WITH_SCALE)
    {
      const int width = std::max(src.getWidth(), 144);

      result = Image<float>(width, src.getHeight() + 50, NO_INIT);
      result.clear(dstrng.min());
      inplacePaste(result, nsrc, Point2D<int>(0,0));

      writeText(result, Point2D<int>(0, src.getHeight()),
                sformat("min: %g", srcrng.min()).c_str(),
                // use the max,min values of the destination image
                // range as the foreground and background colors for
                // text rendering:
                dstrng.max(), dstrng.min());

      writeText(result, Point2D<int>(0, src.getHeight()+25),
                sformat("max: %g", srcrng.max()).c_str(),
                dstrng.max(), dstrng.min());
    }

  return result;
}

// ######################################################################
Image<PixRGB<float> >
normalizeFloatRgb(const Image<PixRGB<float> >& src,
                  const int flags)
{
  if (flags == 0)
    return src;

  float srcmin, srcmax;
  getMinMaxC(src, srcmin, srcmax);
  float dstmin = srcmin, dstmax = srcmax;

  Image<PixRGB<float> > nsrc = src;
  if (flags & FLOAT_NORM_0_255)
    {
      const float scale = 255.0f / srcmax;
      for (Image<PixRGB<float> >::iterator
             nptr = nsrc.beginw(), stop = nsrc.endw();
           nptr != stop; ++nptr)
        {
          *nptr = (*nptr - srcmin) * scale;
        }
      dstmin = 0.0f; dstmax = 255.0f;
    }

  Image<PixRGB<float> > result = nsrc;

  if (flags & FLOAT_NORM_WITH_SCALE)
    {
      const int width = std::max(src.getWidth(), 144);

      result = Image<PixRGB<float> >(width, src.getHeight() + 50, NO_INIT);
      result.clear(PixRGB<float>(dstmin));
      inplacePaste(result, nsrc, Point2D<int>(0,0));

      writeText(result, Point2D<int>(0, src.getHeight()),
                sformat("min: %g", srcmin).c_str(),
                // use the max,min values of the destination image
                // range as the foreground and background colors for
                // text rendering:
                PixRGB<float>(dstmax),
                PixRGB<float>(dstmin));

      writeText(result, Point2D<int>(0, src.getHeight()+25),
                sformat("max: %g", srcmax).c_str(),
                PixRGB<float>(dstmax),
                PixRGB<float>(dstmin));
    }

  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // IMAGE_NORMALIZE_C_DEFINED
