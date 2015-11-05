/*!@file Neuro/EnvSegmenterColorRegion.C FOA segmenter */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvSegmenterColorRegion.C $
// $Id: EnvSegmenterColorRegion.C 9740 2008-05-10 03:20:14Z rjpeters $
//

#ifndef NEURO_ENVSEGMENTERCOLORREGION_C_DEFINED
#define NEURO_ENVSEGMENTERCOLORREGION_C_DEFINED

#include "Neuro/EnvSegmenterColorRegion.H"

#include "Component/ModelOptionDef.H"
#include "Image/ColorOps.H"
#include "Image/Image.H"
#include "Image/ImageSet.H"
#include "Image/LowPass.H"
#include "Image/MathOps.H"
#include "Image/Normalize.H"
#include "Image/Pixels.H"
#include "Image/PyramidOps.H"
#include "Neuro/NeuroOpts.H"
#include "Util/MathFunctions.H"

static const ModelOptionDef OPT_EseDynamicFoa =
  { MODOPT_FLAG, "EseDynamicFoa", &MOC_ITC, OPTEXP_CORE,
    "Whether to use a dynamically-sized FOA obtained from color-based "
    "segmentation",
    "ese-dynamic-foa", '\0', "", "true" };

static const ModelOptionDef OPT_EseFoaSize =
  { MODOPT_ARG(int), "EseFoaSize", &MOC_ITC, OPTEXP_CORE,
    "Size, in pixels, of the FOA box for non-dynamically-sized FOAs",
    "ese-foa-size", '\0', "int", "180" };

static const ModelOptionDef OPT_EseDynamicFoaMinSize =
  { MODOPT_ARG(int), "EseFoaMinSize", &MOC_ITC, OPTEXP_CORE,
    "Minimum size, in pixels, of the FOA box for dynamically-sized FOAs",
    "ese-foa-min-size", '\0', "int", "32" };

static const ModelOptionDef OPT_EseDynamicFoaMaxSize =
  { MODOPT_ARG(int), "EseFoaMaxSize", &MOC_ITC, OPTEXP_CORE,
    "Maximum size, in pixels, of the FOA box for dynamically-sized FOAs",
    "ese-foa-max-size", '\0', "int", "360" };

static const ModelOptionDef OPT_EseSegmentationThresh =
  { MODOPT_ARG(double), "EseSegmentationThresh", &MOC_ITC, OPTEXP_CORE,
    "Pixel-similarity threshold for object segmentation (0.0 .. 1.0)",
    "ese-segmentation-thresh", '\0', "double", "0.5" };

static const ModelOptionDef OPT_EseSegmentationScale =
  { MODOPT_ARG(size_t), "EseSegmentationScale", &MOC_ITC, OPTEXP_CORE,
    "Pyramid scale at which to compute object segmentation",
    "ese-segmentation-scale", '\0', "uint", "4" };

namespace
{
  Rectangle segmentColor(const Image<PixRGB<byte> >& rgbin,
                         const Point2D<int>& foacenter,
                         const size_t segmentationScale,
                         const float segmentationThresh,
                         const int foaminsize,
                         const int foamaxsize,
                         Image<float>& distmap,
                         Image<byte>& mask)
  {
    /* color-based target segmentation:

       (1) scale rgb input down by (1 << segmentationScale), e.g. 16
       (2) convert to h2sv1 color space
       (3) spatially smooth the h1, h2, s, and v maps
       (4) find the color values at the foa center
       (5) compute a distance map reflecting both the distance in
           color space from the foa center as well as the distance in
           retinotopic space -- we want to find pixels that have a
           similar color to the target and that are also spatially
           close to the target
       (6) start a flood from the foa center in the distance map
           looking for connected pixels whose similarity (inverse
           distance) to the target is above some threshold
       (7) the foa is smallest rectangular bounding box that contains
           the flooded pixels, and optionally constrain the foa to
           given min/max dimensions
    */

    const ImageSet<PixRGB<byte> > pyr =
      buildPyrLocalAvg2x2(rgbin, segmentationScale+1);

    Image<PixH2SV1<float> > h2sv1(pyr[segmentationScale]);
    Image<float> h1 = lowPass9(getPixelComponentImage(h2sv1, 0));
    Image<float> h2 = lowPass9(getPixelComponentImage(h2sv1, 1));
    Image<float> s = lowPass9(getPixelComponentImage(h2sv1, 2));
    Image<float> v = lowPass9(getPixelComponentImage(h2sv1, 3));

    const PixH2SV1<float> target(h1.getVal(foacenter),
                                 h2.getVal(foacenter),
                                 s.getVal(foacenter),
                                 v.getVal(foacenter));
    distmap = Image<float>(h2sv1.getDims(), NO_INIT);

    Image<PixH2SV1<float> >::const_iterator ptr = h2sv1.begin();
    Image<float>::iterator bptr = distmap.beginw();

    Point2D<int> p;
    for (p.j = 0; p.j < distmap.getHeight(); ++p.j)
      for (p.i = 0; p.i < distmap.getWidth(); ++p.i)
        {
          const float dist = p.distance(foacenter) / 15.0;
          *bptr =
            0.2 * exp(-dist*dist)
            +
            0.8 * exp(-sqrt(0.3 * squareOf(ptr->p[0] - target.p[0])
                            +
                            0.3 * squareOf(ptr->p[1] - target.p[1])
                            +
                            0.3 * squareOf(ptr->p[2] - target.p[2])
                            +
                            0.1 * squareOf(ptr->p[3] - target.p[3])
                            ));

          ++bptr;
          ++ptr;
        }

    distmap = lowPass9(distmap);

    float mi, ma, avg; getMinMaxAvg(distmap, mi, ma, avg);

    const float thresh = avg + segmentationThresh * (ma - avg);

    mask = Image<byte>(distmap.getDims(), ZEROS);

    std::vector<Point2D<int> > pts;
    pts.push_back(foacenter);
    mask[foacenter] = 255;

#if 0
    const Point2D<int> offsets[8] =
      {
        Point2D<int>(-1, -1), Point2D<int>(0, -1), Point2D<int>(1, -1),
        Point2D<int>(-1,  0),                 Point2D<int>(1,  0),
        Point2D<int>(-1,  1), Point2D<int>(0,  1), Point2D<int>(1,  1)
      };
#endif

    const Point2D<int> offsets[4] =
      {
        Point2D<int>(0, -1),
        Point2D<int>(-1,  0), Point2D<int>(1,  0),
        Point2D<int>(0,  1),
      };

    int left = foacenter.i, top = foacenter.j, right = foacenter.i, bottom = foacenter.j;

    while (pts.size() > 0)
      {
        const Point2D<int> p = pts.back();
        pts.pop_back();

        if      (p.i < left) left = p.i;
        else if (p.i > right) right = p.i;
        if      (p.j < top) top = p.j;
        else if (p.j > bottom) bottom = p.j;

        for (size_t i = 0; i < sizeof(offsets) / sizeof(offsets[0]); ++i)
          if (distmap.coordsOk(p+offsets[i]) &&
              distmap[p+offsets[i]] >= thresh &&
              mask[p+offsets[i]] == 0)
            {
              pts.push_back(p+offsets[i]);
              mask[p+offsets[i]] = 255;
            }
      }

    ASSERT(left >= 0);
    ASSERT(right < mask.getWidth());
    ASSERT(top >= 0);
    ASSERT(bottom < mask.getHeight());

    const Rectangle rawfoa =
       Rectangle::tlbrO(top << segmentationScale,
                        left << segmentationScale,
                        (bottom + 1) << segmentationScale,
                        (right + 1) << segmentationScale);

    return constrainRect(rawfoa, rgbin.getBounds(),
                         foaminsize, foamaxsize);
  }
}

// ######################################################################
EnvSegmenterColorRegion::EnvSegmenterColorRegion(OptionManager& mgr)
  :
  EnvSegmenter(mgr, "Embeddable Color-Region FOA Segmenter", "EnvSegmenterColorRegion"),
  itsDynamicFoa(&OPT_EseDynamicFoa, this, ALLOW_ONLINE_CHANGES),
  itsFoaSize(&OPT_EseFoaSize, this, ALLOW_ONLINE_CHANGES),
  itsFoaMinSize(&OPT_EseDynamicFoaMinSize, this, ALLOW_ONLINE_CHANGES),
  itsFoaMaxSize(&OPT_EseDynamicFoaMaxSize, this, ALLOW_ONLINE_CHANGES),
  itsSegmentationThresh(&OPT_EseSegmentationThresh, this, ALLOW_ONLINE_CHANGES),
  itsSegmentationScale(&OPT_EseSegmentationScale, this, ALLOW_ONLINE_CHANGES)
{}

// ######################################################################
EnvSegmenterColorRegion::~EnvSegmenterColorRegion()
{}

// ######################################################################
void EnvSegmenterColorRegion::paramChanged(ModelParamBase* const param,
                                     const bool valueChanged,
                                     ParamClient::ChangeStatus* status)
{
  EnvSegmenter::paramChanged(param, valueChanged, status);

  if (param == &itsDynamicFoa)
    {
      const bool dyn = itsDynamicFoa.getVal();

      itsFoaSize.setInactive(dyn);
      itsFoaMinSize.setInactive(!dyn);
      itsFoaMaxSize.setInactive(!dyn);
      itsSegmentationThresh.setInactive(!dyn);
    }
  else if (param == &itsFoaSize)
    {
      if (itsFoaSize.getVal() < 16)
        *status = ParamClient::CHANGE_REJECTED;
    }
  else if (param == &itsFoaMinSize)
    {
      if (itsFoaMinSize.getVal() < 16)
        *status = ParamClient::CHANGE_REJECTED;
    }
  else if (param == &itsFoaMaxSize)
    {
      if (itsFoaMaxSize.getVal() < itsFoaMinSize.getVal())
        *status = ParamClient::CHANGE_REJECTED;
    }
  else if (param == &itsSegmentationScale)
    {
      if (itsSegmentationScale.getVal() > 8)
        *status = ParamClient::CHANGE_REJECTED;
    }
}

// ######################################################################
Rectangle EnvSegmenterColorRegion::getFoa(const Image<PixRGB<byte> >& rgbin,
                               const Point2D<int>& center,
                               Image<byte>* foamask,
                               Image<PixRGB<byte> >* segmentdisp) const
{
  *foamask = Image<byte>();
  *segmentdisp = Image<PixRGB<byte> >();

  if (itsDynamicFoa.getVal())
    {
      const int seg_zoom = (1 << itsSegmentationScale.getVal());

      const Point2D<int> seg_scaled_maxpos = center / seg_zoom;

      Image<float> distmap;
      const Rectangle foa =
        segmentColor(rgbin, seg_scaled_maxpos,
                     itsSegmentationScale.getVal(),
                     itsSegmentationThresh.getVal(),
                     itsFoaMinSize.getVal(),
                     itsFoaMaxSize.getVal(),
                     distmap, *foamask);

      *segmentdisp = normalizeFloat(distmap, FLOAT_NORM_0_255);

      return foa;
    }
  else
    {
      Point2D<int> foatopleft = center - itsFoaSize.getVal()/2;
      foatopleft.clampToDims(rgbin.getDims());

      const Rectangle foa =
        Rectangle(foatopleft, Dims(itsFoaSize.getVal(), itsFoaSize.getVal()))
        .getOverlap(rgbin.getBounds());

      return foa;
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_ENVSEGMENTERCOLORREGION_C_DEFINED
