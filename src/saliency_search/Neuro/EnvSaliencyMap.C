/*!@file Neuro/EnvSaliencyMap.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EnvSaliencyMap.C $
// $Id: EnvSaliencyMap.C 11389 2009-06-30 03:08:01Z lior $
//

#ifndef NEURO_ENVSALIENCYMAP_C_DEFINED
#define NEURO_ENVSALIENCYMAP_C_DEFINED

#include "Neuro/EnvSaliencyMap.H"

#include "Component/GlobalOpts.H" // for OPT_TextLogFile
#include "Component/ModelOptionDef.H"
#include "Image/MathOps.H"
#include "Image/DrawOps.H"
#include "Image/ShapeOps.H"
#include "Neuro/EnvOpts.H"
#include "Util/MathFunctions.H" // for clampValue()
#include "Util/TextLog.H"
#include "Util/log.H"

static const ModelOptionCateg MOC_ESM = {
  MOC_SORTPRI_3, "EnvSaliencyMap-related Options" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmUseFixed =
  { MODOPT_FLAG, "EsmUseFixed", &MOC_ESM, OPTEXP_CORE,
    "Whether to use a fixed center-of-attention given by the user",
    "esm-use-fixed", '\0', "", "false" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmFixedX =
  { MODOPT_ARG(int), "EsmFixedX", &MOC_ESM, OPTEXP_CORE,
    "X coordinate of fixed center-of-atteniton location",
    "esm-fixed-x", '\0', "int", "20" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmFixedY =
  { MODOPT_ARG(int), "EsmFixedY", &MOC_ESM, OPTEXP_CORE,
    "Y coordinate of fixed center-of-atteniton location",
    "esm-fixed-y", '\0', "int", "15" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_GetNMostSalientLoc =
  { MODOPT_ARG(int), "GetNMostSalientLoc", &MOC_ESM, OPTEXP_CORE,
    "Return a vector with the stop n most salient locations",
    "get-nmost-salient-loc", '\0', "int", "1" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_InternalIORRadius =
  { MODOPT_ARG(int), "InternalIORRadius", &MOC_ESM, OPTEXP_CORE,
    "The size of the internal IOR in pixels used for retrieving the N Most salient locations. "
    " This options will not change the actual saliency map.",
    "internal-ior-radius", '\0', "int", "10" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmIorHalfLife =
  { MODOPT_ARG(double), "EsmIorHalfLife", &MOC_ESM, OPTEXP_CORE,
    "Number of frames in which the IOR map decays by half",
    "esm-ior-halflife", '\0', "<double>", "6.5" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmIorStrength =
  { MODOPT_ARG(double), "EsmIorStrength", &MOC_ESM, OPTEXP_CORE,
    "Magnitude of IOR (useful range 0..255)",
    "esm-ior-strength", '\0', "<double>", "16.0" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmIorRadius =
  { MODOPT_ARG(double), "EsmIorRadius", &MOC_ESM, OPTEXP_CORE,
    "Radius of IOR",
    "esm-ior-radius", '\0', "<double>", "32.0" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmInertiaRadius =
  { MODOPT_ARG(double), "EsmInertiaRadius", &MOC_ESM, OPTEXP_CORE,
    "Radius of inertia blob",
    "esm-inertia-radius", '\0', "<double>", "32.0" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmInertiaStrength =
  { MODOPT_ARG(double), "EsmInertiaStrength", &MOC_ESM, OPTEXP_CORE,
    "Initial strength of inertia blob",
    "esm-inertia-strength", '\0', "<double>", "100.0" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmInertiaHalfLife =
  { MODOPT_ARG(double), "EsmInertiaHalfLife", &MOC_ESM, OPTEXP_CORE,
    "Number of frames in which the inertia blob decays by half",
    "esm-inertia-halflife", '\0', "<double>", "6.5" };

// Used by: EnvSaliencyMap
const ModelOptionDef OPT_EsmInertiaShiftThresh =
  { MODOPT_ARG(double), "EsmInertiaShiftThresh", &MOC_ESM, OPTEXP_CORE,
    "Distance threshold for inertia shift",
    "esm-inertia-thresh", '\0', "<double>", "5.0" };

static const ModelOptionDef OPT_DynamicFeedback =
  { MODOPT_ARG(double), "DynamicFeedback", &MOC_ESM, OPTEXP_CORE,
    "How strongly the amount of temporal change in the visual cortex "
    "output causes suppression of IOR and inertia. A large value means "
    "that IOR and inertia will be totally suppressed by a small amount "
    "of temporal change in the visual cortex output, while a value of "
    "0.0 means that IOR and inertia will not be suppressed at all, "
    "regardless of the amount of temporal change in the visual cortex "
    "output.",
    "dynamic-feedback", '\0', "<double>", "1.5" };

// ######################################################################
static double meanAbsDiff(const Image<float>* x,
                          const Image<byte>* y)
{
  ASSERT(x->getDims() == y->getDims());

  double sum = 0.0;

  const size_t sz = x->getSize();

  const Image<float>::const_iterator xptr = x->begin();
  const Image<byte>::const_iterator yptr = y->begin();

  for (size_t i = 0; i < sz; ++i)
    sum += fabs(xptr[i] - yptr[i]);

  return sz > 0 ? sum/sz : 0.0;
}

// ######################################################################
static double sigmoid(double v)
{
  return 1.0 / (1.0 + exp(-v));
}

// ######################################################################
EnvSaliencyMap::EnvSaliencyMap(OptionManager& mgr)
  :
  ModelComponent(mgr, "Embeddable Saliency Map", "EnvSaliencyMap"),
  itsInertiaLoc(-1,-1),
  itsCurrentInertiaFactor(1.0),
  itsInertiaMap(),
  itsInhibMap(),
  itsUseFixed(&OPT_EsmUseFixed, this, ALLOW_ONLINE_CHANGES),
  itsFixedX(&OPT_EsmFixedX, this, ALLOW_ONLINE_CHANGES),
  itsFixedY(&OPT_EsmFixedY, this, ALLOW_ONLINE_CHANGES),
  itsGetNMostSalientLoc(&OPT_GetNMostSalientLoc, this, ALLOW_ONLINE_CHANGES),
  itsInternalIORRadius(&OPT_InternalIORRadius, this, ALLOW_ONLINE_CHANGES),
  itsDynamicFeedback(&OPT_DynamicFeedback, this, ALLOW_ONLINE_CHANGES),
  itsInertiaHalfLife(&OPT_EsmInertiaHalfLife, this, ALLOW_ONLINE_CHANGES),
  itsInertiaStrength(&OPT_EsmInertiaStrength, this, ALLOW_ONLINE_CHANGES),
  itsInertiaRadius(&OPT_EsmInertiaRadius, this, ALLOW_ONLINE_CHANGES),
  itsInertiaShiftThresh(&OPT_EsmInertiaShiftThresh, this, ALLOW_ONLINE_CHANGES),
  itsIorHalfLife(&OPT_EsmIorHalfLife, this, ALLOW_ONLINE_CHANGES),
  itsIorStrength(&OPT_EsmIorStrength, this, ALLOW_ONLINE_CHANGES),
  itsIorRadius(&OPT_EsmIorRadius, this, ALLOW_ONLINE_CHANGES),
  itsLevelSpec(&OPT_EnvLevelSpec, this),
  itsTextLogFile(&OPT_TextLogFile, this),
  itsDynamicFactor(1.0),
  itsVcxMovingAvg(),
  itsVcxFlicker(0.0),
  itsVcxMeanDiffCenter(4.0),
  itsVcxFlickerMin(sigmoid(0.5*(-itsVcxMeanDiffCenter)))
{}

// ######################################################################
EnvSaliencyMap::~EnvSaliencyMap()
{}

// ######################################################################
void EnvSaliencyMap::paramChanged(ModelParamBase* const param,
                                  const bool valueChanged,
                                  ParamClient::ChangeStatus* status)
{
  if (param == &itsUseFixed)
    {
      itsFixedX.setInactive(!itsUseFixed.getVal());
      itsFixedY.setInactive(!itsUseFixed.getVal());
    }
}

// ######################################################################
EnvSaliencyMap::State
EnvSaliencyMap::getSalmap(const Image<byte>& vcxmap,
                          const Point2D<int>& forceWinnerFullres)
{
  State result;

  result.salmap = vcxmap;

//  if (itsInertiaMap.initialized() && itsInhibMap.initialized())
//    result.salmap += (itsInertiaMap - itsInhibMap);
//  else if (itsInertiaMap.initialized())
//    result.salmap += itsInertiaMap;
//  else if (itsInhibMap.initialized())
//    result.salmap -= itsInertiaMap;

  const int zoom = (1 << itsLevelSpec.getVal().mapLevel());

  //Apply the bias if we have one
  if (itsBiasImg.initialized())
  {
    if (itsBiasImg.getDims() != result.salmap.getDims())
      itsBiasImg = rescale(itsBiasImg, result.salmap.getDims());
    result.salmap *= itsBiasImg;
  }

  if (itsUseFixed.getVal())
    {
      const Point2D<int> forceWinner
        (clampValue(itsFixedX.getVal(), 0, result.salmap.getWidth()-1),
         clampValue(itsFixedY.getVal(), 0, result.salmap.getHeight()-1));

      ASSERT(result.salmap.coordsOk(forceWinner));
      result.lowres_maxpos = forceWinner;
      result.maxval = result.salmap.getVal(result.lowres_maxpos);
    }
  else if (forceWinnerFullres.isValid())
    {
      const Point2D<int> forceWinner
        (clampValue(forceWinnerFullres.i / zoom, 0, result.salmap.getWidth()-1),
         clampValue(forceWinnerFullres.j / zoom, 0, result.salmap.getHeight()-1));

      ASSERT(result.salmap.coordsOk(forceWinner));
      result.lowres_maxpos = forceWinner;
      result.maxval = result.salmap.getVal(result.lowres_maxpos);
    }
  else
    {
      findMax(result.salmap, result.lowres_maxpos, result.maxval);
    }


  //Get the N most salient location from a temporary saliency map
  //The first location is retrieved from the result structure itself
  Image<byte> tmpSmap = vcxmap;


  Point2D<int> currentMaxLoc = result.lowres_maxpos;
  LocInfo locInfo;
  locInfo.lowres_maxpos = result.lowres_maxpos;
  locInfo.fullres_maxpos = locInfo.lowres_maxpos * zoom + zoom/2;
  locInfo.maxval = result.maxval;
  result.nMostSalientLoc.push_back(locInfo);

  for(int i=1; i<itsGetNMostSalientLoc.getVal(); i++)
  {
    //Apply the IOR to the tmp smap
    drawDisk(tmpSmap, currentMaxLoc, itsInternalIORRadius.getVal(), (byte)0);

    //Get the next most salient location
    findMax(tmpSmap, locInfo.lowres_maxpos, locInfo.maxval);
    locInfo.fullres_maxpos = locInfo.lowres_maxpos * zoom + zoom/2;
    result.nMostSalientLoc.push_back(locInfo);
    currentMaxLoc = locInfo.lowres_maxpos;
  }

  result.fullres_maxpos = result.lowres_maxpos * zoom + zoom/2;

  if (!itsVcxMovingAvg.initialized())
    itsVcxMovingAvg = vcxmap;

  const double vcxmeandiff = meanAbsDiff(&itsVcxMovingAvg, &vcxmap);
  itsVcxFlicker =
    (sigmoid(0.5*(vcxmeandiff-itsVcxMeanDiffCenter)) - itsVcxFlickerMin)
    /
    (1.0 - itsVcxFlickerMin);

  itsDynamicFactor =
    pow(1.0 - itsVcxFlicker,
        std::max(0.0, itsDynamicFeedback.getVal()));

  // Update inertia map. The inertia center always follows the current
  // saliency map peak. If the new peak is within a threshold distance
  // from the previous peak, then the inertia strength decays by a
  // factor according to its half-life -- this behavior allow an
  // inertia blob to track a moving object while decaying at the same
  // time. On the other hand if the new peak is distant from the
  // previous peak, then the inertia strength is reset to the max
  // value so that we essentially begin a new inertia sequence at the
  // new peak location.

  if (itsInertiaLoc.i < 0 || itsInertiaLoc.j < 0
      || (result.lowres_maxpos.squdist(itsInertiaLoc)
          > (itsInertiaShiftThresh.getVal()
             * itsInertiaShiftThresh.getVal())))
    {
      itsCurrentInertiaFactor = 1.0;
      LDEBUG("inertia shift to (%d,%d)",
             result.lowres_maxpos.i, result.lowres_maxpos.j);
    }
  else
    {
      const float factor =
        (itsDynamicFactor * itsInertiaHalfLife.getVal()) > 0
        ? pow(0.5, 1.0/(itsDynamicFactor * itsInertiaHalfLife.getVal()))
        : 0.0f;
      itsCurrentInertiaFactor *= factor;
    }

  {
    if (itsInertiaMap.getDims() != result.salmap.getDims())
      itsInertiaMap = Image<byte>(result.salmap.getDims(), ZEROS);

    itsInertiaLoc = result.lowres_maxpos;

    Image<float>::iterator iptr = itsInertiaMap.beginw();

    const double s =
      itsInertiaStrength.getVal()
      * itsDynamicFactor
      * itsCurrentInertiaFactor;

    const double r_inv =
      itsInertiaRadius.getVal() > 0.0
      ? (1.0 / itsInertiaRadius.getVal())
      : 0.0;

    Point2D<int> p;
    for (p.j = 0; p.j < itsInertiaMap.getHeight(); ++p.j)
      for (p.i = 0; p.i < itsInertiaMap.getWidth(); ++p.i)
        {
          const int dsq =
            (itsInertiaLoc.i - p.i) * (itsInertiaLoc.i - p.i)
            + (itsInertiaLoc.j - p.j) * (itsInertiaLoc.j - p.j);

          *iptr++ = s * exp(-dsq * r_inv);
        }
  }

  if (itsDynamicFactor * itsIorStrength.getVal() > 0.0)
    {
      if (itsInhibMap.getDims() != result.salmap.getDims())
        itsInhibMap = Image<byte>(result.salmap.getDims(), ZEROS);

      const float factor =
        (itsDynamicFactor * itsIorHalfLife.getVal()) > 0
        ? pow(0.5, 1.0/(itsDynamicFactor * itsIorHalfLife.getVal()))
        : 0.0f;

      Image<byte>::iterator iptr = itsInhibMap.beginw();

      Point2D<int> p;
      for (p.j = 0; p.j < itsInhibMap.getHeight(); ++p.j)
        for (p.i = 0; p.i < itsInhibMap.getWidth(); ++p.i)
          {
            const int dsq =
              (result.lowres_maxpos.i - p.i) * (result.lowres_maxpos.i - p.i)
              + (result.lowres_maxpos.j - p.j) * (result.lowres_maxpos.j - p.j);

            const int newval =
              int(*iptr * factor
                  + (itsDynamicFactor * itsIorStrength.getVal()
                     * exp(- dsq / itsIorRadius.getVal())));

            *iptr++ =
              newval < 0
              ? 0
              : newval > 255
              ? 255
              : byte(newval);
          }
    }
  else
    itsInhibMap.clear(0);

  itsVcxMovingAvg += vcxmap;
  itsVcxMovingAvg *= 0.5;

  textLog(itsTextLogFile.getVal(), "FOAcenter",
          convertToString(result.fullres_maxpos));

  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_ENVSALIENCYMAP_C_DEFINED
