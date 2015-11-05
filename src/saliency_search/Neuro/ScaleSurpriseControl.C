/*!@file Neuro/ScaleSurpriseControl.C attempt to remove surprise from image */

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
// Primary maintainer for this file: T. Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/ScaleSurpriseControl.C $
// $Id: ScaleSurpriseControl.C 10845 2009-02-13 08:49:12Z itti $
//

#ifndef SCALE_SURPRISE_CONTROL_C_DEFINED
#define SCALE_SURPRISE_CONTROL_C_DEFINED

#include "Neuro/ScaleSurpriseControl.H"
#include "Image/ColorOps.H"

/*************************************************************************/

template <class FLOAT>
ScaleSurpriseControl<FLOAT>::ScaleSurpriseControl(const ushort sizeX,
                                                  const ushort sizeY,
                                                  const string confFile)
{
  itsLevelSpecSet = false;
  SSCreadConfig(confFile);
  SSCinit(sizeX,sizeY);
}

/*************************************************************************/

template <class FLOAT>
ScaleSurpriseControl<FLOAT>::ScaleSurpriseControl()
{
  itsLevelSpecSet = false;
}

/*************************************************************************/

template <class FLOAT>
ScaleSurpriseControl<FLOAT>::~ScaleSurpriseControl()
{}

/*************************************************************************/
template <class FLOAT> inline
void ScaleSurpriseControl<FLOAT>::SSCsetLevelSpecInfo(const uint levMin,
                                                      const uint levMax,
                                                      const uint delMin,
                                                      const uint delMax,
                                                      const uint mapLevel,
                                                      const uint maxIndex,
                                                      const uint maxDepth)
{
  itsLevMin   = static_cast<ushort>(levMin);
  itsLevMax   = static_cast<ushort>(levMax);
  itsDelMin   = static_cast<ushort>(delMin);
  itsDelMax   = static_cast<ushort>(delMax);
  itsMapLevel = static_cast<ushort>(mapLevel);
  itsMaxIndex = static_cast<ushort>(maxIndex);
  itsMaxDepth = static_cast<ushort>(maxDepth);

  itsLevelSpecSet = true;
}

/*************************************************************************/
template <class FLOAT>
void ScaleSurpriseControl<FLOAT>::SSCreadConfig(const string confFile)
{
  LINFO("Reading config file %s",confFile.c_str());
  itsReadConfig.openFile(confFile.c_str());

  // get basic information and biases
  itsAxisBiasX      = itsReadConfig.getItemValueF("itsAxisBiasX");
  itsAxisBiasY      = itsReadConfig.getItemValueF("itsAxisBiasY");
  itsAxisBiasZ      = itsReadConfig.getItemValueF("itsAxisBiasZ");

  itsConspicMapBias[SC_DR0]  = itsReadConfig.getItemValueF("itsDRConBias0");
  itsConspicMapBias[SC_DR1]  = itsReadConfig.getItemValueF("itsDRConBias1");
  itsConspicMapBias[SC_DR2]  = itsReadConfig.getItemValueF("itsDRConBias2");
  itsConspicMapBias[SC_DR3]  = itsReadConfig.getItemValueF("itsDRConBias3");

  itsConspicMapBias[SC_GA0]  = itsReadConfig.getItemValueF("itsGAConBias0");
  itsConspicMapBias[SC_GA1]  = itsReadConfig.getItemValueF("itsGAConBias1");
  itsConspicMapBias[SC_GA2]  = itsReadConfig.getItemValueF("itsGAConBias2");
  itsConspicMapBias[SC_GA3]  = itsReadConfig.getItemValueF("itsGAConBias3");

  itsConspicMapBias[SC_IN]   = itsReadConfig.getItemValueF("itsINConBias");
  itsConspicMapBias[SC_FL]   = itsReadConfig.getItemValueF("itsFLConBias");
  itsConspicMapBias[SC_RG]   = itsReadConfig.getItemValueF("itsRGConBias");
  itsConspicMapBias[SC_BY]   = itsReadConfig.getItemValueF("itsBYConBias");

  itsConspicMapBias[SC_H1]   = itsReadConfig.getItemValueF("itsH1ConBias");
  itsConspicMapBias[SC_H2]   = itsReadConfig.getItemValueF("itsH2ConBias");
  itsConspicMapBias[SC_HS]   = itsReadConfig.getItemValueF("itsHSConBias");
  itsConspicMapBias[SC_HV]   = itsReadConfig.getItemValueF("itsHVConBias");

  itsLambda         = itsReadConfig.getItemValueF("itsLambda");
  itsStdSize        = itsReadConfig.getItemValueF("itsStdSize");
  itsZSigma         = itsReadConfig.getItemValueF("itsZSigma");
  itsUseMaxLevel    = itsReadConfig.getItemValueB("itsUseMaxLevel");
  itsH1Bias         = itsReadConfig.getItemValueF("itsH1Bias");
  itsH2Bias         = itsReadConfig.getItemValueF("itsH2Bias");
  itsSBias          = itsReadConfig.getItemValueF("itsSBias");
  itsVBias          = itsReadConfig.getItemValueF("itsVBias");

  itsOriginalImageWeight =
    itsReadConfig.getItemValueF("itsOriginalImageWeight");

  itsMasterConspicBias =
    itsReadConfig.getItemValueF("itsMasterConspicBias");

  itsSharpFactorH1 =
    itsReadConfig.getItemValueF("itsSharpFactorH1");
  itsSharpFactorH2 =
    itsReadConfig.getItemValueF("itsSharpFactorH2");
  itsSharpFactorS =
    itsReadConfig.getItemValueF("itsSharpFactorS");
  itsSharpFactorV =
    itsReadConfig.getItemValueF("itsSharpFactorV");

  itsGetReduced =
    itsReadConfig.getItemValueB("itsGetReduced");

  itsUseAndersonSeperable =
    itsReadConfig.getItemValueB("itsUseAndersonSeperable");

  itsUseTemporal =
    itsReadConfig.getItemValueB("itsUseTemporal");

  itsNormalizeBiasWithScale =
    itsReadConfig.getItemValueB("itsNormalizeBiasWithScale");

  itsBaseFilterSize = (ushort)itsReadConfig.getItemValueF("itsBaseFilterSize");


  // get LevelSpec information from file if not already supplied
  if(!itsLevelSpecSet)
  {
    LINFO("NOTICE: Setting LevelSpec info from config file");
    itsLevMin    = (ushort)itsReadConfig.getItemValueF("itsLevMin");
    itsLevMax    = (ushort)itsReadConfig.getItemValueF("itsLevMax");
    itsDelMin    = (ushort)itsReadConfig.getItemValueF("itsDelMin");
    itsDelMax    = (ushort)itsReadConfig.getItemValueF("itsDelMax");
    itsMapLevel  = (ushort)itsReadConfig.getItemValueF("itsMapLevel");
    itsMaxIndex  = (ushort)itsReadConfig.getItemValueF("itsMaxIndex");
    itsMaxDepth  = (ushort)itsReadConfig.getItemValueF("itsMaxDepth");

    itsLevelSpecSet = true;
  }
  itsScaleBias.resize(itsMaxIndex,0.0F);

  std::string baseScale = "itsScaleBias_";

  // read in each scale bias
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    char temp[100];
    sprintf(temp,"%s%d",baseScale.c_str(),i);
    itsScaleBias[i] = itsReadConfig.getItemValueF(static_cast<string>(temp));
  }

  itsScalePower.resize(itsMaxIndex,0.0F);
  baseScale = "itsScalePower_";

  // read in each scale power
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    char temp[100];
    sprintf(temp,"%s%d",baseScale.c_str(),i);
    itsScalePower[i] = itsReadConfig.getItemValueF(static_cast<string>(temp));
  }
}

/*************************************************************************/

template <class FLOAT>
void ScaleSurpriseControl<FLOAT>::SSCinit(const ushort sizeX,
                                          const ushort sizeY)
{
  LINFO("INIT");

  if(!itsLevelSpecSet)
    LFATAL("LevelSpec not yet set. It must be set before calling init");

  itsFrameCounter = 0;

  itsImageSizeX = sizeX;
  itsImageSizeY = sizeY;

  // create a SurpriseControl for each scale
  LINFO("Creating temp object");
  SurpriseControl<PIX_H2SV_TYPE<FLOAT>,PixHyper<FLOAT,SC_MAX_CHANNELS>,FLOAT>
    tempSC(sizeX,sizeY);
  itsSurpriseControl.resize(itsMaxIndex,tempSC);
  itsFinalImage.resize(sizeX,sizeY);

  itsImageSizesX.resize(itsMaxIndex,0);
  itsImageSizesY.resize(itsMaxIndex,0);
  itsFilterSizesX.resize(itsMaxIndex,0);
  itsFilterSizesY.resize(itsMaxIndex,0);
  itsFilterSizesZ.resize(itsMaxIndex,0);
  itsResultImages.resize(itsMaxIndex,itsFinalImage);

  itsImageBaseX = itsImageSizeX/(ushort)(pow(2.0F,(FLOAT)itsLevMin));
  itsImageBaseY = itsImageSizeY/(ushort)(pow(2.0F,(FLOAT)itsLevMin));

  LINFO("Base Image Size %d x %d",itsImageBaseX,itsImageBaseY);

  uint indx = 0;

  // determine the pyramid sizes used and store
  // also determin how large the feature filter would be on a non-reduced image
  LINFO("Setting up filters");
  for(ushort i = 0; i < (itsDelMax - itsDelMin + 1); i++)
  {
    ushort bx = itsImageBaseX;
    ushort by = itsImageBaseY;
    FLOAT fz  = itsZSigma;
    for(ushort j = 0; j < (itsLevMax - itsLevMin + 1); j++)
    {
      itsImageSizesX[indx]  = bx;
      itsFilterSizesX[indx] = (ushort)round(((FLOAT)itsBaseFilterSize/
                               (FLOAT)itsImageSizesX[indx]) *
                                (FLOAT)itsImageSizeX);
      LINFO("%d : Filter Size X %d",indx,itsFilterSizesX[indx]);
      itsImageSizesY[indx]  = by;
      itsFilterSizesY[indx] = (ushort)round(((FLOAT)itsBaseFilterSize/
                               (FLOAT)itsImageSizesY[indx]) *
                               (FLOAT)itsImageSizeY);
      LINFO("%d : Filter Size Y %d",indx,itsFilterSizesY[indx]);
      itsFilterSizesZ[indx] = fz;
      LINFO("%d : Filter Size Z %f",indx,itsFilterSizesZ[indx]);
      bx = bx/2; by = by/2; // fz = fz*2;
      indx++;
    }
  }

  // set up each SurpriseControl
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    LINFO("Setting up RemoveSaliency for scale %d",i);
    itsSurpriseControl[i].SCsetAxisBias(itsAxisBiasX,
                                        itsAxisBiasY,
                                        itsAxisBiasZ);
    // we try to fit something close to the orignal filters used
    const FLOAT sigma   = itsFilterSizesX[i]/5;
    const FLOAT z_sigma = itsFilterSizesZ[i];
    LINFO("%d Sigma %f Z Sigma %f StdDev %f",i,sigma,z_sigma,itsStdSize);
    if(itsUseAndersonSeperable)
      itsSurpriseControl[i].SCcreateAndersonSepFilters(itsFilterSizesX[i]);
    else
      itsSurpriseControl[i].SCcreateSepFilters(sigma,z_sigma,itsStdSize);
    itsSurpriseControl[i].SCfindConvolutionEndPoints();
    LINFO("%d Scale Power %f",i,itsScalePower[i]);
    LINFO("%d Bias H1 %f H2 %f S %f V %f",i,
          itsH1Bias,itsH2Bias,itsSBias,itsVBias);
    itsSurpriseControl[i].SCsetH2SVBias(itsH1Bias,itsH2Bias,itsSBias,itsVBias);
    itsSurpriseControl[i].SCuseMaxLevel(itsUseMaxLevel);
    itsSurpriseControl[i].SCuseTemporal(itsUseTemporal);
    itsSurpriseControl[i].SCnormalizeBiasWithScale(itsNormalizeBiasWithScale);
    LINFO("%d Lambda %f",i,itsLambda);
    itsSurpriseControl[i].SCsetLambda(itsLambda);
    LINFO("%d Original Image Weight %f",i,itsOriginalImageWeight);
    itsSurpriseControl[i].SCsetOriginalImageWeight(itsOriginalImageWeight);
    LINFO("%d Master Conspic Map Bias %f",i,itsMasterConspicBias);
    itsSurpriseControl[i].SCsetMasterConspicBias(itsMasterConspicBias);
    itsSurpriseControl[i].SCsetMyScale(i);

  }
}

/*************************************************************************/

template <class FLOAT> inline
void ScaleSurpriseControl<FLOAT>::SSCinputRawImage(
                                     const Image<PixRGB<FLOAT> >& rawImage)
{
  itsRawImage = rawImage;
}

/*************************************************************************/

template <class FLOAT> inline
void ScaleSurpriseControl<FLOAT>::SSCinputSalMap(const Image<FLOAT>& salMap)
{
  itsSalMap = salMap;
}

/*************************************************************************/

template <class FLOAT> inline
void ScaleSurpriseControl<FLOAT>::SSCinputBayesWeightImage(
                                  const Image<FLOAT>& bayesImage)
{
  itsBayesWeightImage = bayesImage;
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    itsSurpriseControl[i].SCinputBayesWeightImage(itsBayesWeightImage);
  }
}

/*************************************************************************/

template <class FLOAT> inline
void ScaleSurpriseControl<FLOAT>::SSCinputMaskImage(
                                  const Image<FLOAT>& maskImage)
{
  itsMaskImage = maskImage;
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    itsSurpriseControl[i].SCinputMaskImage(itsMaskImage);
  }
}

/*************************************************************************/

template <class FLOAT>
void ScaleSurpriseControl<FLOAT>::SSCprocessFrame(Brain* brain)
{
  ////////itsSalMap   = brain->getSM()->getV(true);
  LFATAL("FIXME PLEASE!");
  /*
  itsSalMap   = rescale(itsSalMap,itsImageSizeX,itsImageSizeY);
  // get each brain conspicuity map if we have one
  nub::soft_ref<VisualCortex> vc;///////////FIXME = brain->getVC();

  nub::soft_ref<ColorChannel>       cc;
  nub::soft_ref<IntensityChannel>   ic;
  nub::soft_ref<FlickerChannel>     fc;
  nub::soft_ref<MotionChannel>      mc;
  nub::soft_ref<OrientationChannel> oc;
  nub::soft_ref<H2SVChannel>        hc;

  // process over each scale type from the image pyramid

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    itsSurpriseControl[i].SCinputRawImage(itsRawImage);
    itsSurpriseControl[i].SCinputSalMap(itsSalMap);
  }

  // set up biases durring the first frame, maybe change this later
  if(itsFrameCounter == 0)
  {
    if(vc->hasSubChan("H2SVcolor"))
    {
      LINFO("INPUT H2SV Bias");
      for(ushort i = 0; i < itsMaxIndex; i++)
      {
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_H1]
                                               *itsScalePower[i],SC_H1);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_H2]
                                               *itsScalePower[i],SC_H2);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_HS]
                                               *itsScalePower[i],SC_HS);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_HV]
                                               *itsScalePower[i],SC_HV);
      }
    }
    else if(vc->hasSubChan("intensity"))
    {
      LINFO("INPUT Intensity Bias");
      for(ushort i = 0; i < itsMaxIndex; i++)
      {
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_IN]
                                               *itsScalePower[i],SC_IN);
      }
    }
    if(vc->hasSubChan("color"))
    {
      LINFO("INPUT Color Bias");
      for(ushort i = 0; i < itsMaxIndex; i++)
      {
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_RG]
                                               *itsScalePower[i],SC_RG);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_BY]
                                               *itsScalePower[i],SC_BY);
      }
    }
    if(vc->hasSubChan("flicker"))
    {
      LINFO("INPUT Flicker Bias");
      for(ushort i = 0; i < itsMaxIndex; i++)
      {
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_FL]
                                               *itsScalePower[i],SC_FL);
      }
    }
    if(vc->hasSubChan("orientation"))
    {
      LINFO("INPUT Orientation Bias");
      for(ushort i = 0; i < itsMaxIndex; i++)
      {
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_GA0]
                                               *itsScalePower[i],SC_GA0);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_GA1]
                                               *itsScalePower[i],SC_GA1);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_GA2]
                                               *itsScalePower[i],SC_GA2);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_GA3]
                                               *itsScalePower[i],SC_GA3);
      }
    }
    if(vc->hasSubChan("motion"))
    {
      LINFO("INPUT Motion Bias");
      for(ushort i = 0; i < itsMaxIndex; i++)
      {
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_DR0]
                                               *itsScalePower[i],SC_DR0);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_DR1]
                                               *itsScalePower[i],SC_DR1);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_DR2]
                                               *itsScalePower[i],SC_DR2);
        itsSurpriseControl[i].SCsetConspicBias(itsConspicMapBias[SC_DR3]
                                               *itsScalePower[i],SC_DR3);
      }
    }
  }

  Image<FLOAT> ftmp;

  // ### H2SV
  if(vc->hasSubChan("H2SVcolor"))
  {
    LINFO("INPUT H2SV Channel");
    dynCastWeakToFrom(hc, vc->subChan("H2SVcolor"));

    if(hc->H1().numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in h1 H2SV color channel, got %d",
             itsMaxIndex,hc->H1().numSubmaps());
    if(hc->H2().numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in h2 H2SV color channel, got %d",
             itsMaxIndex,hc->H2().numSubmaps());
    if(hc->S().numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in s H2SV color channel, got %d",
             itsMaxIndex,hc->S().numSubmaps());
    if(hc->V().numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in v H2SV color channel, got %d",
             itsMaxIndex,hc->V().numSubmaps());
    for(ushort i = 0; i < itsMaxIndex; i++)
    {
      ftmp = hc->H1().getSubmap(i);
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_H1);

      ftmp = hc->H2().getSubmap(i);
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_H2);

      ftmp = hc->S().getSubmap(i);
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_HS);

      ftmp = hc->V().getSubmap(i);
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_HV);
    }
  }
  // ### Intensity
  // Since H2SV V is the same as intensity, do not include this twice
  else if(vc->hasSubChan("intensity"))
  {
    LINFO("INPUT Intensity Channel");
    dynCastWeakToFrom(ic, vc->subChan("intensity"));
    if(ic->numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in intensity channel, got %d",
             itsMaxIndex,ic->numSubmaps());
    for(ushort i = 0; i < itsMaxIndex; i++)
    {
      ftmp = ic->getSubmap(i);
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_IN);
    }
  }

  // ### Color
  if(vc->hasSubChan("color"))
  {
    LINFO("INPUT Color Channel");
    dynCastWeakToFrom(cc, vc->subChan("color"));
    if(cc->rg().numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in rg color channel, got %d",
             itsMaxIndex,cc->rg().numSubmaps());
    if(cc->by().numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in by color channel, got %d",
             itsMaxIndex,cc->by().numSubmaps());
    for(ushort i = 0; i < itsMaxIndex; i++)
    {
      ftmp = cc->rg().getSubmap(i);
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_RG);

      ftmp = cc->by().getSubmap(i);
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_BY);
    }
  }

  // ### flicker
  if(vc->hasSubChan("flicker"))
  {
    LINFO("INPUT Flicker Channel");
    dynCastWeakToFrom(fc, vc->subChan("flicker"));
    if(fc->numSubmaps() != itsMaxIndex)
      LFATAL("Expected %d scales from brain in flicker channel, got %d",
             itsMaxIndex,fc->numSubmaps());
    for(ushort i = 0; i < itsMaxIndex; i++)
    {
      //Do we have a pyramid yet? If not just fill up with zeros
      if(fc->hasPyramid())
      {
        ftmp = fc->getSubmap(i);
      }
      else
      {
        typename Image<FLOAT>::iterator ftmpItr = ftmp.beginw();
        while(ftmpItr != ftmp.endw())
        {
          *ftmpItr = 0;
          ++ftmpItr;
        }
      }
      ftmp = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_FL);
    }
  }

  // ### Orientation
  if(vc->hasSubChan("orientation"))
  {
    LINFO("INPUT Orientation Channel");
    dynCastWeakToFrom(oc, vc->subChan("orientation"));
    const uint imi = (uint)(itsMaxIndex*4);
    if(oc->numSubmaps() != imi)
      LFATAL("Expected %d scales from brain in orientation channel, got %d",
             imi,oc->numSubmaps());
    if(oc->numChans() != 4)
      LFATAL("I only work with a 4-gabor orientation channels!");
    for(ushort i = 0; i < itsMaxIndex; i++)
    {
      ftmp  = oc->gabor(0).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_GA0);

      ftmp  = oc->gabor(1).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_GA1);

      ftmp  = oc->gabor(2).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_GA2);

      ftmp  = oc->gabor(3).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_GA3);
    }
  }

  // ### Motion
  if(vc->hasSubChan("motion"))
  {
    LINFO("INPUT Motion Channel");
    dynCastWeakToFrom(mc, vc->subChan("motion"));
    const uint imi = (uint)(itsMaxIndex*4);
    if(mc->numSubmaps() != imi)
      LFATAL("Expected %d scales from brain in motion channel, got %d",
             imi,oc->numSubmaps());
    if(mc->numChans() != 4)
      LFATAL("I only work with a 4-direction motion channels!");
    for(ushort i = 0; i < itsMaxIndex; i++)
    {
      ftmp  = mc->dirChan(0).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_DR0);

      ftmp  = mc->dirChan(1).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_DR1);

      ftmp  = mc->dirChan(2).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_DR2);

      ftmp  = mc->dirChan(3).getSubmap(i);
      ftmp  = rescale(ftmp,itsImageSizeX,itsImageSizeY);
      itsSurpriseControl[i].SCinputConspicMap(ftmp,SC_DR3);
    }
  }

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    // process this scale
    itsSurpriseControl[i].SCprocessFrameSeperable();

    if(itsGetReduced)
    {
      // get result image for this scale
      itsResultImages[i] = itsSurpriseControl[i].SCgetFrame();
    }
    else
    {
      PIX_H2SV_TYPE<float> pix(itsSharpFactorH1,itsSharpFactorH2,
                               itsSharpFactorS, itsSharpFactorV);
      itsResultImages[i] = itsSurpriseControl[i].SCgetSharpened(pix);
    }
    LINFO("done");
  }

  // zero it out before we store new values here
  FLOAT norm = 0.0F;
  typename Image<PixRGB<FLOAT> >::iterator finalImageItr =
    itsFinalImage.beginw();
  while(finalImageItr !=  itsFinalImage.endw())
  {
    *finalImageItr = PixRGB<FLOAT>(0,0,0);
    ++finalImageItr;
  }

  // first form a biased sum over each scale
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    finalImageItr = itsFinalImage.beginw();
    typename Image<PixRGB<FLOAT> >::const_iterator resultImageItr =
      itsResultImages[i].beginw();
    const FLOAT bias = itsScaleBias[i];
    norm += bias;
    while(finalImageItr !=  itsFinalImage.endw())
    {
      *finalImageItr += (*resultImageItr) * bias;
      ++finalImageItr; ++resultImageItr;
    }
  }

  // then normalize into the final image
  for(typename Image<PixRGB<FLOAT> >::iterator finalImageItr =
        itsFinalImage.beginw();
      finalImageItr != itsFinalImage.endw();
      ++finalImageItr)
  {
    *finalImageItr = (*finalImageItr)/norm;
  }
  itsFrameCounter++;
  */
}

/*************************************************************************/

template <class FLOAT> inline
void ScaleSurpriseControl<FLOAT>::SSCprocessFrame(const uint frame)
{
  itsFrameCounter++;
}
/*************************************************************************/

template <class FLOAT> inline
Image<PixRGB<FLOAT> > ScaleSurpriseControl<FLOAT>::SSCgetFrame() const
{
  return itsFinalImage;
}

/*************************************************************************/

template <class FLOAT> inline Image<PixRGB<FLOAT> >
ScaleSurpriseControl<FLOAT>::SSCgetDiffImage(const bool normalize) const
{
  Image<PixRGB<float> > diffImage;
  diffImage.resize(itsImageSizeX,itsImageSizeY);

  const Image<PixRGB<float> > inImage = itsSurpriseControl[0].SCgetInImage();

  typename Image<PixRGB<float> >::const_iterator finalImageItr =
    itsFinalImage.begin();

  typename Image<PixRGB<float> >::const_iterator rawImageItr  =
    inImage.begin();

  typename Image<PixRGB<float> >::iterator diffImageItr =
    diffImage.beginw();

  while(rawImageItr != inImage.end())
  {
    //*diffImageItr = (*rawImageItr);
    //*diffImageItr = (*finalImageItr);
    *diffImageItr = abs((*rawImageItr) - (*finalImageItr));
    ++diffImageItr; ++rawImageItr; ++finalImageItr;
  }

  if(normalize) diffImage = normalizeRGB(diffImage,PixRGB<FLOAT>(0,0,0),
                                         PixRGB<FLOAT>(255,255,255));

  return diffImage;
}

/*************************************************************************/

template <class FLOAT> std::vector<Image<PixRGB<FLOAT> > >
ScaleSurpriseControl<FLOAT>::SSCgetDiffParts() const
{
  Image<PIX_H2SV_TYPE<FLOAT> > rawDiffImage;
  rawDiffImage.resize(itsImageSizeX,itsImageSizeY);

  typename Image<PIX_H2SV_TYPE<FLOAT> >::iterator rawDiffImageItr =
    rawDiffImage.beginw();

  while(rawDiffImageItr != rawDiffImage.end())
  {
    (*rawDiffImageItr) = PIX_H2SV_TYPE<FLOAT>(0);
    ++rawDiffImageItr;
  }

  for(ushort i = 0; i < itsMaxIndex; i++)
  //for(ushort i = 0; i < 1; i++)
  {
    const Image<PIX_H2SV_TYPE<FLOAT> > outImage =
      itsSurpriseControl[i].SCgetRawOutImage();
    const Image<PIX_H2SV_TYPE<FLOAT> > inImage  =
      itsSurpriseControl[i].SCgetRawInImage();

    typename Image<PIX_H2SV_TYPE<FLOAT> >::const_iterator outImageItr     =
      outImage.begin();
    typename Image<PIX_H2SV_TYPE<FLOAT> >::const_iterator inImageItr      =
      inImage.begin();

    rawDiffImageItr = rawDiffImage.beginw();

    //while(outImageItr != outImage.end())
    while(inImageItr != inImage.end())
    {
      //*rawDiffImageItr += (*outImageItr);
      //*rawDiffImageItr += (*inImageItr);
      (*rawDiffImageItr) += (*outImageItr) - (*inImageItr);
      ++outImageItr; ++inImageItr; ++rawDiffImageItr;
    }
  }

  while(rawDiffImageItr != rawDiffImage.end())
  {
    (*rawDiffImageItr) = (*rawDiffImageItr)/itsMaxIndex;
    ++rawDiffImageItr;
  }

  Image<FLOAT> baseImage;
  baseImage.resize(itsImageSizeX,itsImageSizeY);
  std::vector<Image<FLOAT> > outImageVec(4,baseImage);


  for(ushort x = 0; x < itsImageSizeX; x++)
  {
    for(ushort y = 0; y < itsImageSizeY; y++)
    {
      outImageVec[0].setVal(x,y,rawDiffImage.getVal(x,y).p[0]);
      outImageVec[1].setVal(x,y,rawDiffImage.getVal(x,y).p[1]);
      outImageVec[2].setVal(x,y,rawDiffImage.getVal(x,y).p[2]);
      outImageVec[3].setVal(x,y,rawDiffImage.getVal(x,y).p[3]);
    }
  }

  Image<PixRGB<FLOAT> > baseImageRGB;
  baseImageRGB.resize(itsImageSizeX,itsImageSizeY);
  std::vector<Image<PixRGB<FLOAT> > > outImageRGVec(4,baseImageRGB);

  (outImageRGVec[0]) = normalizeRGPolarAuto(outImageVec[0]);

  (outImageRGVec[1]) = normalizeRGPolarAuto(outImageVec[1]);

  (outImageRGVec[2]) = normalizeRGPolarAuto(outImageVec[2]);

  (outImageRGVec[3]) = normalizeRGPolarAuto(outImageVec[3]);

  return outImageRGVec;
}

/*************************************************************************/

template <class FLOAT> inline std::vector<Image<FLOAT> >
ScaleSurpriseControl<FLOAT>::SSCgetBetaParts(const bool normalize) const
{
  Image<PixHyper<FLOAT,SC_MAX_CHANNELS> > outBetaImage;
  outBetaImage.resize(itsImageSizeX,itsImageSizeY);

  /*
  for(typename Image<PixHyper<FLOAT,SC_MAX_CHANNELS> >::iterator bptr
        = outBetaImage.beginw();
      bptr != outBetaImage.endw();
      ++bptr)
  {
    for(ushort i = 0; i < SC_MAX_CHANNELS; i++)
    {
      bptr->p[i] = 0;
    }
  }
  */

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    const Image<PixHyper<FLOAT,SC_MAX_CHANNELS> > betaImage =
      itsSurpriseControl[i].SCgetBetaImage();

    typename Image<PixHyper<FLOAT,SC_MAX_CHANNELS> >::const_iterator
      betaImageItr    = betaImage.begin();
    typename Image<PixHyper<FLOAT,SC_MAX_CHANNELS> >::iterator
      outBetaImageItr = outBetaImage.beginw();

    PixHyper<FLOAT,SC_MAX_CHANNELS> pIndex((FLOAT)itsMaxIndex);

    while(betaImageItr != betaImage.end())
    {
      (*outBetaImageItr) += (*betaImageItr)/pIndex;
      ++betaImageItr; ++outBetaImageItr;
    }
  }

  Image<FLOAT> baseImage;
  baseImage.resize(itsImageSizeX,itsImageSizeY);
  std::vector<Image<FLOAT> > outImageVec(SC_MAX_CHANNELS,baseImage);

  for(ushort x = 0; x < itsImageSizeX; x++)
  {
    for(ushort y = 0; y < itsImageSizeY; y++)
    {
      for(ushort i = 0; i < SC_MAX_CHANNELS; i++)
      {
        outImageVec[i].setVal(x,y,outBetaImage.getVal(x,y).p[i]);
      }
    }
  }

  if(normalize)
  {
    for(ushort i = 0; i < SC_MAX_CHANNELS; i++)
    {
      outImageVec[i] = normalizeFloat(outImageVec[i],FLOAT_NORM_0_255);
    }
  }
  return outImageVec;
}

/*************************************************************************/

template <class FLOAT> inline void
ScaleSurpriseControl<FLOAT>::SSCgetBiasParts(
                             std::vector<Image<PixRGB<FLOAT> > > &H1,
                             std::vector<Image<PixRGB<FLOAT> > > &H2,
                             std::vector<Image<PixRGB<FLOAT> > > &S,
                             std::vector<Image<PixRGB<FLOAT> > > &V) const
{
  Image<PixRGB<FLOAT> > baseImage;
  baseImage.resize(itsImageSizeX,itsImageSizeY);
  /*
  for(typename Image<PixRGB<FLOAT> >::iterator bptr = baseImage.beginw();
      bptr != baseImage.endw();
      ++bptr)
  {
    *bptr = PixRGB<FLOAT>(0);
  }
  */
  std::vector<Image<PixRGB<FLOAT> > > outH1(itsMaxIndex,baseImage);
  std::vector<Image<PixRGB<FLOAT> > > outH2(itsMaxIndex,baseImage);
  std::vector<Image<PixRGB<FLOAT> > > outS(itsMaxIndex, baseImage);
  std::vector<Image<PixRGB<FLOAT> > > outV(itsMaxIndex, baseImage);

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    Image<FLOAT> h1; Image<FLOAT> h2; Image<FLOAT> s; Image<FLOAT> v;
    itsSurpriseControl[i].SCgetLocalBiasImages(h1, h2, s, v);

    outH1[i] = normalizeScaleRainbow(h1,0,1);
    outH2[i] = normalizeScaleRainbow(h2,0,1);
    outS[i]  = normalizeScaleRainbow(s, 0,1);
    outV[i]  = normalizeScaleRainbow(v, 0,1);
    //outH1[i] = h1;
  }
  H1 = outH1; H2 = outH2; S = outS; V = outV;
}

/*************************************************************************/

template <class FLOAT> inline void
ScaleSurpriseControl<FLOAT>::SSCgetSeperableParts(
                             std::vector<Image<PixRGB<FLOAT> > > &Zimgs,
                             std::vector<Image<PixRGB<FLOAT> > > &Yimgs,
                             const bool normalize) const
{
  Image<PixRGB<FLOAT> > baseImage;
  baseImage.resize(itsImageSizeX,itsImageSizeY);
  /*
  for(typename Image<PixRGB<FLOAT> >::iterator bptr = baseImage.beginw();
      bptr != baseImage.endw();
      ++bptr)
  {
    *bptr = PixRGB<FLOAT>(0);
  }
  */
  std::vector<Image<PixRGB<FLOAT> > > outZ(itsMaxIndex,baseImage);
  std::vector<Image<PixRGB<FLOAT> > > outY(itsMaxIndex,baseImage);

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    Image<PIX_H2SV_TYPE<FLOAT> > Zimg, Yimg;

    itsSurpriseControl[i].SCgetSeperableParts(Zimg,Yimg);

    outZ[i] = static_cast<Image<PixRGB<FLOAT> > >(Zimg);
    outY[i] = static_cast<Image<PixRGB<FLOAT> > >(Yimg);

    if(normalize)
    {
      outZ[i] = normalizeRGB(outZ[i],PixRGB<FLOAT>(0,0,0),
                             PixRGB<FLOAT>(255,255,255));
      outY[i] = normalizeRGB(outY[i],PixRGB<FLOAT>(0,0,0),
                             PixRGB<FLOAT>(255,255,255));
    }
  }

  Zimgs = outZ;
  Yimgs = outY;
}

/*************************************************************************/
template class ScaleSurpriseControl<float>;

#endif
