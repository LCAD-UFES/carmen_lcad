/*!@file Surprise/SurpriseControl.C attempt to control surprise in an image  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseControl.C $
// $Id: SurpriseControl.C 15310 2012-06-01 02:29:24Z itti $
//

#ifndef SURPRISE_CONTROL_C_DEFINED
#define SURPRISE_CONTROL_C_DEFINED

#include "Surprise/SurpriseControl.H"

#include <typeinfo>
#include <climits>
#include <cfloat>

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SurpriseControl(const ushort sizeX,
                                                         const ushort sizeY)
{
  LINFO("Creating SurpriseControl Object");


  SCinit(sizeX,sizeY);
  SCuseMaxLevel(false);

  if(typeid(BETATYPE).name() != typeid(itsHyper).name())
  {
    LFATAL("Run time type error. Type: %s is not supported",
           typeid(BETATYPE).name());
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SurpriseControl()
{
  LINFO("Creating SurpriseControl Object NOTE: Call init()");

  if(typeid(BETATYPE).name() != typeid(itsHyper).name())
  {
    LFATAL("Run time type error. Type: %s is not supported",
           typeid(BETATYPE).name());
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::~SurpriseControl()
{}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCuseMaxLevel(const bool useML)
{
  LINFO("Setting itsUseMaxLevel");
  itsUseMaxLevel = useML;
}

/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCuseTemporal(const bool useTMP)
{
  LINFO("Setting itsUseTemporal");
  itsUseTemporal = useTMP;
}

/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>:: SCnormalizeBiasWithScale(
                                                        const bool useNBS)
{
  LINFO("Setting itsNormalizeBiasWithScale");
  itsNormalizeBiasWithScale = useNBS;
}

/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetMasterConspicBias(
                                                    const FLOAT bias)
{
  itsMasterConspicBias = bias;
}

/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetConspicBias(
                                              const FLOAT chan,
                                              const int chan_enum)
{
  //LINFO("Using %s map - bias %f",sc_channel_name[chan_enum].c_str(),chan);
  itsConspicMapBias[chan_enum] = chan;
  itsUseConspicMap[chan_enum]  = true;
}
/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetMyScale(const ushort scale)
{
  itsScale = scale;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetAxisBias(const FLOAT X,
                                                            const FLOAT Y,
                                                            const FLOAT Z)
{
  itsXBias = X; itsYBias = Y; itsZBias = Z;
}
/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetH2SVBias(const FLOAT H1,
                                                            const FLOAT H2,
                                                            const FLOAT S,
                                                            const FLOAT V)
{
  itsH1Bias = H1; itsH2Bias = H2; itsSBias = S; itsVBias = V;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCinit(const ushort sizeX,
                                                     const ushort sizeY)
{
  LINFO("INIT: image size %d x %d",sizeX,sizeY);

  for(ushort i = 0; i < SC_MAX_CHANNELS; i++)
  {
    itsConspicMapBias[i]  = 1.0F;
    itsUseConspicMap[i]   = false;
  }

  itsIterCounter = 0;
  itsScale       = 0;

  itsXBias = 1.0F; itsYBias = 1.0F; itsZBias = 1.0F;

  itsLambda                 = 0.1;
  itsInitBuffer             = false;
  itsUseCorrMatrixSet       = false;
  itsUseBayesWeightImage    = false;
  itsUseMaskImage           = false;
  itsUseTargetFrame         = false;
  itsUseTemporal            = true;
  itsBufferFull             = false;
  itsNormalizeBiasWithScale = false;
  itsImageSizeX             = sizeX;
  itsImageSizeY             = sizeY;

  itsTargetFrame         = 0;

  itsInImage.resize(sizeX,sizeY);
  itsOutImage.resize(sizeX,sizeY);
  itsFinalImage.resize(sizeX,sizeY);
  itsLocalBiasH1.resize(sizeX,sizeY);
  itsLocalBiasH2.resize(sizeX,sizeY);
  itsLocalBiasS.resize(sizeX,sizeY);
  itsLocalBiasV.resize(sizeX,sizeY);
  itsSmallSaliency.resize(sizeX,sizeY);
  BETATYPE blank(0.0F);
  PIXTYPE  blank2(0.0F);

  Image<BETATYPE> initBeta; initBeta.resize(sizeX,sizeY);
  typename Image<BETATYPE>::iterator betaImageItr = initBeta.beginw();
  Image<PIXTYPE> tempImage;
  tempImage.resize(sizeX,sizeY);
  typename Image<PIXTYPE>::iterator tempImageItr = tempImage.beginw();
  while(betaImageItr != initBeta.endw())
  {
    *betaImageItr = blank;
    *tempImageItr = blank2;
    ++betaImageItr; ++tempImageItr;
  }

  itsBetaImage.push_front(initBeta);

  // resize intermediate images, set equal to the blank beta image

  itsInterImage.resize(2,tempImage);
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetLambda(const FLOAT lambda)
{
  itsLambda = lambda;
}
// 1. create a guassian filtered frame of the current movie
/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetTargetFrame(const uint frame)
{
  itsTargetFrame    = frame;
  itsUseTargetFrame = true;
}
// 2. merge the guassian filtered frame with the current frame giving more
//    weight to filtered pixels if the surprise is higher.

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCsetOriginalImageWeight(
                                              const FLOAT origImageWeight)
{
  itsOriginalImageWeight = origImageWeight;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCcreateAndersonSepFilters(
                                                             const ushort size)
{
  const ushort basesize = 5;

  if(size%basesize != 0)
    LFATAL("This kernel size must be evenly divisable by 5 got %d",size);

  const FLOAT anderson[basesize] = {1.0/16.0, 4.0/16.0, 6.0/16.0,
                                    4.0/16.0, 1.0/16.0};
  for(ushort i = 0; i < basesize; i++)
  {
    LINFO("%f",anderson[i]);
  }
  const short div = size/basesize;

  bool evensize;

  if(size%2 == 0)
  {
    evensize = true;
    itsKernelSizeX = size + 1;
    itsKernelSizeY = size + 1;
  }
  else
  {
    itsKernelSizeX = size;
    itsKernelSizeY = size;
    evensize = false;
  }
  itsKernelSizeZ = basesize;

  itsKalmanKernelX.resize(itsKernelSizeX,0.0F);
  itsKalmanKernelY.resize(itsKernelSizeY,0.0F);
  itsKalmanKernelZ.resize(itsKernelSizeZ,0.0F);

  const FLOAT centerX = floor((FLOAT)itsKernelSizeX/2.0F);

  ushort offset = 0;
  LINFO("Kernel (Anderson Variant) size %d",itsKernelSizeX);
  for(ushort i = 0; i < itsKernelSizeX; i++)
  {
    if((i < centerX) || (evensize == false))
    {
      if((i%div == 0) && (i != 0))
        offset++;
    }
    else
    {
      if((i - 1)%div == 0)
        offset++;
    }

    itsKalmanKernelX[i] = anderson[offset];
    itsKalmanKernelY[i] = anderson[offset];
    std::cerr << i << " : " << itsKalmanKernelX[i] << "\n";
  }

  for(ushort i = 0; i < itsKernelSizeZ; i++)
  {
    itsKalmanKernelZ[i] = anderson[i];
  }

  itsImageSizeZ = itsKernelSizeZ;
  itsTemporalOffset = (ushort)floor(itsKernelSizeZ/2.0);
}

/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCcreateSepFilters(
                                              const FLOAT spatSigma,
                                              const FLOAT tempSigma,
                                              const FLOAT stdDevSize)
{
  LINFO("ASAC SETTING UP KERNELS");
  // create three components for a std seperable guassian kernel
  ushort rs1 = (ushort)ceil(spatSigma*stdDevSize*2.0F);
  ushort rs2 = (ushort)floor(spatSigma*stdDevSize*2.0F);
  // make sure the kernel is odd sized spatial
  if(rs1%2 == 0)
  {
    if(rs2%2 == 0)
    {
      itsKernelSizeX = rs2 + 1;
      itsKernelSizeY = rs2 + 1;
    }
    else
    {
      itsKernelSizeX = rs2;
      itsKernelSizeY = rs2;
    }
  }
  else
  {
    itsKernelSizeX = rs1;
    itsKernelSizeY = rs1;
  }
  itsKalmanKernelX.resize(itsKernelSizeX,0.0F);
  itsKalmanKernelY.resize(itsKernelSizeY,0.0F);

  rs1 = (ushort)ceil((tempSigma*stdDevSize*2.0F));
  rs2 = (ushort)floor((tempSigma*stdDevSize*2.0F));
  // make sure the kernel is odd sized temporal
  if(rs1%2 == 0)
  {
    if(rs2%2 == 0)
    {
      itsKernelSizeZ = rs2 + 1;
    }
    else
    {
      itsKernelSizeZ = rs2;
    }
  }
  else
  {
    itsKernelSizeZ = rs1;
  }
  itsKalmanKernelZ.resize(itsKernelSizeZ,0.0F);

  itsImageSizeZ = itsKernelSizeZ;

  // find the kernel center

  const FLOAT centerX = floor((FLOAT)itsKernelSizeX/2.0F);
  const FLOAT centerY = floor((FLOAT)itsKernelSizeY/2.0F);
  const FLOAT centerZ = floor((FLOAT)itsKernelSizeZ/2.0F);

  FLOAT gmod = 1.0F/sqrt(2.0F*M_PI*pow(spatSigma,2));

  LINFO("Kernel X size %d",itsKernelSizeX);
  for(ushort i = 0; i < itsKernelSizeX; i++)
  {
    const FLOAT dist = pow((i - centerX),2);
    itsKalmanKernelX[i] =
      (gmod*exp((-1.0F*dist)/pow(2*spatSigma,2)));
    std::cerr << i << " : " << itsKalmanKernelX[i] << "\n";
  }
  LINFO("Kernel Y size %d",itsKernelSizeY);
  for(ushort i = 0; i < itsKernelSizeY; i++)
  {
    const FLOAT dist = pow((i - centerY),2);
    itsKalmanKernelY[i] =
      (gmod*exp((-1.0F*dist)/pow(2*spatSigma,2)));
    std::cerr << i << " : " << itsKalmanKernelY[i] << "\n";
  }

  gmod = 1.0F/sqrt(2.0F*M_PI*pow(tempSigma,2));
  LINFO("Kernel Z size %d",itsKernelSizeZ);
  for(ushort i = 0; i < itsKernelSizeZ; i++)
  {
    const FLOAT dist = pow((i - centerZ),2);
    itsKalmanKernelZ[i] =
      (gmod*exp((-1.0F*dist)/pow(2*tempSigma,2)));
    std::cerr << i << " : " << itsKalmanKernelZ[i] << "\n";
  }

  itsTemporalOffset = (ushort)floor(itsKernelSizeZ/2.0);
}

/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCfindConvolutionEndPoints()
{
  LINFO("SETTING UP CONVOLUTION END POINTS");

  itsXStart.resize(itsImageSizeX,itsImageSizeY);
  itsYStart.resize(itsImageSizeX,itsImageSizeY);
  itsZStart.resize(itsImageSizeX,itsImageSizeY);
  itsXStop.resize( itsImageSizeX,itsImageSizeY);
  itsYStop.resize( itsImageSizeX,itsImageSizeY);
  itsZStop.resize( itsImageSizeX,itsImageSizeY);
  itsKXStart.resize(itsImageSizeX,itsImageSizeY);
  itsKYStart.resize(itsImageSizeX,itsImageSizeY);
  itsKZStart.resize(itsImageSizeX,itsImageSizeY);

  Image<ushort>::iterator itsZStartItr  = itsZStart.beginw();
  Image<ushort>::iterator itsZStopItr   = itsZStop.beginw();
  Image<ushort>::iterator itsKZStartItr = itsKZStart.beginw();

  Image<ushort>::iterator itsYStartItr  = itsYStart.beginw();
  Image<ushort>::iterator itsYStopItr   = itsYStop.beginw();
  Image<ushort>::iterator itsKYStartItr = itsKYStart.beginw();

  Image<ushort>::iterator itsXStartItr  = itsXStart.beginw();
  Image<ushort>::iterator itsXStopItr   = itsXStop.beginw();
  Image<ushort>::iterator itsKXStartItr = itsKXStart.beginw();

  for(ushort y = 0; y < itsImageSizeY; y++)
  {
    for(ushort x = 0; x < itsImageSizeX; x++)
    {
      // Z is not interesting, its starts at the current frame and
      // runs the size of the deque

      *itsKZStartItr = 0;
      *itsZStartItr  = 0;
      *itsZStopItr   = itsKernelSizeZ;

      const ushort yhalf = (ushort)floor((FLOAT)itsKernelSizeY/2.0F);
      const ushort yend  = itsImageSizeY - yhalf;
      if(y < yhalf + 1)
      {
        *itsYStartItr  = 0;
        *itsKYStartItr = yhalf - y;
        *itsYStopItr   = y + yhalf;
      }
      else if(y > yend)
      {
        *itsYStartItr  = y - yhalf;
        *itsKYStartItr = 0;
        *itsYStopItr   = itsImageSizeY;
      }
      else
      {
        *itsYStartItr  = y - yhalf;
        *itsKYStartItr = 0;
        *itsYStopItr   = y + yhalf;
      }

      const ushort xhalf = (ushort)floor((FLOAT)itsKernelSizeX/2.0F);
      const ushort xend  = itsImageSizeX - xhalf;
      if(x < xhalf + 1)
      {
        *itsXStartItr  = 0;
        *itsKXStartItr = xhalf - x;
        *itsXStopItr   = x + xhalf;
      }
      else if(x > xend)
      {
        *itsXStartItr  = x - xhalf;
        *itsKXStartItr = 0;
        *itsXStopItr   = itsImageSizeX;
      }
      else
      {
        *itsXStartItr  = x - xhalf;
        *itsKXStartItr = 0;
        *itsXStopItr   = x + xhalf;
      }

      ++itsZStartItr; ++itsZStopItr; ++itsKZStartItr;
      ++itsYStartItr; ++itsYStopItr; ++itsKYStartItr;
      ++itsXStartItr; ++itsXStopItr; ++itsKXStartItr;
    }
  }
}


/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCinputRawImage(
                                     const Image<PixRGB<FLOAT> >& rawImage)
{
  //LINFO("INPUT RAW IMAGE");

  if(rawImage.getHeight() != itsImageSizeY)
  {
    LINFO("Input raw image is not the correct size");
    LFATAL("Raw %d != sizeY %d",rawImage.getHeight(),itsImageSizeY);
  }
  if(rawImage.getWidth() != itsImageSizeX)
  {
    LINFO("Input raw image is not the correct size");
    LFATAL("Raw %d != sizeX %d",rawImage.getWidth(),itsImageSizeX);
  }

  typename Image<PIXTYPE >::iterator inImageItr =
    itsInImage.beginw();
  typename Image<PixRGB<FLOAT> >::const_iterator rawImageItr =
    rawImage.begin();

  // convert the input image to H2SV2
  while(inImageItr != itsInImage.endw())
  {
    *inImageItr = PIXTYPE(*rawImageItr);
    ++inImageItr; ++rawImageItr;
  }

  itsFrameBuffer.push_front(itsInImage);
  itsInitBuffer = true;

  //if we get to big, pop off the back image
  if(itsFrameBuffer.size() > (unsigned)(itsImageSizeZ + 1))
  {
    itsFrameBuffer.pop_back();
    itsBufferFull = true;
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCinputSalMap(
                                              const Image<FLOAT>& salMap)
{
  if(itsInImage.getWidth() != salMap.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Sal Map Width %d != Raw Image Width %d",salMap.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != salMap.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Sal Map Height %d != Raw Image Height %d",salMap.getHeight(),
           itsInImage.getHeight());
  }

  // normalize from 0 to 1
  itsSalMap = salMap/255.0F;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCinputConspicMap(
                                              const Image<FLOAT>& cmap,
                                              const int cmap_enum)
{
  if(itsUseConspicMap[cmap_enum] != true)
  {
    LINFO("Called to input %s image, but did not expect it",
          sc_channel_name[cmap_enum].c_str());
    LFATAL("Be sure to set the bias first!");
  }
  if(itsInImage.getWidth() != cmap.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",cmap.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != cmap.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",cmap.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  itsConspicMap[cmap_enum] = cmap/255.0F;
  itsConspicMap[cmap_enum] = itsConspicMap[cmap_enum] *
                             itsConspicMapBias[cmap_enum];

  itsConspicMap[cmap_enum] = itsConspicMap[cmap_enum] * itsMasterConspicBias;


  // if we use a bayes image, weight here
  if(itsUseBayesWeightImage)
  {
    if(itsConspicMap[cmap_enum].getWidth() != itsBayesWeightImage.getWidth())
      LFATAL("Bayes image must be same width as conspicuity maps");
    if(itsConspicMap[cmap_enum].getHeight() != itsBayesWeightImage.getHeight())
      LFATAL("Bayes image must be same height as conspicuity maps");

    typename Image<FLOAT>::iterator icmap = itsConspicMap[cmap_enum].beginw();
    typename Image<FLOAT>::const_iterator bayes = itsBayesWeightImage.begin();
    while(icmap != itsConspicMap[cmap_enum].endw())
    {
      *icmap = (*icmap) * (*bayes);
      ++icmap; ++bayes;
    }
  }
  // if we use a mask, modify conspic maps here
  if(itsUseMaskImage)
  {
    if(itsConspicMap[cmap_enum].getWidth() != itsMaskImage.getWidth())
      LFATAL("Mask image must be same width as conspicuity maps");
    if(itsConspicMap[cmap_enum].getHeight() != itsMaskImage.getHeight())
      LFATAL("Mask image must be same height as conspicuity maps");

    typename Image<FLOAT>::iterator icmap = itsConspicMap[cmap_enum].beginw();
    typename Image<FLOAT>::const_iterator mask  = itsMaskImage.begin();
    while(icmap != itsConspicMap[cmap_enum].endw())
    {
      *icmap = (*icmap) * (*mask);

      ++icmap; ++mask;
    }
  }

  if(SC_DEBUG)
  {
    char name[100];
    sprintf(name,"debug-conspic-%d-%d-%d.png",itsScale,itsIterCounter,cmap_enum);
    std::string prefix    = name;
    Image<PixRGB<FLOAT> > temp =
      normalizeScaleRainbow(itsConspicMap[cmap_enum],0,5);
    Raster::WriteRGB(temp, prefix);
  }

}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCinputBayesWeightImage(
                                              const Image<FLOAT> &bayesImage)
{
  LINFO("INPUT Bayes Weight Image");
  itsUseBayesWeightImage = true;
  itsBayesWeightImage = bayesImage;
}
/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCinputMaskImage(
                                              const Image<FLOAT> &maskImage)

{
  FLOAT mi, ma;
  getMinMax(maskImage,mi,ma);
  if(mi < 0)
  {
    LINFO("Mask Image value too low = %f",mi);
    LFATAL("Mask Image pixel values must be from 0 to 1");
  }
  if(ma > 1)
  {
    LINFO("Mask Image value too high = %f",ma);
    LFATAL("Mask Image pixel values must be from 0 to 1");
  }
  LINFO("INPUT independant mask image");
  itsUseMaskImage = true;
  itsMaskImage = maskImage;
}
/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCcomputeNewBeta()
{


  // Go through each conspicuity map and if we are using it, compute
  // beta as a kalman smoothed set of frames.
  // buffer beta so that it can align with the temporal components

  Image<BETATYPE> betaImage; betaImage.resize(itsImageSizeX,itsImageSizeY);
  for(ushort i = 0; i < SC_MAX_CHANNELS; i++)
  {
    if(itsUseConspicMap[i])
    {
      typename Image<BETATYPE>::iterator betaImageItr;
      typename Image<FLOAT>::iterator consMapItr               =
        itsConspicMap[i].beginw();
      typename Image<BETATYPE>::const_iterator betaImageOldItr =
        itsBetaImage[0].begin();
      typename Image<FLOAT>::const_iterator itsBayesWeightItr  =
        itsBayesWeightImage.begin();

      for(betaImageItr = betaImage.beginw();
          betaImageItr != betaImage.endw();
          ++betaImageItr,      ++consMapItr,
          ++itsBayesWeightItr, ++betaImageOldItr)
      {
        if(itsUseBayesWeightImage)
        {
          *consMapItr = *consMapItr * (*itsBayesWeightItr);
        }

        betaImageItr->p[i] =
          (betaImageOldItr->p[i] * itsLambda + *consMapItr)/
          (1 + 1 * itsLambda);
      }

      if(SC_DEBUG)
      {
        Image<FLOAT> ftemp; ftemp.resize(betaImage.getWidth(),
                                         betaImage.getHeight());

        typename Image<FLOAT>::iterator itmp = ftemp.beginw();

        for(betaImageItr = betaImage.beginw();
            betaImageItr != betaImage.endw();
            ++betaImageItr, ++itmp)
          *itmp = betaImageItr->p[i];

        char name[100];
        sprintf(name,"debug-beta-%d-%d-%d.png",itsScale,itsIterCounter,i);
        std::string prefix    = name;
        Image<PixRGB<FLOAT> > temp =
          normalizeScaleRainbow(ftemp,0,5);
        Raster::WriteRGB(temp, prefix);
      }
    }
  }
  itsBetaImage.push_front(betaImage);

  //if we get to big, pop off the back image
  if(itsBetaImage.size() > (unsigned)(itsImageSizeZ + 1))
    itsBetaImage.pop_back();
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCprocessFrameSeperable()
{
  LINFO("PROCESSING FRAME Kernel Size X - %d Y - %d Z - %d",
        itsKernelSizeX,
        itsKernelSizeY,
        itsKernelSizeZ);

  typename Image<PixRGB<FLOAT> >::iterator   finalImageItr =
    itsFinalImage.beginw();

  // copy itsInImage to itsOutImage so we can take the conspicuity weighted
  // average of the two

  // Derive the new bias with low pass smoothing

  SCcomputeNewBeta();

  LINFO("Buffer %" ZU " ksize %d",itsFrameBuffer.size(),itsKernelSizeZ);
  if(itsFrameBuffer.size() > (uint)itsTemporalOffset)
  {
    itsOutputReady  = true;
    itsFrameCurrent = &itsFrameBuffer[(uint)itsTemporalOffset];
    itsBetaCurrent  = &itsBetaImage[(uint)itsTemporalOffset];

    SCcomputeLocalBias();

    if(itsUseTemporal)
    {
      LINFO("Convolving image x,y,z");
      SCseperateConvXYZ();
    }
    else
    {
      LINFO("Convolving image x,y");
      SCseperateConvXY();
    }

    LINFO("FINISHING image");
    typename Image<PIXTYPE >::const_iterator outImageItr = itsOutImage.begin();
    for(finalImageItr  = itsFinalImage.beginw();
        finalImageItr != itsFinalImage.endw();
        ++finalImageItr, ++outImageItr)
    {
      *finalImageItr = PixRGB<FLOAT>(*outImageItr);
    }
    LINFO("DONE");
  }
  else
  {
    LINFO("FRAME BUFFER NOT YET HALF FULL");
    itsOutputReady = false;
    for(finalImageItr  = itsFinalImage.beginw();
        finalImageItr != itsFinalImage.endw();
        ++finalImageItr)
    {
      *finalImageItr = PixRGB<FLOAT>(0,0,0);
    }
  }
  itsIterCounter++;
}

/*************************************************************************/
template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCcomputeLocalBias()
{

  if(!(itsUseConspicMap[SC_DR0] == itsUseConspicMap[SC_DR1] &&
       itsUseConspicMap[SC_DR0] == itsUseConspicMap[SC_DR2] &&
       itsUseConspicMap[SC_DR0] == itsUseConspicMap[SC_DR3]))
    LFATAL("You must use all four direction channels or none!");

  if(!(itsUseConspicMap[SC_GA0] == itsUseConspicMap[SC_GA1] &&
       itsUseConspicMap[SC_GA0] == itsUseConspicMap[SC_GA2] &&
       itsUseConspicMap[SC_GA0] == itsUseConspicMap[SC_GA3]))
    LFATAL("You must use all four orientation channels or none!");

  if(!(itsUseConspicMap[SC_H1] == itsUseConspicMap[SC_H2] &&
       itsUseConspicMap[SC_H1] == itsUseConspicMap[SC_HS] &&
       itsUseConspicMap[SC_H1] == itsUseConspicMap[SC_HV]))
    LFATAL("You must use all four H2SV channels or none!");

  if(!(itsUseConspicMap[SC_RG] == itsUseConspicMap[SC_BY]))
    LFATAL("You must use both RG and BY channels or none!");
  Image<bool>::iterator  iSmallS;

  // find if any parts have a very small surprise
  iSmallS = itsSmallSaliency.beginw();
  for(typename Image<BETATYPE>::const_iterator betaImageItr =
        itsBetaCurrent->begin(); betaImageItr != itsBetaCurrent->end();
      ++betaImageItr, ++iSmallS)
  {
    // get the surprise biases smoothed
    //*iSmallS = false;
    *iSmallS = true;

    for(ushort i = 0; i < SC_MAX_CHANNELS; i++)
    {
      if(betaImageItr->p[i] > SMALL_SALIENCY)
        *iSmallS = false;
    }
  }

  // reset all localbias maps, set to zero
  typename Image<FLOAT>::iterator iBiasH1 = itsLocalBiasH1.beginw();
  typename Image<FLOAT>::iterator iBiasH2 = itsLocalBiasH2.beginw();
  typename Image<FLOAT>::iterator iBiasS  = itsLocalBiasS.beginw();
  typename Image<FLOAT>::iterator iBiasV  = itsLocalBiasV.beginw();

  for(typename Image<BETATYPE>::const_iterator betaImageItr =
        itsBetaCurrent->begin(); betaImageItr != itsBetaCurrent->end();
      ++betaImageItr,
        ++iBiasH1, ++iBiasH2, ++iBiasS, ++iBiasV, ++iSmallS)
  {
    *iBiasH1 = 0; *iBiasH2 = 0; *iBiasS = 0; *iBiasV = 0;
  }

  // compute bias using maximum saliency value
  if(itsUseMaxLevel)
  {

    iBiasH1 = itsLocalBiasH1.beginw();
    iBiasH2 = itsLocalBiasH2.beginw();
    iBiasS  = itsLocalBiasS.beginw();
    iBiasV  = itsLocalBiasV.beginw();

    for(typename Image<BETATYPE>::const_iterator betaImageItr =
          itsBetaCurrent->begin(); betaImageItr != itsBetaCurrent->end();
        ++betaImageItr,
          ++iBiasH1, ++iBiasH2, ++iBiasS, ++iBiasV)
    {
      // compute the surprise bias combinations with the axis bias
      /* If we use max level then surprise removal is scaled by the max
         level of all conspicuity maps, this limits reduction to be no more
         than the highest conspicuity map and prevents to much application
         at certian locations. However, it may not scale properly
      */

      // Bias color by RG/BY or H2SV

      const FLOAT *betaImg = betaImageItr->p;

      if(itsUseConspicMap[SC_H1])
      {
        *iBiasH1 = betaImg[SC_H1];
        *iBiasH2 = betaImg[SC_H2];
        *iBiasS  = betaImg[SC_HS];
        *iBiasV  = betaImg[SC_HV];
      }
      else
      {
        if(itsUseConspicMap[SC_BY])
        {
          if(betaImageItr->p[SC_BY] > betaImageItr->p[SC_RG])
          {
            *iBiasH1 = betaImg[SC_BY];
            *iBiasH2 = betaImg[SC_BY];
            *iBiasS  = betaImg[SC_BY];
          }
          else
          {
            *iBiasH1 = betaImg[SC_RG];
            *iBiasH2 = betaImg[SC_RG];
            *iBiasS  = betaImg[SC_RG];
          }
        }
      }
      // Bias Value by the max of all feature types
      FLOAT newVal = 0;
      for(ushort i = 0; i < SC_MAX_CHANNELS; i++)
      {
        if(betaImg[i] > newVal && itsUseConspicMap[i])
          newVal = betaImg[i];
      }
      *iBiasV = newVal;
    }
  }
  else
  {
    iBiasH1 = itsLocalBiasH1.beginw();
    iBiasH2 = itsLocalBiasH2.beginw();
    iBiasS  = itsLocalBiasS.beginw();
    iBiasV  = itsLocalBiasV.beginw();

    for(typename Image<BETATYPE>::const_iterator betaImageItr =
          itsBetaCurrent->begin(); betaImageItr != itsBetaCurrent->end();
        ++betaImageItr,
          ++iBiasH1, ++iBiasH2, ++iBiasS, ++iBiasV)
    {
      // compute the surprise bias combinations with the axis bias
      /* If we use max level then surprise removal is scaled by the max
         level of all conspicuity maps, this limits reduction to be no more
         than the highest conspicuity map and prevents to much application
         at certian locations. However, it may not scale properly
      */

      const FLOAT *betaImg = betaImageItr->p;
      // get the surprise biases smoothed
      // First bias by each feature type if we have it
      if(itsUseConspicMap[SC_H1])
      {
        *iBiasH1 = betaImg[SC_H1];
        *iBiasH2 = betaImg[SC_H2];
        *iBiasS  = betaImg[SC_HS];
        *iBiasV  = betaImg[SC_HV];
      }
      else
      {
        if(itsUseConspicMap[SC_BY])
        {
          *iBiasH1 = (betaImg[SC_BY] + betaImg[SC_RG]);
          *iBiasH2 = (betaImg[SC_BY] + betaImg[SC_RG]);
          *iBiasS  = (betaImg[SC_BY] + betaImg[SC_RG]);
          *iBiasV  = (betaImg[SC_BY] + betaImg[SC_RG]);
        }

        if(itsUseConspicMap[SC_IN])
          *iBiasV  += betaImg[SC_IN];
      }

      if(itsUseConspicMap[SC_FL])
        *iBiasV  += betaImg[SC_FL];
      if(itsUseConspicMap[SC_DR0])
        *iBiasV  += betaImg[SC_DR0] + betaImg[SC_DR1] +
                    betaImg[SC_DR2] + betaImg[SC_DR3];
      if(itsUseConspicMap[SC_GA0])
        *iBiasV  += betaImg[SC_GA0] + betaImg[SC_GA1] +
                    betaImg[SC_GA2] + betaImg[SC_GA3];
    }
  }

  if(itsNormalizeBiasWithScale)
  {
    FLOAT miH1, maH1, miH2, maH2, miS, maS, miV, maV;
    FLOAT fmaxH1, fmaxH2, fmaxS, fmaxV;
    getMinMax(itsLocalBiasH1,miH1,maH1);
    getMinMax(itsLocalBiasH2,miH2,maH2);
    getMinMax(itsLocalBiasS, miS, maS);
    getMinMax(itsLocalBiasV, miV, maV);

    // normalize but maintain relative scale between H1,H2,S and V

    if((maH1 > maH2) && (maH1 > maS) && (maH1 > maV))
    {
      const FLOAT adj = 1.0 / maH1;
      fmaxH1 = 1.0;
      fmaxH2 = maH2 * adj;
      fmaxS  = maS  * adj;
      fmaxV  = maV  * adj;
    }
    else if((maH2 > maS) && (maH2 > maV))
    {
      const FLOAT adj = 1.0 / maH2;
      fmaxH2 = 1.0;
      fmaxH1 = maH1 * adj;
      fmaxS  = maS  * adj;
      fmaxV  = maV  * adj;
    }
    else if(maS > maV)
    {
      const FLOAT adj = 1.0 / maS;
      fmaxS  = 1.0;
      fmaxH1 = maH1 * adj;
      fmaxH2 = maH2 * adj;
      fmaxV  = maV  * adj;
    }
    else
    {
      const FLOAT adj = 1.0 / maV;
      fmaxV  = 1.0;
      fmaxH1 = maH1 * adj;
      fmaxH2 = maH2 * adj;
      fmaxS  = maS  * adj;
    }

    for(iBiasH1 = itsLocalBiasH1.beginw(); iBiasH1 != itsLocalBiasH1.endw();
        ++iBiasH1)
    {
      *iBiasH1 = ((*iBiasH1 - miH1) / (maH1 - miH1)) * fmaxH1;
    }

    for(iBiasH2 = itsLocalBiasH2.beginw(); iBiasH2 != itsLocalBiasH2.endw();
        ++iBiasH2)
    {
      *iBiasH2 = ((*iBiasH2 - miH2) / (maH2 - miH2)) * fmaxH2;
    }

    for(iBiasS  = itsLocalBiasS.beginw();  iBiasS  != itsLocalBiasS.endw();
        ++iBiasS)
    {
      *iBiasS  = ((*iBiasS  - miS)  / (maS  - miS))  * fmaxS;
    }

    for(iBiasV  = itsLocalBiasV.beginw();  iBiasV  != itsLocalBiasV.endw();
        ++iBiasV)
    {
      *iBiasV  = ((*iBiasV  - miV)  / (maV  - miV))  * fmaxV;
    }
  }

  // put in the final bias over each channel

  for(iBiasH1 = itsLocalBiasH1.beginw(); iBiasH1 != itsLocalBiasH1.endw();
      ++iBiasH1)
  {
    *iBiasH1 = itsH1Bias * *iBiasH1;
  }

  for(iBiasH2 = itsLocalBiasH2.beginw(); iBiasH2 != itsLocalBiasH2.endw();
      ++iBiasH2)
  {
    *iBiasH2 = itsH2Bias * *iBiasH2;
  }

  for(iBiasS  = itsLocalBiasS.beginw();  iBiasS  != itsLocalBiasS.endw();
      ++iBiasS)
  {
    *iBiasS  = itsSBias * *iBiasS;
  }

  for(iBiasV  = itsLocalBiasV.beginw();  iBiasV  != itsLocalBiasV.endw();
      ++iBiasV)
  {
    *iBiasV  = itsVBias * *iBiasV;
  }

  if(SC_DEBUG)
  {
    char name[100];
    sprintf(name,"debug-biasParts-H1-%d-%d.png",itsScale,itsIterCounter);
    std::string prefix    = name;
    Image<PixRGB<FLOAT> > temp =
      normalizeScaleRainbow(itsLocalBiasH1,0,5);
    Raster::WriteRGB(temp, prefix);

    sprintf(name,"debug-biasParts-H2-%d-%d.png",itsScale,itsIterCounter);
    prefix    = name;
    temp = normalizeScaleRainbow(itsLocalBiasH2,0,5);
    Raster::WriteRGB(temp, prefix);

    sprintf(name,"debug-biasParts-S-%d-%d.png",itsScale,itsIterCounter);
    prefix    = name;
    temp = normalizeScaleRainbow(itsLocalBiasS,0,5);
    Raster::WriteRGB(temp, prefix);

    sprintf(name,"debug-biasParts-V-%d-%d.png",itsScale,itsIterCounter);
    prefix    = name;
    temp = normalizeScaleRainbow(itsLocalBiasV,0,5);
    Raster::WriteRGB(temp, prefix);
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCseperateConvXYZ()
{
  SCseperateConv('z');
  SCseperateConv('y');
  SCseperateConv('x');
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCseperateConvXY()
{
  // skip over the z layer convolution and start at y
  // we need to set the starting point where z would have left off
  itsInterImage[0] = *itsFrameCurrent;
  SCseperateConv('y');
  SCseperateConv('x');
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCseperateConv(const char axis)
{
  const typename std::deque<Image<PIXTYPE > >::const_iterator
    frameBufferBegin = itsFrameBuffer.begin();
  //const typename std::deque<Image<PIXTYPE > >::const_iterator
  //  frameBufferEnd   = itsFrameBuffer.end();

  typename Image<PIXTYPE >::iterator       outImageItr;
  typename Image<PIXTYPE >::iterator       outImageBegin;
  typename Image<PIXTYPE >::iterator       outImageEnd;
  typename Image<PIXTYPE >::const_iterator inImageItr;
  typename Image<PIXTYPE >::const_iterator inImageTrueItr;

  Image<ushort>::const_iterator      itsStartItr;
  Image<ushort>::const_iterator      itsStopItr;
  Image<ushort>::const_iterator      itsKStartItr;

  typename std::deque<Image<PIXTYPE > >::const_iterator frameBufferItr;
  typename std::vector<FLOAT>::const_iterator           kalmanKernelbegin;
  FLOAT axisBias;

  if(axis == 'z')
  {
    //inImageItr        = itsInImage.beginw();
    // Pick the image in the middle of the frame buffer
    // We use floor so that it points to an array index which starts at 0
    inImageItr        = itsFrameCurrent->begin();
    outImageBegin     = itsInterImage[0].beginw();
    outImageEnd       = itsInterImage[0].endw();
    itsStartItr       = itsZStart.begin();
    itsStopItr        = itsZStop.begin();
    itsKStartItr      = itsKZStart.begin();
    kalmanKernelbegin = itsKalmanKernelZ.begin();
    axisBias          = itsZBias;
  }
  else if(axis == 'y')
  {
    // blank the output image
    inImageItr        = itsInterImage[0].begin();
    outImageBegin     = itsInterImage[1].beginw();
    outImageEnd       = itsInterImage[1].endw();
    itsStartItr       = itsYStart.begin();
    itsStopItr        = itsYStop.begin();
    itsKStartItr      = itsKYStart.begin();
    kalmanKernelbegin = itsKalmanKernelY.begin();
    axisBias          = itsYBias;
  }
  else if(axis == 'x')
  {
    inImageItr        = itsInterImage[1].begin();
    outImageBegin     = itsOutImage.beginw();
    outImageEnd       = itsOutImage.endw();
    itsStartItr       = itsXStart.begin();
    itsStopItr        = itsXStop.begin();
    itsKStartItr      = itsKXStart.begin();
    kalmanKernelbegin = itsKalmanKernelX.begin();
    axisBias          = itsXBias;
  }
  else
  {
    LINFO("Must use axis as z,x then y in that order");
    LFATAL("Unknown axis specified %c",axis);
    // put values here to make to compiler shut the %*&! up
    inImageItr        = itsInterImage[1].begin();
    outImageBegin     = itsOutImage.beginw();
    outImageEnd       = itsOutImage.endw();
    itsStartItr       = itsXStart.begin();
    itsStopItr        = itsXStop.begin();
    itsKStartItr      = itsKXStart.begin();
    kalmanKernelbegin = itsKalmanKernelX.begin();
    axisBias          = itsXBias;
  }

  // blank the output image and the norm image
  for(outImageItr = outImageBegin; outImageItr != outImageEnd; ++outImageItr)
  {
    outImageItr->p[0]     = 0.0F;
    outImageItr->p[1]     = 0.0F;
    outImageItr->p[2]     = 0.0F;
    outImageItr->p[3]     = 0.0F;
  }

  typename Image<FLOAT>::const_iterator iBiasH1 = itsLocalBiasH1.begin();
  typename Image<FLOAT>::const_iterator iBiasH2 = itsLocalBiasH2.begin();
  typename Image<FLOAT>::const_iterator iBiasS  = itsLocalBiasS.begin();
  typename Image<FLOAT>::const_iterator iBiasV  = itsLocalBiasV.begin();

  inImageTrueItr = itsFrameCurrent->begin();
  // We have the biases now convolve by the biases

  int      pos = 0;
  ushort Zstop = 0;

  if(axis == 'z')
  {
    if(itsFrameBuffer.size() > itsKernelSizeZ)
      Zstop = itsKernelSizeZ;
    else
      Zstop = itsFrameBuffer.size();
  }

  for(outImageItr  = outImageBegin;
      outImageItr != outImageEnd; pos++,
        ++outImageItr, ++inImageItr,
        ++itsStartItr, ++itsStopItr, ++itsKStartItr, ++inImageTrueItr,
        ++iBiasH1, ++iBiasH2, ++iBiasS, ++iBiasV)
  {

    //LINFO("POS %d",pos);
    FLOAT npixH1 = 0.0F;
    FLOAT npixH2 = 0.0F;
    FLOAT npixS  = 0.0F;
    FLOAT npixV  = 0.0F;

    const FLOAT H1bias = *iBiasH1 * axisBias;
    const FLOAT H2bias = *iBiasH2 * axisBias;
    const FLOAT Sbias  = *iBiasS  * axisBias;
    const FLOAT Vbias  = *iBiasV  * axisBias;

    ushort start,stop;

    if(axis == 'z')
    {
      start = 0;
      stop  = Zstop;
    }
    else
    {
      start = *itsStartItr;
      stop  = *itsStopItr;
    }

    // figure out where we are in the image and the kernel
    frameBufferItr                 = frameBufferBegin;

    int posMod = 0;
    if(axis == 'y')
      posMod = pos%itsImageSizeX;
    else if(axis == 'x')
      posMod = pos/itsImageSizeX;

    typename std::vector<FLOAT>::const_iterator k = kalmanKernelbegin +
                                                      (*itsKStartItr);

    // iterate over the kernel
    for(ushort i = start; i < stop; i++, ++k)
    {
      // get the current other pixel x,y,z
      PIXTYPE curr;
      if(axis == 'z')
      {
        curr = frameBufferItr->getVal(pos);
        frameBufferItr++;
      }
      else if(axis == 'y')
        curr = itsInterImage[0].getVal(posMod,i);
      else
        curr = itsInterImage[1].getVal(i,posMod);

      // if other pixel is to dim, don't use its color
      // this avoids H2SV (and HSV) singularities at black for hue

      if(curr.p[3] > SC_NEAR_BLACK)
      {
        const FLOAT H1      = (*k)      * H1bias * curr.p[2];
        outImageItr->p[0]  += curr.p[0] * H1;
        npixH1             += H1;

        const FLOAT H2      = (*k)      * H2bias * curr.p[2];
        outImageItr->p[1]  += curr.p[1] * H2;
        npixH2             += H2;
      }

      // saturation is biased by color, intensity,
      // orientation and motion
      const FLOAT S       = (*k)       * Sbias;
      outImageItr->p[2]  += curr.p[2]  * S;
      npixS              += S;

      const FLOAT V       = (*k)       * Vbias;
      outImageItr->p[3]  += curr.p[3]  * V;
      npixV              += V;
    }

    // accumulate the normalization

    // at x, the final iteration normalize the final image
    if(axis == 'x')
    {
      // add in the original image to average (in a sense) with the
      // blurred image
      outImageItr->p[0] += inImageTrueItr->p[0] * itsOriginalImageWeight;
      outImageItr->p[1] += inImageTrueItr->p[1] * itsOriginalImageWeight;
      outImageItr->p[2] += inImageTrueItr->p[2] * itsOriginalImageWeight;
      outImageItr->p[3] += inImageTrueItr->p[3] * itsOriginalImageWeight;

      npixH1 += itsOriginalImageWeight;
      npixH2 += itsOriginalImageWeight;
      npixS  += itsOriginalImageWeight;
      npixV  += itsOriginalImageWeight;
    }
    if(npixH1 != 0)
      outImageItr->p[0] = outImageItr->p[0]/npixH1;
    if(npixH2 != 0)
      outImageItr->p[1] = outImageItr->p[1]/npixH2;
    if(npixS != 0)
      outImageItr->p[2] = outImageItr->p[2]/npixS;
    if(npixV != 0)
      outImageItr->p[3] = outImageItr->p[3]/npixV;
  }

  // clamp just in case to at least 0
  if(axis == 'x')
  {
    if(outImageItr->p[0] < 0)
      outImageItr->p[0] = 0;
    if(outImageItr->p[1] < 0)
      outImageItr->p[1] = 0;
    if(outImageItr->p[2] < 0)
      outImageItr->p[2] = 0;
    if(outImageItr->p[3] < 0)
      outImageItr->p[3] = 0;
  }
}


/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
Image<PixRGB<FLOAT> > SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetSharpened(
                                                   const PIXTYPE scale_factor)
  const
{
  if(itsOutputReady)
  {
    const Image<PIXTYPE> fbuffer  = *itsFrameCurrent;

    // get img 1 - img 2 as a difference image
    const Image<PIXTYPE> diffimg  = fbuffer - itsOutImage;

    // add back the original with a scaling factor
    Image<PIXTYPE> finalimg;
    finalimg.resize(fbuffer.getWidth(), fbuffer.getHeight());


    // clamp values just in case
    typename Image<PIXTYPE>::const_iterator dimg = diffimg.begin();
    typename Image<PIXTYPE>::const_iterator fimg = fbuffer.begin();

    typename Image<FLOAT>::const_iterator iBiasH1 = itsLocalBiasH1.begin();
    typename Image<FLOAT>::const_iterator iBiasH2 = itsLocalBiasH2.begin();
    typename Image<FLOAT>::const_iterator iBiasS  = itsLocalBiasS.begin();
    typename Image<FLOAT>::const_iterator iBiasV  = itsLocalBiasV.begin();

    for(typename Image<PIXTYPE>::iterator aptr = finalimg.beginw();
        aptr != finalimg.endw();
        ++aptr, ++dimg, ++fimg , ++iBiasH1, ++iBiasH2, ++iBiasS, ++iBiasV)
    {
      aptr->p[0] = ((dimg->p[0] * scale_factor.p[0] * *iBiasH1) + fimg->p[0]);
      if(aptr->p[0] > 1) aptr->p[0] = 1;
      aptr->p[1] = ((dimg->p[1] * scale_factor.p[1] * *iBiasH2) + fimg->p[1]);
      if(aptr->p[1] > 1) aptr->p[1] = 1;
      aptr->p[2] = ((dimg->p[2] * scale_factor.p[2] * *iBiasS) + fimg->p[2]);
      if(aptr->p[2] > 1) aptr->p[2] = 1;
      aptr->p[3] = ((dimg->p[3] * scale_factor.p[3] * *iBiasV) + fimg->p[3]);
      if(aptr->p[3] > 1) aptr->p[3] = 1;
    }

    return static_cast<Image<PixRGB<FLOAT> > >(finalimg);
  }
  else
  {
    Image<PixRGB<FLOAT> > blank; blank.resize(itsImageSizeX,itsImageSizeY);
    typename Image<PixRGB<FLOAT> >::iterator ib = blank.beginw();

    PixRGB<FLOAT>bp = PixRGB<FLOAT>(0,0,0);

    while(ib != blank.endw()) {*ib = bp; ++ib;}

    return blank;
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
Image<PixRGB<FLOAT> > SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetFrame()
const
{
  if(itsOutputReady)
  {
    return itsFinalImage;
  }
  else
  {
    Image<PixRGB<FLOAT> > blank; blank.resize(itsImageSizeX,itsImageSizeY);
    typename Image<PixRGB<FLOAT> >::iterator ib = blank.beginw();

    PixRGB<FLOAT>bp = PixRGB<FLOAT>(0,0,0);

    while(ib != blank.endw()) {*ib = bp; ++ib;}

    return blank;
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
Image<PixRGB<FLOAT> > SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetOutImage()
const
{
  Image<PixRGB<FLOAT> > returnImage;
  returnImage.resize(itsOutImage.getWidth(),itsOutImage.getHeight());
  typename Image<PixRGB<FLOAT> >::iterator returnImageItr =
    returnImage.beginw();
  typename Image<PIXTYPE>::const_iterator outImageItr =
    itsOutImage.begin();
  while(returnImageItr != returnImage.endw())
  {
    *returnImageItr = PixRGB<FLOAT>(*outImageItr);
    ++returnImageItr; ++outImageItr;
  }

  return returnImage;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
Image<PixRGB<FLOAT> > SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetInImage()
const
{
  if(itsOutputReady)
  {
    Image<PixRGB<FLOAT> > returnImage;
    const Image<PIXTYPE> fbuffer = *itsFrameCurrent;
    returnImage.resize(itsFrameCurrent->getWidth(),
                       itsFrameCurrent->getHeight());
    typename Image<PixRGB<FLOAT> >::iterator returnImageItr =
      returnImage.beginw();
    typename Image<PIXTYPE>::const_iterator inImageItr = fbuffer.begin();

    while(returnImageItr != returnImage.endw())
    {
      *returnImageItr = PixRGB<FLOAT>(*inImageItr);
      ++returnImageItr; ++inImageItr;
    }
    return returnImage;
  }
  else
  {
    Image<PixRGB<FLOAT> > blank; blank.resize(itsImageSizeX,itsImageSizeY);
    typename Image<PixRGB<FLOAT> >::iterator ib = blank.beginw();

    PixRGB<FLOAT>bp = PixRGB<FLOAT>(0,0,0);

    while(ib != blank.endw()) {*ib = bp; ++ib;}

    return blank;
  }
}
/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
Image<PIXTYPE> SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetRawOutImage() const
{
  return itsOutImage;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
Image<PIXTYPE> SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetRawInImage() const
{
  if(itsOutputReady)
  {
    return *itsFrameCurrent;
  }
  else
  {
    Image<PIXTYPE> blank; blank.resize(itsImageSizeX,itsImageSizeY);
    typename Image<PIXTYPE>::iterator ib = blank.beginw();

    PIXTYPE bp = PIXTYPE(0,0,0,0);
    while(ib != blank.endw()) {*ib = bp; ++ib;}

    return blank;
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
Image<BETATYPE> SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetBetaImage() const
{
  if(itsOutputReady)
  {
    return *itsBetaCurrent;
  }
  else
  {
    Image<BETATYPE> blank; blank.resize(itsImageSizeX,itsImageSizeY);
    typename Image<BETATYPE>::iterator ib = blank.beginw();

    PixHyper<FLOAT,SC_MAX_CHANNELS> bp = PixHyper<FLOAT,SC_MAX_CHANNELS>(0);

    while(ib != blank.endw()) {*ib = bp; ++ib;}

    return blank;
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
ushort SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetTemporalOffset() const
{
  return itsTemporalOffset;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
bool SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCisOutputReady() const
{
  return itsOutputReady;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetLocalBiasImages(
                                   Image<FLOAT> &H1, Image<FLOAT> &H2,
                                   Image<FLOAT> &S, Image<FLOAT>  &V) const
{
  if(itsOutputReady)
  {
    H1 = itsLocalBiasH1; H2 = itsLocalBiasH2;
    S  = itsLocalBiasS;  V  = itsLocalBiasV;
  }
  else
  {
    Image<FLOAT> blank;  blank.resize(itsImageSizeX,itsImageSizeY);
    typename Image<FLOAT>::iterator ib = blank.beginw();
    FLOAT bp = 0;

    while(ib != blank.endw()) {*ib = bp; ++ib;}

    H1 = blank; H2 = blank; S = blank; V = blank;
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT> inline
void SurpriseControl<PIXTYPE,BETATYPE,FLOAT>::SCgetSeperableParts(
                                              Image<PIXTYPE> &Zimg,
                                              Image<PIXTYPE> &Yimg) const
{
  if(itsOutputReady)
  {
    Zimg = itsInterImage[0];
    Yimg = itsInterImage[1];
  }
  else
  {
    Image<PIXTYPE> blank; blank.resize(itsImageSizeX,itsImageSizeY);
    typename Image<PIXTYPE>::iterator ib = blank.beginw();

    PIXTYPE bp = PIXTYPE(0,0,0,0);
    while(ib != blank.endw()) {*ib = bp; ++ib;}

    Zimg = blank; Yimg = blank;
  }
}

/*************************************************************************/

template class SurpriseControl<PixH2SV1<float>,
                               PixHyper<float,SC_MAX_CHANNELS>,float>;
template class SurpriseControl<PixH2SV2<float>,
                               PixHyper<float,SC_MAX_CHANNELS>,float>;

#endif
