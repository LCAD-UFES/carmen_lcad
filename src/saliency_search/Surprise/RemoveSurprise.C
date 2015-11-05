/*!@file Surprise/RemoveSurprise.C attempt to remove surprise from image  */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/RemoveSurprise.C $
// $Id: RemoveSurprise.C 14376 2011-01-11 02:44:34Z pez $
//

#ifndef REMOVE_SURPRISE_C_DEFINED
#define REMOVE_SURPRISE_C_DEFINED

#include "Surprise/RemoveSurprise.H"
#include <typeinfo>

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RemoveSurprise(const ushort sizeX,
                                                       const ushort sizeY)
{
  LINFO("Creating RemoveSurprise Object");

  itsINbias = 1.0F; itsDRbias = 1.0F; itsFLbias = 1.0F; itsGAbias = 1.0F;
  itsRGbias = 1.0F; itsBYbias = 1.0F;

  itsCObias = 1.0F;  itsMObias = 1.0F;  itsORbias = 1.0F;  itsINbias = 1.0F;

  RSinit(sizeX,sizeY);
  RSuseTrueKalman(false);
  RSuseMaxLevel(false);

  if((typeid(BETATYPE).name() != typeid(itsHyper4).name()) &&
     (typeid(BETATYPE).name() != typeid(itsHyper6).name()))
  {
    LFATAL("Run time type error. Type: %s is not supported",
           typeid(BETATYPE).name());
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RemoveSurprise()
{
  LINFO("Creating RemoveSurprise Object NOTE: Call init()");

  itsINbias = 1.0F; itsDRbias = 1.0F; itsFLbias = 1.0F; itsGAbias = 1.0F;
  itsRGbias = 1.0F; itsBYbias = 1.0F;

  itsCObias = 1.0F;  itsMObias = 1.0F;  itsORbias = 1.0F;  itsINbias = 1.0F;
  RSuseTrueKalman(false);
  if((typeid(BETATYPE).name() != typeid(itsHyper4).name()) &&
     (typeid(BETATYPE).name() != typeid(itsHyper6).name()))
  {
    LFATAL("Run time type error. Type: %s is not supported",
           typeid(BETATYPE).name());
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::~RemoveSurprise()
{}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSuseTrueKalman(const bool useTK)
{
  LINFO("Setting itsUseTrueKalman");
  itsUseTrueKalman = useTK;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSuseMaxLevel(const bool useML)
{
  LINFO("Setting itsUseMaxLevel");
  itsUseMaxLevel = useML;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSsetConspicBias(const FLOAT CO,
                                                              const FLOAT MO,
                                                              const FLOAT OR,
                                                              const FLOAT IN)
{
  LINFO("Setting new biases with 4 maps");
  if(typeid(BETATYPE).name() != typeid(itsHyper4).name())
  {
    LFATAL("Run time type error, expected %s got %s",
           typeid(itsHyper4).name(),typeid(BETATYPE).name());
  }
  itsCObias = CO; itsMObias = MO; itsORbias = OR; itsINbias = IN;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSsetConspicBias(const FLOAT IN,
                                                              const FLOAT DR,
                                                              const FLOAT FL,
                                                              const FLOAT GA,
                                                              const FLOAT RG,
                                                              const FLOAT BY)
{
  LINFO("Setting new biases with 6 maps");
  if(typeid(BETATYPE).name() != typeid(itsHyper6).name())
  {
    LFATAL("Run time type error, expected %s got %s",
           typeid(itsHyper6).name(), typeid(BETATYPE).name());
  }
  itsINbias = IN; itsDRbias = DR; itsFLbias = FL; itsGAbias = GA;
  itsRGbias = RG; itsBYbias = BY;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSsetAxisBias(const FLOAT X,
                                                           const FLOAT Y,
                                                           const FLOAT Z)
{
  itsXbias = X; itsYbias = Y; itsZbias = Z;
}
/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSsetH2SVBias(const FLOAT H1,
                                                           const FLOAT H2,
                                                           const FLOAT S,
                                                           const FLOAT V)
{
  itsH1bias = H1; itsH2bias = H2; itsSbias = S; itsVbias = V;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinit(const ushort sizeX,
                                                    const ushort sizeY)
{
  LINFO("INIT");
  itsLambda              = 0.1;
  itsInitBuffer          = false;
  itsUseCorrMatrixSet    = false;
  itsUseBayesWeightImage = false;
  itsImageSizeX          = sizeX;
  itsImageSizeY          = sizeY;
  itsXbias = 1.0F; itsYbias = 1.0F; itsZbias = 1.0F;
  itsInImage.resize(sizeX,sizeY);
  itsOutImage.resize(sizeX,sizeY);
  itsFinalImage.resize(sizeX,sizeY);

  itsBetaImage.resize(sizeX,sizeY);
  BETATYPE blank(0.0F);
  PIXTYPE  blank2(0.0F);
  typename Image<BETATYPE>::iterator betaImageItr = itsBetaImage.beginw();
  Image<PIXTYPE> tempImage;
  tempImage.resize(sizeX,sizeY);
  typename Image<PIXTYPE>::iterator tempImageItr = tempImage.beginw();
  while(betaImageItr != itsBetaImage.endw())
  {
    *betaImageItr = blank;
    *tempImageItr = blank2;
    ++betaImageItr; ++tempImageItr;
  }

  // resize intermediate images, set equal to the blank beta image

  itsInterImage.resize(2,tempImage);
  itsInterImageNorm.resize(1,tempImage);

  if(typeid(BETATYPE).name() == typeid(itsHyper4).name())
  {
    itsInit.resize(8,false);
    itsInitMessage.resize(8,"");
    itsInitMessage[0] = "Input Raw Image at method RSinputRawImage";
    itsInitMessage[1] = "Input Sal Map at method RSinputSalMap";
    itsInitMessage[2] = "Input conspicuity map for color at method RSinputConspicCO";
    itsInitMessage[3] = "Input conspicuity map for motion at method RSinputConspicMO";
    itsInitMessage[4] = "Input conspicuity map for orientation at method RSinputConspicOR";
    itsInitMessage[5] = "Input conspicuity map for intensity at method RSinputConspicIN";
    itsInitMessage[6] = "Set up filters at RScreateSepFilters";
    itsInitMessage[7] = "Set up convolution bounds at RSfindConvolutionEndPoints";
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSsetLambda(const FLOAT lambda)
{
  itsLambda = lambda;
}
// 1. create a guassian filtered frame of the current movie
/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSsetDesatBias(const FLOAT desat)
{
  itsDesatbias = desat;
}

// 2. merge the guassian filtered frame with the current frame giving more
//    weight to filtered pixels if the surprise is higher.

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputRawImage(
                                     const Image<PixRGB<FLOAT> >& rawImage,
                                     const uint frame)
{
  LINFO("INPUT RAW IMAGE");
  itsInit[0] = true;
  itsFrameNumber = frame;
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

  // If we wish to use a more proper kalman filter then store the new image
  // else just buffer the raw images
  if((itsUseTrueKalman) && (itsInitBuffer))
  {
    itsFrameBuffer.push_front(itsOutImage);
  }
  else
  {
    itsFrameBuffer.push_front(itsInImage);
    itsInitBuffer = true;
  }

  //if we get to big, pop off the back image
  if(itsFrameBuffer.size() > itsKernelSizeZ)
  {
    itsFrameBuffer.pop_back();
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputSalMap(const Image<FLOAT>& salMap)
{
  LINFO("INPUT Salmap IMAGE");
  itsInit[1] = true;
  if(itsInImage.getWidth() != salMap.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Sal Width %d != Raw Width %d",salMap.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != salMap.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Sal Height %d != Raw Height %d",salMap.getHeight(),
           itsInImage.getHeight());
  }

  // normalize from 0 to 1
  itsSalMap = salMap/255.0F;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicCO(const Image<FLOAT>& conspicCO)
{
  LINFO("INPUT CO IMAGE");
  itsInit[2] = true;
  if(itsInImage.getWidth() != conspicCO.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicCO.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicCO.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicCO.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  itsConspicCO = conspicCO/255.0F;
  itsConspicCO = itsConspicCO*itsCObias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicMO(const Image<FLOAT>& conspicMO)
{
  LINFO("INPUT MO IMAGE");
  itsInit[3] = true;
  if(itsInImage.getWidth() != conspicMO.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicMO.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicMO.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicMO.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  itsConspicMO = conspicMO/255.0F;
  itsConspicMO = itsConspicMO*itsMObias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicOR(const Image<FLOAT>& conspicOR)
{
  LINFO("INPUT OR IMAGE");
  itsInit[4] = true;
  if(itsInImage.getWidth() != conspicOR.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicOR.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicOR.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicOR.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  itsConspicOR = conspicOR/255.0F;
  itsConspicOR = itsConspicOR*itsORbias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicIN(const Image<FLOAT>& conspicIN)
{
  LINFO("INPUT IN IMAGE");
  itsInit[5] = true;
  if(itsInImage.getWidth() != conspicIN.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicIN.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicIN.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicIN.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  //itsConspicIN = conspicIN/255.0F;
  itsConspicIN = conspicIN;
  itsConspicIN = itsConspicIN*itsINbias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicDR(const Image<FLOAT>& conspicDR)
{
  LINFO("INPUT DR IMAGE");

  if(itsInImage.getWidth() != conspicDR.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicDR.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicDR.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicDR.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  //itsConspicDR = conspicDR/255.0F;
  itsConspicDR = conspicDR;
  itsConspicDR = itsConspicDR*itsDRbias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicFL(const Image<FLOAT>& conspicFL)
{
  LINFO("INPUT FL IMAGE");
  itsInit[5] = true;
  if(itsInImage.getWidth() != conspicFL.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicFL.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicFL.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicFL.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  //itsConspicFL = conspicFL/255.0F;
  itsConspicFL = conspicFL;
  itsConspicFL = itsConspicFL*itsFLbias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicGA(const Image<FLOAT>& conspicGA)
{
  LINFO("INPUT GA IMAGE");
  itsInit[5] = true;
  if(itsInImage.getWidth() != conspicGA.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicGA.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicGA.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicGA.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  //itsConspicGA = conspicGA/255.0F;
  itsConspicGA = conspicGA;
  itsConspicGA = itsConspicGA*itsGAbias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicRG(const Image<FLOAT>& conspicRG)
{
  LINFO("INPUT RG IMAGE");
  itsInit[5] = true;
  if(itsInImage.getWidth() != conspicRG.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicRG.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicRG.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicRG.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  //itsConspicRG = conspicRG/255.0F;
  itsConspicRG = conspicRG;
  itsConspicRG = itsConspicRG*itsRGbias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputConspicBY(const Image<FLOAT>& conspicBY)
{
  LINFO("INPUT BY IMAGE");
  itsInit[5] = true;
  if(itsInImage.getWidth() != conspicBY.getWidth())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Width %d != Raw Width %d",conspicBY.getWidth(),
           itsInImage.getWidth());
  }
  if(itsInImage.getHeight() != conspicBY.getHeight())
  {
    LINFO("Saliency map is the wrong size or raw image not initalized");
    LFATAL("Conspic Height %d != Raw Height %d",conspicBY.getHeight(),
           itsInImage.getHeight());
  }
  // normalize from 0 to 1
  //itsConspicBY = conspicBY/255.0F;
  itsConspicBY = conspicBY;
  itsConspicBY = itsConspicBY*itsBYbias;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSinputBayesWeightImage(
                                             const Image<FLOAT> &bayesImage)
{
  LINFO("INPUT Bayes Weight Image");
  itsUseBayesWeightImage = true;
  itsBayesWeightImage = bayesImage;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSsetCorrWeightMat(
                          const std::vector<std::vector<FLOAT> > corAnti,
                          const std::vector<std::vector<FLOAT> > corBase)
{
  LINFO("INPUT Corr Weight Matrix");
  itsUseCorrMatrixSet = true;
  itsAntiCorrelationMat = corAnti;
  itsBaseCorrelationMat = corBase;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RScreateSepFilters(const FLOAT spatSigma,
                                        const FLOAT tempSigma,
                                        const FLOAT stdDevSize)
{
  LINFO("SETTING UP FILTERS");
  itsInit[6] = true;
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

  // the temporal filter is one tailed (we can't smooth into the future)
  rs1 = (uint)ceil((tempSigma*stdDevSize));
  rs2 = (uint)floor((tempSigma*stdDevSize));
  // make sure the kernel is odd sized temporal
  if(rs1%2 == 0)
  {
    if(rs2%2 == 0)
    {
      itsKernelSizeZ = (ushort)rs2 + 1;
    }
    else
    {
      itsKernelSizeZ = (ushort)rs2;
    }
  }
  else
  {
    itsKernelSizeZ = (ushort)rs1;
  }
  itsKalmanKernelZ.resize(itsKernelSizeZ,0.0F);

  // find the kernel center

  const FLOAT centerX = floor((FLOAT)itsKernelSizeX/2.0F);
  const FLOAT centerY = floor((FLOAT)itsKernelSizeY/2.0F);

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

  gmod = 2.0F/sqrt(2.0F*M_PI*pow(spatSigma,2));
  LINFO("Kernel Z size %d",itsKernelSizeZ);
  for(ushort i = 0; i < itsKernelSizeZ; i++)
  {
    const FLOAT dist = pow((FLOAT)i,2.0F);
    itsKalmanKernelZ[i] =
      (gmod*exp((-1.0F*dist)/pow(2*spatSigma,2)));
    std::cerr << i << " : " << itsKalmanKernelZ[i] << "\n";
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSfindConvolutionEndPoints()
{
  LINFO("SETTING UP CONVOLUTION END POINTS");
  itsInit[7] = true;
  itsXStart.resize(itsImageSizeX,itsImageSizeY);
  itsYStart.resize(itsImageSizeX,itsImageSizeY);
  itsZStart.resize(itsImageSizeX,itsImageSizeY);
  itsXStop.resize( itsImageSizeX,itsImageSizeY);
  itsYStop.resize( itsImageSizeX,itsImageSizeY);
  itsZStop.resize( itsImageSizeX,itsImageSizeY);
  itsKXStart.resize(itsImageSizeX,itsImageSizeY);
  itsKYStart.resize(itsImageSizeX,itsImageSizeY);
  itsKZStart.resize(itsImageSizeX,itsImageSizeY);

  Image<ushort>::iterator itsXStartItr  = itsXStart.beginw();
  Image<ushort>::iterator itsXStopItr   = itsXStop.beginw();
  Image<ushort>::iterator itsKXStartItr = itsKXStart.beginw();
  Image<ushort>::iterator itsYStartItr  = itsYStart.beginw();
  Image<ushort>::iterator itsYStopItr   = itsYStop.beginw();
  Image<ushort>::iterator itsKYStartItr = itsKYStart.beginw();
  Image<ushort>::iterator itsZStartItr  = itsZStart.beginw();
  Image<ushort>::iterator itsZStopItr   = itsZStop.beginw();
  Image<ushort>::iterator itsKZStartItr = itsKZStart.beginw();

  // foo

  for(ushort y = 0; y < itsImageSizeY; y++)
  {
    for(ushort x = 0; x < itsImageSizeX; x++)
    {
      const ushort yhalf = (ushort)floor(itsKernelSizeY/2);
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

      const ushort xhalf = (ushort)floor(itsKernelSizeX/2);
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

      // Z is not interesting, its starts at the current frame and
      // runs the size of the deque
      *itsKZStartItr = 0;
      *itsZStartItr  = 0;
      *itsZStopItr   = itsKernelSizeZ;

      //LINFO("X %d - %d K %d",*itsXStartItr,*itsXStopItr,*itsKXStartItr);
      //LINFO("Y %d - %d K %d",*itsYStartItr,*itsYStopItr,*itsKYStartItr);
      ++itsXStartItr; ++itsXStopItr; ++itsKXStartItr;
      ++itsYStartItr; ++itsYStopItr; ++itsKYStartItr;
      ++itsZStartItr; ++itsZStopItr; ++itsKZStartItr;
    }
  }
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSprocessFrame()
{
  LINFO("PROCESSING FRAME %d",itsFrameNumber);
  for(uint i = 0; i < itsInit.size(); i++)
  {
    if(!itsInit[i])
    {
      std::cerr << itsInitMessage[i] << "\n";
      LFATAL("One or more nessesary methods not run");
    }
  }
  if(typeid(BETATYPE).name() != typeid(itsHyper4).name())
  {
    LFATAL("Run time type mismatch. Expected %s got %s",
            typeid(itsHyper4).name(),typeid(BETATYPE).name());
  }


  Image<ushort>::iterator            itsXStartItr  = itsXStart.beginw();
  Image<ushort>::iterator            itsYStartItr  = itsYStart.beginw();
  Image<ushort>::iterator            itsXStopItr   = itsXStop.beginw();
  Image<ushort>::iterator            itsYStopItr   = itsYStop.beginw();
  Image<ushort>::iterator            itsKXStartItr = itsKXStart.beginw();
  Image<ushort>::iterator            itsKYStartItr = itsKYStart.beginw();

  typename Image<PIXTYPE >::iterator outImageItr;
  typename Image<PIXTYPE >::iterator inImageItr    = itsInImage.beginw();
  typename Image<BETATYPE>::iterator betaImageItr  = itsBetaImage.beginw();

  typename Image<FLOAT>::iterator    itsConsCOItr  = itsConspicCO.beginw();
  typename Image<FLOAT>::iterator    itsConsMOItr  = itsConspicMO.beginw();
  typename Image<FLOAT>::iterator    itsConsORItr  = itsConspicOR.beginw();
  typename Image<FLOAT>::iterator    itsConsINItr  = itsConspicIN.beginw();

  typename Image<PixRGB<FLOAT> >::iterator finalImageItr =
    itsFinalImage.beginw();
  typename std::deque<Image<PIXTYPE > >::iterator frameBufferItr;

  const typename std::deque<Image<PIXTYPE > >::iterator frameBufferEnd =
    itsFrameBuffer.end();
  const typename std::deque<Image<PIXTYPE > >::iterator frameBufferBegin =
    itsFrameBuffer.begin();

  const typename std::vector<FLOAT>::iterator kalmanKernelXbegin =
    itsKalmanKernelX.begin();
  const typename std::vector<FLOAT>::iterator kalmanKernelYbegin =
    itsKalmanKernelY.begin();
  const typename std::vector<FLOAT>::iterator kalmanKernelZbegin =
    itsKalmanKernelZ.begin();

  // copy itsInImage to itsOutImage so we can take the conspicuity weighted
  // average of the two

  // for the output image pixels
  for(outImageItr  = itsOutImage.beginw();
      outImageItr != itsOutImage.endw();
      ++outImageItr,   ++finalImageItr,  ++inImageItr, ++betaImageItr,
      ++itsYStartItr,  ++itsXStartItr,
      ++itsYStopItr,   ++itsXStopItr,
      ++itsKYStartItr, ++itsKXStartItr,
      ++itsConsCOItr,  ++itsConsMOItr,
      ++itsConsORItr,  ++itsConsINItr)
  {

    PIXTYPE normal(0.0F);

    // reset this pixel
    *outImageItr         = normal;

    const FLOAT CO       = *itsConsCOItr;
    const FLOAT MO       = *itsConsMOItr;
    const FLOAT OR       = *itsConsORItr;
    const FLOAT IN       = *itsConsINItr;
    const ushort XStart  = *itsXStartItr;
    const ushort YStart  = *itsYStartItr;
    const ushort KXStart = *itsKXStartItr;
    const ushort KYStart = *itsKYStartItr;
    const ushort XStop   = *itsXStopItr;
    const ushort YStop   = *itsYStopItr;

    // copy these variables into a more register friendly data structure

    FLOAT outH1 = inImageItr->H1();
    FLOAT outH2 = inImageItr->H2();
    FLOAT outS  = inImageItr->S();
    FLOAT outV  = inImageItr->V();

    // use normal to hold the final divisor for the convolution

    FLOAT npixS  = normal.p[2]; npixS++;
    FLOAT npixV  = normal.p[3]; npixV++;
    FLOAT npixH1 = normal.p[0]; npixH1++;
    FLOAT npixH2 = normal.p[1]; npixH2++;

    // smooth bias over iterations with a decay term


    const FLOAT betaCO =
      (betaImageItr->p[0] * itsLambda + CO)/(1 + 1 * itsLambda);

    betaImageItr->p[0] = betaCO;

    const FLOAT betaMO =
      (betaImageItr->p[1] * itsLambda + MO)/(1 + 1 * itsLambda);
    betaImageItr->p[1] = betaMO;

    const FLOAT betaOR =
      (betaImageItr->p[2] * itsLambda + OR)/(1 + 1 * itsLambda);
    betaImageItr->p[2] = betaOR;

    const FLOAT betaIN =
      (betaImageItr->p[3] * itsLambda + IN)/(1 + 1 * itsLambda);
    betaImageItr->p[3] = betaIN;


    if((betaCO > SMALL_SALIENCY) || (betaMO > SMALL_SALIENCY) ||
       (betaOR > SMALL_SALIENCY) || (betaIN > SMALL_SALIENCY))
    {
      // iterate over Y
      const typename std::vector<FLOAT>::iterator kalmanKernelXbeginLoc =
        kalmanKernelXbegin + KXStart;



      typename std::vector<FLOAT>::iterator ky = kalmanKernelYbegin + KYStart;

      for(ushort j = YStart; j < YStop; j++, ++ky)
      {
        const FLOAT COky = betaCO * (*ky);
        const FLOAT MOky = betaMO * (*ky);
        const FLOAT ORky = betaOR * (*ky);
        const FLOAT INky = betaIN * (*ky);
        const int   posy = j * itsImageSizeX;
        // iterate over X
        typename std::vector<FLOAT>::iterator kx = kalmanKernelXbeginLoc;
        for(ushort i = XStart; i < XStop; i++, ++kx)
        {
          const FLOAT COkx     = (*kx) * COky;
          const FLOAT MOkx     = (*kx) * MOky;
          const FLOAT ORkx     = (*kx) * ORky;
          const FLOAT INkx     = (*kx) * INky;
          const FLOAT COMO     = COkx  + MOkx;
          const FLOAT COINORMO = COMO  + ORkx + INkx;
          const int   posx     = posy  + i;
          // iterate through each frame in the buffer Z
          typename std::vector<FLOAT>::iterator kz = kalmanKernelZbegin;

          for(frameBufferItr =  frameBufferBegin;
              frameBufferItr != frameBufferEnd;
              ++frameBufferItr, ++kz)
          {
            const PIXTYPE *curr = &frameBufferItr->getVal(posx);

            // Hue is biased by color and motion
            // if intensity is very low, don't bother with color since
            // HSV is singular for hue if color is black
            if(curr->p[3] > 0.15F)
            {
              const FLOAT COMOkz  = (*kz) * COMO;
              outH1  += curr->p[0] * COMOkz;
              outH2  += curr->p[1] * COMOkz;
              npixH1 += COMOkz;
            }

            // saturation is biased by color, intensity,
            // orientation and motion
            const FLOAT COINORMOkz = (*kz) * COINORMO;
            outS  += curr->p[2] * COINORMOkz;
            npixS += COINORMOkz;

            // intensity is biased by color, intensity,
            // orientation and motion
            outV  += curr->p[3] * COINORMOkz;
          }
        }
      }
    }

    outImageItr->p[0] = outH1/npixH1;
    outImageItr->p[1] = outH2/npixH1;
    outImageItr->p[2] = outS /npixS;
    outImageItr->p[3] = outV /npixS;

    *finalImageItr = PixRGB<FLOAT>(*outImageItr);
    // final image is a combination of the current image and
    // the convolved image weighted by surprise
  }
}


/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSprocessFrameSeperable()
{
  LINFO("PROCESSING FRAME %d",itsFrameNumber);
  typename Image<BETATYPE>::iterator betaImageItr;
  typename Image<PixRGB<FLOAT> >::iterator   finalImageItr =
    itsFinalImage.beginw();

  if(typeid(BETATYPE).name() == typeid(itsHyper4).name())
  {
    LINFO("Non-Scale 4 Channel Model");
    typename Image<FLOAT>::iterator    itsConsCOItr  = itsConspicCO.beginw();
    typename Image<FLOAT>::iterator    itsConsMOItr  = itsConspicMO.beginw();
    typename Image<FLOAT>::iterator    itsConsORItr  = itsConspicOR.beginw();
    typename Image<FLOAT>::iterator    itsConsINItr  = itsConspicIN.beginw();

    const PIXTYPE ZERO(0.0F);
    PIXTYPE normal;
    // copy itsInImage to itsOutImage so we can take the conspicuity weighted
    // average of the two
    // Derive the new bias with low pass smoothing
    for(betaImageItr = itsBetaImage.beginw();
        betaImageItr != itsBetaImage.endw();
        ++betaImageItr,
        ++itsConsCOItr,  ++itsConsMOItr,
        ++itsConsORItr,  ++itsConsINItr)
    {
      const FLOAT CO       = *itsConsCOItr;
      const FLOAT MO       = *itsConsMOItr;
      const FLOAT OR       = *itsConsORItr;
      const FLOAT IN       = *itsConsINItr;

      betaImageItr->p[0] =
        (betaImageItr->p[0] * itsLambda + CO)/(1 + 1 * itsLambda);

      betaImageItr->p[1] =
        (betaImageItr->p[1] * itsLambda + MO)/(1 + 1 * itsLambda);

      betaImageItr->p[2] =
        (betaImageItr->p[2] * itsLambda + OR)/(1 + 1 * itsLambda);

      betaImageItr->p[3] =
        (betaImageItr->p[3] * itsLambda + IN)/(1 + 1 * itsLambda);
    }
  }
  else
  {
    LINFO("Scale 6 Channel Model");
    typename Image<FLOAT>::iterator    itsConsINItr  = itsConspicIN.beginw();
    typename Image<FLOAT>::iterator    itsConsDRItr  = itsConspicDR.beginw();
    typename Image<FLOAT>::iterator    itsConsFLItr  = itsConspicFL.beginw();
    typename Image<FLOAT>::iterator    itsConsGAItr  = itsConspicGA.beginw();
    typename Image<FLOAT>::iterator    itsConsRGItr  = itsConspicRG.beginw();
    typename Image<FLOAT>::iterator    itsConsBYItr  = itsConspicBY.beginw();
    typename Image<FLOAT>::iterator    itsBayesWeightItr =
      itsBayesWeightImage.beginw();

    const PIXTYPE ZERO(0.0F);
    PIXTYPE normal;
    // copy itsInImage to itsOutImage so we can take the conspicuity weighted
    // average of the two

    // Derive the new bias with low pass smoothing
    for(betaImageItr = itsBetaImage.beginw();
        betaImageItr != itsBetaImage.endw();
        ++betaImageItr,
        ++itsConsINItr,  ++itsConsDRItr,
        ++itsConsFLItr,  ++itsConsGAItr,
        ++itsConsRGItr,  ++itsConsBYItr,
        ++itsBayesWeightItr)
    {
      if(itsUseBayesWeightImage)
      {
        *itsConsINItr = *itsConsINItr * *itsBayesWeightItr;
        *itsConsDRItr = *itsConsDRItr * *itsBayesWeightItr;
        *itsConsFLItr = *itsConsFLItr * *itsBayesWeightItr;
        *itsConsGAItr = *itsConsGAItr * *itsBayesWeightItr;
        *itsConsRGItr = *itsConsRGItr * *itsBayesWeightItr;
        *itsConsBYItr = *itsConsBYItr * *itsBayesWeightItr;
      }
      if(itsUseCorrMatrixSet)
      {
        const FLOAT IN       = *itsConsINItr;
        const FLOAT DR       = *itsConsDRItr;
        const FLOAT FL       = *itsConsFLItr;
        const FLOAT GA       = *itsConsGAItr;
        const FLOAT RG       = *itsConsRGItr;
        const FLOAT BY       = *itsConsBYItr;

        const FLOAT INnew = pow(RScomputeCor(IN,DR,FL,GA,RG,BY,0),20) * IN;
        //LINFO("IN old %f new %f",IN,INnew);
        betaImageItr->p[0] =
          (betaImageItr->p[0] * itsLambda + INnew)/
          (1 + 1 * itsLambda);

        const FLOAT DRnew = pow(RScomputeCor(IN,DR,FL,GA,RG,BY,1),20) * DR;
        //LINFO("DR old %f new %f",DR,DRnew);
        betaImageItr->p[1] =
          (betaImageItr->p[1] * itsLambda + DRnew)/
          (1 + 1 * itsLambda);

        const FLOAT FLnew = pow(RScomputeCor(IN,DR,FL,GA,RG,BY,2),20) * FL;
        //LINFO("FL old %f new %f",FL,FLnew);
        betaImageItr->p[2] =
          (betaImageItr->p[2] * itsLambda + FLnew)/
          (1 + 1 * itsLambda);

        const FLOAT GAnew = pow(RScomputeCor(IN,DR,FL,GA,RG,BY,3),20) * GA;
        //LINFO("GA old %f new %f",GA,GAnew);
        betaImageItr->p[3] =
          (betaImageItr->p[3] * itsLambda + GAnew)/
          (1 + 1 * itsLambda);

        const FLOAT RGnew = pow(RScomputeCor(IN,DR,FL,GA,RG,BY,4),20) * RG;
        //LINFO("RG old %f new %f",RG,RGnew);
        betaImageItr->p[4] =
          (betaImageItr->p[4] * itsLambda + RGnew)/
          (1 + 1 * itsLambda);

        const FLOAT BYnew = pow(RScomputeCor(IN,DR,FL,GA,RG,BY,5),20) * BY;
        //LINFO("BY old %f new %f",BY,BYnew);
        betaImageItr->p[5] =
          (betaImageItr->p[5] * itsLambda + BYnew)/
          (1 + 1 * itsLambda);
      }
      else
      {
        betaImageItr->p[0] =
          (betaImageItr->p[0] * itsLambda + *itsConsINItr)/
          (1 + 1 * itsLambda);

        betaImageItr->p[1] =
          (betaImageItr->p[1] * itsLambda + *itsConsDRItr)/
          (1 + 1 * itsLambda);

        betaImageItr->p[2] =
          (betaImageItr->p[2] * itsLambda + *itsConsFLItr)/
          (1 + 1 * itsLambda);

        betaImageItr->p[3] =
          (betaImageItr->p[3] * itsLambda + *itsConsGAItr)/
          (1 + 1 * itsLambda);

        betaImageItr->p[4] =
          (betaImageItr->p[4] * itsLambda + *itsConsRGItr)/
          (1 + 1 * itsLambda);

        betaImageItr->p[5] =
          (betaImageItr->p[5] * itsLambda + *itsConsBYItr)/
          (1 + 1 * itsLambda);
      }
    }
  }
  LINFO("BETA done");
  // do x,y and z seperate convolutions
  RSseperateConv('z'); RSseperateConv('y'); RSseperateConv('x');
  LINFO("FINISHING image");
  typename Image<PIXTYPE >::iterator outImageItr    = itsOutImage.beginw();
  for(finalImageItr  = itsFinalImage.beginw();
      finalImageItr != itsFinalImage.endw();
      ++finalImageItr, ++outImageItr)
  {
    *finalImageItr = PixRGB<FLOAT>(*outImageItr);
  }
  LINFO("DONE");
}


/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
void RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSseperateConv(const char axis)
{
  typename Image<BETATYPE>::iterator betaImageItr = itsBetaImage.beginw();

  const typename std::deque<Image<PIXTYPE > >::iterator frameBufferBegin =
    itsFrameBuffer.begin();
  // const typename std::deque<Image<PIXTYPE > >::iterator frameBufferEnd   =
  //  itsFrameBuffer.end();

  typename Image<PIXTYPE >::iterator outImageItr;
  typename Image<PIXTYPE >::iterator outImageBegin;
  typename Image<PIXTYPE >::iterator outImageEnd;
  //typename Image<PIXTYPE >::iterator outImageNormItr;
  typename Image<PIXTYPE >::iterator inImageItr;
  typename Image<PIXTYPE >::iterator inImageTrueItr;

  Image<ushort>::iterator           itsStartItr;
  Image<ushort>::iterator           itsStopItr;
  Image<ushort>::iterator           itsKStartItr;

  typename std::deque<Image<PIXTYPE > >::iterator frameBufferItr;
  typename std::vector<FLOAT>::iterator           kalmanKernelbegin;
  FLOAT axisBias;

  if(axis == 'z')
  {
    inImageItr        = itsInImage.beginw();
    outImageBegin     = itsInterImage[0].beginw();
    outImageEnd       = itsInterImage[0].endw();
    //outImageNormItr   = itsInterImageNorm[0].beginw();
    itsStartItr       = itsZStart.beginw();
    itsStopItr        = itsZStop.beginw();
    itsKStartItr      = itsKZStart.beginw();
    kalmanKernelbegin = itsKalmanKernelZ.begin();
    axisBias          = itsZbias;

    // blank the output image and the norm image
    for(outImageItr  = outImageBegin;
        outImageItr != outImageEnd;
        ++outImageItr)
    {
      outImageItr->p[0]     = 0.0F;
      outImageItr->p[1]     = 0.0F;
      outImageItr->p[2]     = 0.0F;
      outImageItr->p[3]     = 0.0F;
      /*
      outImageNormItr->p[0] = 0.0F;
      outImageNormItr->p[1] = 0.0F;
      outImageNormItr->p[2] = 0.0F;
      outImageNormItr->p[3] = 0.0F;
      */
    }
    //outImageNormItr   = itsInterImageNorm[0].beginw();

  }
  else if(axis == 'y')
  {
    // blank the output image
    inImageItr        = itsInterImage[0].beginw();
    outImageBegin     = itsInterImage[1].beginw();
    outImageEnd       = itsInterImage[1].endw();
    //outImageNormItr   = itsInterImageNorm[0].beginw();
    itsStartItr       = itsYStart.beginw();
    itsStopItr        = itsYStop.beginw();
    itsKStartItr      = itsKYStart.beginw();
    kalmanKernelbegin = itsKalmanKernelY.begin();
    axisBias          = itsYbias;

    // blank the output image
    for(outImageItr = outImageBegin; outImageItr != outImageEnd; ++outImageItr)
    {
      outImageItr->p[0]     = 0.0F;
      outImageItr->p[1]     = 0.0F;
      outImageItr->p[2]     = 0.0F;
      outImageItr->p[3]     = 0.0F;
    }
  }
  else if(axis == 'x')
  {
    inImageItr        = itsInterImage[1].beginw();
    outImageBegin     = itsOutImage.beginw();
    outImageEnd       = itsOutImage.endw();
    //outImageNormItr   = itsInterImageNorm[0].beginw();
    itsStartItr       = itsXStart.beginw();
    itsStopItr        = itsXStop.beginw();
    itsKStartItr      = itsKXStart.beginw();
    kalmanKernelbegin = itsKalmanKernelX.begin();
    axisBias          = itsXbias;

    // blank the output image
    for(outImageItr = outImageBegin; outImageItr != outImageEnd; ++outImageItr)
    {
      outImageItr->p[0]     = 0.0F;
      outImageItr->p[1]     = 0.0F;
      outImageItr->p[2]     = 0.0F;
      outImageItr->p[3]     = 0.0F;
    }
  }
  else
  {
    LINFO("Must use axis as z,x then y in that order");
    LFATAL("Unknown axis specified %c",axis);
    // put values here to make to compiler shut the %*&! up
    inImageItr        = itsInterImage[1].beginw();
    outImageBegin     = itsOutImage.beginw();
    outImageEnd       = itsOutImage.endw();
    //outImageNormItr   = itsInterImageNorm[0].beginw();
    itsStartItr       = itsXStart.beginw();
    itsStopItr        = itsXStop.beginw();
    itsKStartItr      = itsKXStart.beginw();
    kalmanKernelbegin = itsKalmanKernelX.begin();
    axisBias          = itsXbias;
  }

  int pos = 0;
  FLOAT localBiasH1 = 0.0F;
  FLOAT localBiasH2 = 0.0F;
  FLOAT localBiasS  = 0.0F;
  FLOAT localBiasV  = 0.0F;

  const bool hyper6 = (typeid(BETATYPE).name() == typeid(itsHyper6).name());
  inImageTrueItr = itsInImage.beginw();

  for(outImageItr  = outImageBegin;
      outImageItr != outImageEnd; pos++,
        ++outImageItr, ++inImageItr, ++betaImageItr,
        ++itsStartItr, ++itsStopItr, ++itsKStartItr, ++inImageTrueItr)
  {
    FLOAT npixH1 = 0.0F;
    FLOAT npixH2 = 0.0F;
    FLOAT npixS  = 0.0F;
    FLOAT npixV  = 0.0F;

    if(hyper6)
    {
      // get the surprise biases smoothed
      const FLOAT betaIN = betaImageItr->p[0];
      const FLOAT betaDR = betaImageItr->p[1];
      const FLOAT betaFL = betaImageItr->p[2];
      const FLOAT betaGA = betaImageItr->p[3];

      LFATAL("two lines following this message do not compile. FIXME");
      const FLOAT betaRG = 0; ///////betaImageItr->p[4];
      const FLOAT betaBY = 0; ///////betaImageItr->p[5];

      // compute the surprise bias combinations with the axis bias
      /* If we use max level then surprise removal is scaled by the max
         level of all conspicuity maps, this limits reduction to be no more
         than the highest conspicuity map and prevents to much application
         at certian locations. However, it may not scale properly
      */

      if(itsUseMaxLevel)
      {
        if(betaBY > betaRG)
        {
          localBiasH1 = betaBY * axisBias * itsH1bias;
          localBiasH2 = betaBY * axisBias * itsH2bias;
          localBiasS  = betaBY * axisBias * itsSbias;
        }
        else
        {
          localBiasH1 = betaRG * axisBias * itsH1bias;
          localBiasH2 = betaRG * axisBias * itsH2bias;
          localBiasS  = betaRG * axisBias * itsSbias;
        }
        FLOAT newVal = betaBY;
        if(betaRG > newVal) newVal = betaRG;
        if(betaDR > newVal) newVal = betaDR;
        if(betaIN > newVal) newVal = betaIN;
        if(betaFL > newVal) newVal = betaFL;
        if(betaGA > newVal) newVal = betaGA;
        localBiasV = newVal * axisBias * itsVbias;
      }
      else
      {
        localBiasH1 = (betaBY + betaRG) * axisBias * itsH1bias;
        localBiasH2 = (betaBY + betaRG) * axisBias * itsH2bias;
        localBiasS  = (betaBY + betaRG) * axisBias * itsSbias;
        localBiasV  = (betaDR + betaBY + betaRG +
                       betaIN + betaFL + betaGA) * axisBias *
                      itsVbias;
      }

      const FLOAT colorBias = (betaRG * betaBY) / (itsBYbias * itsRGbias);
      FLOAT desat           = 1.0F - (colorBias * itsDesatbias);

      //make sure desat stays normal positive
      if(desat < 0.0F)
        desat = 0.0F;

      // Make sure that the values here are not sooo small as to not matter
      if((betaIN > SMALL_SALIENCY) || (betaDR > SMALL_SALIENCY) ||
         (betaFL > SMALL_SALIENCY) || (betaGA > SMALL_SALIENCY) ||
         (betaBY > SMALL_SALIENCY) || (betaRG > SMALL_SALIENCY))
      {
        // figure out where we are in the image and the kernel
        frameBufferItr                 = frameBufferBegin;
        typename std::vector<FLOAT>::iterator k = kalmanKernelbegin +
                                                 (*itsKStartItr);
        int posMod = 0;
        if(axis == 'y')
          posMod = pos%itsImageSizeX;
        else if(axis == 'x')
          posMod = pos/itsImageSizeX;
        else if(axis == 'z')
          *itsStopItr = itsFrameBuffer.size();
        // iterate over the kernel
        for(ushort i = *itsStartItr; i < *itsStopItr; i++, ++k)
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
          if(curr.p[3] > NEAR_BLACK)
          {
            const FLOAT H1      = (*k)      * localBiasH1;
            outImageItr->p[0]  += curr.p[0] * H1;
            npixH1             += H1;

            const FLOAT H2      = (*k)      * localBiasH2;
            outImageItr->p[1]  += curr.p[1] * H2;
            npixH2             += H2;
          }

          // saturation is biased by color, intensity,
          // orientation and motion
          const FLOAT S       = (*k)       * localBiasS;
          outImageItr->p[2]  += (curr.p[2] * desat) * S;
          npixS              += S;

          const FLOAT V       = (*k)       * localBiasV;
          // simple desaturation can increase intensity so we compensate
          // V' = (V - V*S)/(1 - S')
          const FLOAT newV    = (curr.p[3] - curr.p[3]*curr.p[2])/
                                (1 - (curr.p[2] * desat));
            //outImageItr->p[3]  += curr.p[3]  * V;
          outImageItr->p[3]  += newV * V;
          npixV              += V;
        }
      }
    }
    else
    {
      // get the surprise biases smoothed
      const FLOAT betaCO = betaImageItr->p[0];
      const FLOAT betaMO = betaImageItr->p[1];
      const FLOAT betaOR = betaImageItr->p[2];
      const FLOAT betaIN = betaImageItr->p[3];

      // compute the surprise bias combinations with the axis bias
      const FLOAT COMO     = (betaCO + betaMO) * axisBias;
      const FLOAT COINORMO = (betaCO + betaIN + betaOR + betaMO) * axisBias;

      // Make sure that the values here are not sooo small as to not matter
      if((betaCO > SMALL_SALIENCY) || (betaMO > SMALL_SALIENCY) ||
         (betaOR > SMALL_SALIENCY) || (betaIN > SMALL_SALIENCY))
      {
        // figure out where we are in the image and the kernel
        frameBufferItr                 = frameBufferBegin;
        typename std::vector<FLOAT>::iterator k = kalmanKernelbegin +
                                                 (*itsKStartItr);
        int posMod = 0;
        if(axis == 'y')
          posMod = pos%itsImageSizeX;
        else if(axis == 'x')
          posMod = pos/itsImageSizeX;
        else if(axis == 'z')
          *itsStopItr = itsFrameBuffer.size();

        // iterate over the kernel
        for(ushort i = *itsStartItr; i < *itsStopItr; i++, ++k)
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
          if(curr.p[3] > NEAR_BLACK)
          {
            const FLOAT COMOk  = (*k) * COMO;
            outImageItr->p[0]  += curr.p[0] * COMOk;
            outImageItr->p[1]  += curr.p[1] * COMOk;
            npixH1 += COMOk;
          }

          // saturation is biased by color, intensity,
          // orientation and motion
          const FLOAT COINORMOk = (*k) * COINORMO;
          outImageItr->p[2]  += curr.p[2] * COINORMOk;
          outImageItr->p[3]  += curr.p[3] * COINORMOk;
          npixS += COINORMOk;
        }
      }
    }
    // accumulate the normalization

    /*
    outImageNormItr->p[0] += npixH1;
    outImageNormItr->p[1] += npixH1;
    outImageNormItr->p[2] += npixS;
    outImageNormItr->p[3] += npixS;
    */

    // at x, the final iteration normalize the final image
    if(axis == 'x')
    {
      // add in the original image to average (in a sense) with the
      // blurred image
      outImageItr->p[0]     += inImageTrueItr->p[0];
      outImageItr->p[1]     += inImageTrueItr->p[1];
      outImageItr->p[2]     += inImageTrueItr->p[2];
      outImageItr->p[3]     += inImageTrueItr->p[3];

      npixH1++; npixH2++; npixS++; npixV++;

      /*
      outImageNormItr->p[0] += 1.0F;
      outImageNormItr->p[1] += 1.0F;
      outImageNormItr->p[2] += 1.0F;
      outImageNormItr->p[3] += 1.0F;
      */
    }


    /*
    outImageItr->p[0] = outImageItr->p[0]/(outImageNormItr->p[0]);
    outImageItr->p[1] = outImageItr->p[1]/(outImageNormItr->p[1]);
    outImageItr->p[2] = outImageItr->p[2]/(outImageNormItr->p[2]);
    outImageItr->p[3] = outImageItr->p[3]/(outImageNormItr->p[3]);

    outImageNormItr->p[0] = 0.0F;
    outImageNormItr->p[1] = 0.0F;
    outImageNormItr->p[2] = 0.0F;
    outImageNormItr->p[3] = 0.0F;
    */

    outImageItr->p[0] = outImageItr->p[0]/npixH1;
    outImageItr->p[1] = outImageItr->p[1]/npixH2;
    outImageItr->p[2] = outImageItr->p[2]/npixS;
    outImageItr->p[3] = outImageItr->p[3]/npixV;

    // We should never increase saturation
    if(axis == 'x')
      if(outImageItr->p[2] > inImageTrueItr->p[2])
        outImageItr->p[2] = inImageTrueItr->p[2];


    // balance intensity by the change in saturation
    /*
    if(axis == 'x')
    {
      // V' = (V - V*S)/(1 - S')
      outImageItr->p[3] = fabs((outImageItr->p[3] -
                                outImageItr->p[3]*inImageTrueItr->p[2])/
                               (1 - outImageItr->p[2]));
    }
    */
  }

  //LINFO("B");
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
FLOAT RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RScomputeCor(const FLOAT IN,
                                                           const FLOAT DR,
                                                           const FLOAT FL,
                                                           const FLOAT GA,
                                                           const FLOAT RG,
                                                           const FLOAT BY,
                                                           const short idx) const
{
  const FLOAT C1 = IN*itsAntiCorrelationMat[idx][0] +
                   DR*itsAntiCorrelationMat[idx][1] +
                   FL*itsAntiCorrelationMat[idx][2] +
                   GA*itsAntiCorrelationMat[idx][3] +
                   RG*itsAntiCorrelationMat[idx][4] +
                   BY*itsAntiCorrelationMat[idx][5];

  const FLOAT C2 = IN*itsBaseCorrelationMat[idx][0] +
                   DR*itsBaseCorrelationMat[idx][1] +
                   FL*itsBaseCorrelationMat[idx][2] +
                   GA*itsBaseCorrelationMat[idx][3] +
                   RG*itsBaseCorrelationMat[idx][4] +
                   BY*itsBaseCorrelationMat[idx][5];
  /*
  LINFO("BASE %f %f %f %f %f %f",
        IN*itsBaseCorrelationMat[idx][0],
        DR*itsBaseCorrelationMat[idx][1],
        FL*itsBaseCorrelationMat[idx][2],
        GA*itsBaseCorrelationMat[idx][3],
        RG*itsBaseCorrelationMat[idx][4],
        BY*itsBaseCorrelationMat[idx][5]
        );

  LINFO("ANTI %f %f %f %f %f %f",
        IN*itsAntiCorrelationMat[idx][0],
        DR*itsAntiCorrelationMat[idx][1],
        FL*itsAntiCorrelationMat[idx][2],
        GA*itsAntiCorrelationMat[idx][3],
        RG*itsAntiCorrelationMat[idx][4],
        BY*itsAntiCorrelationMat[idx][5]
        );

  LINFO("PARTS %f %f %f %f %f %f",
        itsBaseCorrelationMat[idx][0],
        itsBaseCorrelationMat[idx][1],
        itsBaseCorrelationMat[idx][2],
        itsBaseCorrelationMat[idx][3],
        itsBaseCorrelationMat[idx][4],
        itsBaseCorrelationMat[idx][5]);

  LINFO("BETA %f %f %f %f %f %f",IN,DR,FL,GA,RG,BY);
  */
  if(C1 > 0)
    return C2/C1;
  else
    return 0;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
Image<PixRGB<FLOAT> > RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSgetFrame()
{
  itsInit[0] = false;
  itsInit[1] = false;
  itsInit[2] = false;
  itsInit[3] = false;
  itsInit[4] = false;
  itsInit[5] = false;
  return itsFinalImage;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
Image<PixRGB<FLOAT> > RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSgetOutImage()
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

template <class PIXTYPE, class BETATYPE, class FLOAT>
Image<PIXTYPE> RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSgetRawOutImage() const
{
  return itsOutImage;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
Image<PIXTYPE> RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSgetRawInImage() const
{
  return itsInImage;
}

/*************************************************************************/

template <class PIXTYPE, class BETATYPE, class FLOAT>
Image<BETATYPE> RemoveSurprise<PIXTYPE,BETATYPE,FLOAT>::RSgetBetaImage() const
{
  return itsBetaImage;
}

/*************************************************************************/

template class RemoveSurprise<PixH2SV1<float>,PixHyper<float,4>,float>;
template class RemoveSurprise<PixH2SV2<float>,PixHyper<float,4>,float>;
template class RemoveSurprise<PixH2SV1<float>,PixHyper<float,6>,float>;
template class RemoveSurprise<PixH2SV2<float>,PixHyper<float,6>,float>;

#endif
