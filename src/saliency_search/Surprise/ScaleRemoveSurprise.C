/*!@file Surprise/ScaleRemoveSurprise.C attempt to remove surprise from image */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/ScaleRemoveSurprise.C $
// $Id: ScaleRemoveSurprise.C 6795 2006-06-29 20:45:32Z rjpeters $
//

#ifndef SCALE_REMOVE_SURPRISE_C_DEFINED
#define SCALE_REMOVE_SURPRISE_C_DEFINED

#include "Surprise/ScaleRemoveSurprise.H"
#include "Image/ColorOps.H"

#define SURPRISE_CONF_PATH "/lab/mundhenk/saliency/etc/"
#define SURPRISE_CONF_FILE "removeSurprise.conf"

template <class FLOAT>
ScaleRemoveSurprise<FLOAT>::ScaleRemoveSurprise(const ushort sizeX,
                                                const ushort sizeY,
                                                const string confFile)
{
  const string nc = "null";
  string file;
  // if(strcmp(confFile.c_str(),nc.c_str()))
  //  file = SURPRISE_CONF_FILE;
  // else
    file = confFile;

  const string path     = SURPRISE_CONF_PATH;

  char pathFile[100];
  //sprintf(pathFile,"%s%s",path.c_str(),file.c_str());
  sprintf(pathFile,"%s",file.c_str());
  itsReadConfig.openFile(pathFile);

  // get basic information and biases
  itsAxisBiasX   = itsReadConfig.getItemValueF("itsAxisBiasX");
  itsAxisBiasY   = itsReadConfig.getItemValueF("itsAxisBiasY");
  itsAxisBiasZ   = itsReadConfig.getItemValueF("itsAxisBiasZ");
  itsINBias      = itsReadConfig.getItemValueF("itsINbias");
  itsDRBias      = itsReadConfig.getItemValueF("itsDRbias");
  itsFLBias      = itsReadConfig.getItemValueF("itsFLbias");
  itsGABias      = itsReadConfig.getItemValueF("itsGAbias");
  itsRGBias      = itsReadConfig.getItemValueF("itsRGbias");
  itsBYBias      = itsReadConfig.getItemValueF("itsBYbias");
  itsLambda      = itsReadConfig.getItemValueF("itsLambda");
  itsStdSize     = itsReadConfig.getItemValueF("itsStdSize");
  itsZSigma      = itsReadConfig.getItemValueF("itsZSigma");
  itsUseKalman   = itsReadConfig.getItemValueB("itsUseKalman");
  itsUseMaxLevel = itsReadConfig.getItemValueB("itsUseMaxLevel");
  itsH1bias      = itsReadConfig.getItemValueF("itsH1bias");
  itsH2bias      = itsReadConfig.getItemValueF("itsH2bias");
  itsSbias       = itsReadConfig.getItemValueF("itsSbias");
  itsVbias       = itsReadConfig.getItemValueF("itsVbias");

  itsBaseFilterSize = (ushort)itsReadConfig.getItemValueF("itsBaseFilterSize");

  // get LevelSpec information
  itsLevMin    = (ushort)itsReadConfig.getItemValueF("itsLevMin");
  itsLevMax    = (ushort)itsReadConfig.getItemValueF("itsLevMax");
  itsDelMin    = (ushort)itsReadConfig.getItemValueF("itsDelMin");
  itsDelMax    = (ushort)itsReadConfig.getItemValueF("itsDelMax");
  itsMapLevel  = (ushort)itsReadConfig.getItemValueF("itsMapLevel");
  itsMaxIndex  = (ushort)itsReadConfig.getItemValueF("itsMaxIndex");
  itsMaxDepth  = (ushort)itsReadConfig.getItemValueF("itsMaxDepth");

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

  itsDesatBias.resize(itsMaxIndex,0.0F);
  baseScale = "itsDesatBias_";

  // read in each scale power
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    char temp[100];
    sprintf(temp,"%s%d",baseScale.c_str(),i);
    itsDesatBias[i] = itsReadConfig.getItemValueF(static_cast<string>(temp));
  }

  SRSinit(sizeX,sizeY);
}

/*************************************************************************/

template <class FLOAT>
ScaleRemoveSurprise<FLOAT>::~ScaleRemoveSurprise()
{}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRSinit(const ushort sizeX,
                                         const ushort sizeY)
{
  LINFO("INIT");
  itsImageSizeX = sizeX;
  itsImageSizeY = sizeY;

  // create a RemoveSurprise for each scale
  LINFO("Creating temp object");
  RemoveSurprise<PIX_H2SV_TYPE<FLOAT>,PixHyper<FLOAT,6>,FLOAT> tempRS(sizeX,sizeY);
  itsRemoveSurprise.resize(itsMaxIndex,tempRS);
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
      bx = bx/2; by = by/2; fz = fz*2;
      indx++;
    }
  }

  // set up each RemoveSurprise
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    LINFO("Setting up RemoveSaliency for scale %d",i);
    itsRemoveSurprise[i].RSsetAxisBias(itsAxisBiasX,
                                       itsAxisBiasY,
                                       itsAxisBiasZ);
    // we try to fit something close to the orignal filters used
    const FLOAT sigma   = itsFilterSizesX[i]/4;
    const FLOAT z_sigma = itsFilterSizesZ[i];
    LINFO("%d Desat Bias %f",i,itsDesatBias[i]);
    itsRemoveSurprise[i].RSsetDesatBias(itsDesatBias[i]);
    LINFO("%d Sigma %f Z Sigma %f StdDev %f",i,sigma,z_sigma,itsStdSize);
    itsRemoveSurprise[i].RScreateSepFilters(sigma,z_sigma,itsStdSize);
    itsRemoveSurprise[i].RSfindConvolutionEndPoints();
    LINFO("%d IN Bias %f DR Bias %f FL Bias %f",i,itsINBias,itsDRBias,
          itsFLBias);
    LINFO("%d GA Bias %f RG Bias %f BY Bias %f",i,itsGABias,itsRGBias,
          itsBYBias);
    LINFO("%d Scale Power %f",i,itsScalePower[i]);
    itsRemoveSurprise[i].RSsetConspicBias(itsINBias * itsScalePower[i],
                                          itsDRBias * itsScalePower[i],
                                          itsFLBias * itsScalePower[i],
                                          itsGABias * itsScalePower[i],
                                          itsRGBias * itsScalePower[i],
                                          itsBYBias * itsScalePower[i]);
    LINFO("%d Bias H1 %f H2 %f S %f V %f",i,
          itsH1bias,itsH2bias,itsSbias,itsVbias);
    itsRemoveSurprise[i].RSsetH2SVBias(itsH1bias,itsH2bias,itsSbias,itsVbias);
    itsRemoveSurprise[i].RSuseTrueKalman(false);
    itsRemoveSurprise[i].RSuseMaxLevel(itsUseMaxLevel);
    LINFO("%d Lambda %f",i,itsLambda);
    itsRemoveSurprise[i].RSsetLambda(itsLambda);
  }
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRSinputRawImage(
                                     const Image<PixRGB<FLOAT> >& rawImage,
                                     const uint frame)
{
  itsRawImage = rawImage;
  itsFrame    = frame;
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRSinputSalMap(const Image<FLOAT>& salMap)
{
  itsSalMap = salMap;
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRSsetAntiWeights()
{
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    char fileName[100];
    FLOAT mean      = 0;
    int   N         = 0;

    FLOAT mean2     = 0;
    int   N2        = 0;

    sprintf(fileName,"../ANTI/IntensitySurprise.000000.%d.SCSimage.float.pfz",i);
    string fnameS = fileName;
    const Image<FLOAT> INimage = Raster::ReadFloat(fnameS);

    for(int ii = 0; ii < INimage.getWidth(); ii++)
    {
      for(int j = 0; j < INimage.getHeight(); j++)
      {
        mean += INimage.getVal(ii,j);
        N++;
      }
    }
    FLOAT INmean      = (mean/N);
    sprintf(fileName,"../BASE/IntensitySurprise.000000.%d.SCSimage.float.pfz",i);
    fnameS = fileName;
    const Image<FLOAT> INimage2 = Raster::ReadFloat(fnameS);
    for(int ii = 0; ii < INimage2.getWidth(); ii++)
    {
      for(int j = 0; j < INimage2.getHeight(); j++)
      {
        mean2 += INimage2.getVal(ii,j);
        N2++;
      }
    }
    FLOAT INmean2      = (mean2/N2);

    mean = 0;
    N    = 0;
    mean2 = 0;
    N2    = 0;
    sprintf(fileName,"../ANTI/GaborSurprise.000000.%d.SCSimage.float.pfz",i);
    fnameS = fileName;
    const Image<FLOAT> GAimage = Raster::ReadFloat(fnameS);

    for(int ii = 0; ii < GAimage.getWidth(); ii++)
    {
      for(int j = 0; j < GAimage.getHeight(); j++)
      {
        mean += GAimage.getVal(ii,j);
        N++;
      }
    }
    FLOAT GAmean      = (mean/N);
    sprintf(fileName,"../BASE/GaborSurprise.000000.%d.SCSimage.float.pfz",i);
    fnameS = fileName;
    const Image<FLOAT> GAimage2 = Raster::ReadFloat(fnameS);

    for(int ii = 0; ii < GAimage2.getWidth(); ii++)
    {
      for(int j = 0; j < GAimage2.getHeight(); j++)
      {
        mean2 += GAimage2.getVal(ii,j);
        N2++;
      }
    }
    FLOAT GAmean2      = (mean2/N2);

    mean = 0;
    N    = 0;
    mean2 = 0;
    N2    = 0;
    sprintf(fileName,"../ANTI/RedGreenSurprise.000000.%d.SCSimage.float.pfz",i);
    fnameS = fileName;
    const Image<FLOAT> RGimage = Raster::ReadFloat(fnameS);

    for(int ii = 0; ii < RGimage.getWidth(); ii++)
    {
      for(int j = 0; j < RGimage.getHeight(); j++)
      {
        mean += RGimage.getVal(ii,j);
        N++;
      }
    }
    FLOAT RGmean      = (mean/N);
    sprintf(fileName,"../BASE/RedGreenSurprise.000000.%d.SCSimage.float.pfz",i);
    fnameS = fileName;
    const Image<FLOAT> RGimage2 = Raster::ReadFloat(fnameS);

    for(int ii = 0; ii < RGimage2.getWidth(); ii++)
    {
      for(int j = 0; j < RGimage2.getHeight(); j++)
      {
        mean2 += RGimage2.getVal(ii,j);
        N2++;
      }
    }
    FLOAT RGmean2      = (mean2/N2);

    mean = 0;
    N    = 0;
    mean2 = 0;
    N2    = 0;
    sprintf(fileName,"../ANTI/BlueYellowSurprise.000000.%d.SCSimage.float.pfz",i);
    fnameS = fileName;
    const Image<FLOAT> BYimage = Raster::ReadFloat(fnameS);

    for(int ii = 0; ii < BYimage.getWidth(); ii++)
    {
      for(int j = 0; j < BYimage.getHeight(); j++)
      {
        mean += BYimage.getVal(ii,j);
        N++;
      }
    }
    FLOAT BYmean      = (mean/N);
    sprintf(fileName,"../BASE/BlueYellowSurprise.000000.%d.SCSimage.float.pfz",i);
    fnameS = fileName;
    const Image<FLOAT> BYimage2 = Raster::ReadFloat(fnameS);

    for(int ii = 0; ii < BYimage2.getWidth(); ii++)
    {
      for(int j = 0; j < BYimage2.getHeight(); j++)
      {
        mean2 += BYimage2.getVal(ii,j);
        N2++;
      }
    }
    FLOAT BYmean2      = (mean2/N2);

    FLOAT norm  = INmean  + GAmean  + RGmean  + BYmean;
    FLOAT norm2 = INmean2 + GAmean2 + RGmean2 + BYmean2;

    INmean = (INmean/norm) * 4; GAmean = (GAmean/norm) * 4;
    RGmean = (RGmean/norm) * 4; RGmean = (BYmean/norm) * 4;

    INmean2 = (INmean2/norm2) * 4; GAmean2 = (GAmean2/norm2) * 4;
    RGmean2 = (RGmean2/norm2) * 4; BYmean2 = (BYmean2/norm2) * 4;

    LINFO("%d Old RG val %f",i,itsRGBias);
    LINFO("Base Mean %f Anti Mean %f",RGmean2,RGmean);
    const FLOAT itsNewRGBias = itsRGBias * (RGmean2/RGmean);
    LINFO("%d New RG val %f",i,itsNewRGBias);

    LINFO("%d Old BY val %f",i,itsBYBias);
    LINFO("Base Mean %f Anti Mean %f",BYmean2,BYmean);
    const FLOAT itsNewBYBias = itsBYBias * (BYmean2/BYmean);
    LINFO("%d New BY val %f",i,itsNewBYBias);

    LINFO("%d Old GA val %f",i,itsGABias);
    LINFO("Base Mean %f Anti Mean %f",GAmean2,GAmean);
    const FLOAT itsNewGABias = itsGABias * (GAmean2/GAmean);
    LINFO("%d New GA val %f",i,itsNewGABias);

    LINFO("%d Old IN val %f",i,itsINBias);
    LINFO("Base Mean %f Anti Mean %f",INmean2,INmean);
    const FLOAT itsNewINBias = itsINBias * (INmean2/INmean);
    LINFO("%d New IN val %f",i,itsNewINBias);

    const FLOAT itsNewDRBias = itsDRBias;
    const FLOAT itsNewFLBias = itsFLBias;

    itsRemoveSurprise[i].RSsetConspicBias(itsNewINBias * itsScalePower[i],
                                          itsNewDRBias * itsScalePower[i],
                                          itsNewFLBias * itsScalePower[i],
                                          itsNewGABias * itsScalePower[i],
                                          itsNewRGBias * itsScalePower[i],
                                          itsNewBYBias * itsScalePower[i]);
  }
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRSsetAntiWeightsInteract(const uint aframes,
                                                           const uint bframes)
{
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    char fileName[100];
    std::vector<bool> active(6,true);
    std::vector<FLOAT> blankVec(6,0.0);
    std::vector<FLOAT> MEAN(6,1.0);
    std::vector<FLOAT> MEAN2(6,1.0);
    std::vector<FLOAT> STD(6,1.0);
    std::vector<FLOAT> STD2(6,1.0);
    std::vector<std::vector<FLOAT> > COR(6,blankVec);
    std::vector<std::vector<FLOAT> > COR2(6,blankVec);

    Image<FLOAT> blank;
    std::vector<Image<FLOAT> > INimage(aframes,blank);
    std::vector<Image<FLOAT> > GAimage(aframes,blank);
    std::vector<Image<FLOAT> > RGimage(aframes,blank);
    std::vector<Image<FLOAT> > BYimage(aframes,blank);

    std::vector<Image<FLOAT> > INimage2(bframes,blank);
    std::vector<Image<FLOAT> > GAimage2(bframes,blank);
    std::vector<Image<FLOAT> > RGimage2(bframes,blank);
    std::vector<Image<FLOAT> > BYimage2(bframes,blank);

    FLOAT N    = 0, N2    = 0;
    FLOAT Norm = 0, Norm2 = 0;
    FLOAT INmean  = 0, GAmean  = 0, RGmean  = 0, BYmean  = 0;
    FLOAT INmean2 = 0, GAmean2 = 0, RGmean2 = 0, BYmean2 = 0;
    FLOAT INss    = 0, GAss    = 0, RGss    = 0, BYss    = 0;
    FLOAT INss2   = 0, GAss2   = 0, RGss2   = 0, BYss2   = 0;

    for(uint t = 0; t < aframes; t++)
    {
      int frameNumber = (int)t;
      char c[100];
      if(frameNumber < 10)
        sprintf(c,"00000%d",frameNumber);
      else if(frameNumber < 100)
        sprintf(c,"0000%d",frameNumber);
      else if(frameNumber < 1000)
        sprintf(c,"000%d",frameNumber);
      else if(frameNumber < 10000)
        sprintf(c,"00%d",frameNumber);
      else if(frameNumber < 100000)
        sprintf(c,"0%d",frameNumber);
      else
        sprintf(c,"%d",frameNumber);
      sprintf(fileName,"../ANTI/IntensitySurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      string fnameS = fileName;
      INimage[t] = Raster::ReadFloat(fnameS);

      sprintf(fileName,"../ANTI/GaborSurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      fnameS = fileName;
      GAimage[t] = Raster::ReadFloat(fnameS);

      sprintf(fileName,"../ANTI/RedGreenSurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      fnameS = fileName;
      RGimage[t] = Raster::ReadFloat(fnameS);


      sprintf(fileName,"../ANTI/BlueYellowSurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      fnameS = fileName;
      BYimage[t] = Raster::ReadFloat(fnameS);

    }

    for(uint t = 0; t < bframes; t++)
    {
      int frameNumber = (int)t;
      char c[100];
      if(frameNumber < 10)
        sprintf(c,"00000%d",frameNumber);
      else if(frameNumber < 100)
        sprintf(c,"0000%d",frameNumber);
      else if(frameNumber < 1000)
        sprintf(c,"000%d",frameNumber);
      else if(frameNumber < 10000)
        sprintf(c,"00%d",frameNumber);
      else if(frameNumber < 100000)
        sprintf(c,"0%d",frameNumber);
      else
        sprintf(c,"%d",frameNumber);

      sprintf(fileName,"../BASE/IntensitySurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      string fnameS = fileName;
      INimage2[t] = Raster::ReadFloat(fnameS);

      sprintf(fileName,"../BASE/GaborSurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      fnameS = fileName;
      GAimage2[t] = Raster::ReadFloat(fnameS);

      sprintf(fileName,"../BASE/RedGreenSurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      fnameS = fileName;
      RGimage2[t] = Raster::ReadFloat(fnameS);

      sprintf(fileName,"../BASE/BlueYellowSurprise.%s.%d.SCSimage.float.pfz"
              ,c,i);
      fnameS = fileName;
      BYimage2[t] = Raster::ReadFloat(fnameS);
    }

    active[1] = false; active[2] = false;

    for(uint t = 0; t < bframes; t++)
    {
      for(int ii = 0; ii < BYimage2[t].getWidth(); ii++)
      {
        for(int jj = 0; jj < BYimage2[t].getHeight(); jj++)
        {
          INmean2 += INimage2[t].getVal(ii,jj);
          INss2   += pow(INimage2[t].getVal(ii,jj),2);

          GAmean2 += GAimage2[t].getVal(ii,jj);
          GAss2   += pow(GAimage2[t].getVal(ii,jj),2);

          RGmean2 += RGimage2[t].getVal(ii,jj);
          RGss2   += pow(RGimage2[t].getVal(ii,jj),2);

          BYmean2 += BYimage2[t].getVal(ii,jj);
          BYss2   += pow(BYimage2[t].getVal(ii,jj),2);
          N2++;
        }
      }
    }
    INmean2 = INmean2/N2;  GAmean2 = GAmean2/N2;
    RGmean2 = RGmean2/N2;  BYmean2 = BYmean2/N2;
    MEAN2[0] = INmean2; MEAN2[3] = GAmean2;
    MEAN2[4] = RGmean2; MEAN2[5] = BYmean2;
    STD2[0] = sqrt(INss2/N2 - pow(INmean2,2));
    STD2[3] = sqrt(GAss2/N2 - pow(GAmean2,2));
    STD2[4] = sqrt(RGss2/N2 - pow(RGmean2,2));
    STD2[5] = sqrt(BYss2/N2 - pow(BYmean2,2));

    for(uint t = 0; t < aframes; t++)
    {
      for(int ii = 0; ii < BYimage[t].getWidth(); ii++)
      {
        for(int jj = 0; jj < BYimage[t].getHeight(); jj++)
        {
          INmean += INimage[t].getVal(ii,jj);
          INss   += pow(INimage[t].getVal(ii,jj),2);

          GAmean += GAimage[t].getVal(ii,jj);
          GAss   += pow(GAimage[t].getVal(ii,jj),2);

          RGmean += RGimage[t].getVal(ii,jj);
          RGss   += pow(RGimage[t].getVal(ii,jj),2);

          BYmean += BYimage[t].getVal(ii,jj);
          BYss   += pow(BYimage[t].getVal(ii,jj),2);
          N++;
        }
      }
    }

    INmean = INmean/N;  GAmean = GAmean/N;
    RGmean = RGmean/N;  BYmean = BYmean/N;
    MEAN[0] = INmean; MEAN[3] = GAmean;
    MEAN[4] = RGmean; MEAN[5] = BYmean;
    STD[0] = sqrt(INss/N - pow(INmean,2));
    STD[3] = sqrt(GAss/N - pow(GAmean,2));
    STD[4] = sqrt(RGss/N - pow(RGmean,2));
    STD[5] = sqrt(BYss/N - pow(BYmean,2));

    for(uint t = 0; t < bframes; t++)
    {
      for(int ii = 0; ii < BYimage2[t].getWidth(); ii++)
      {
        for(int jj = 0; jj < BYimage2[t].getHeight(); jj++)
        {
          COR2[0][0] += pow(INimage2[t].getVal(ii,jj),2);
          COR2[0][1] = 0;
          COR2[0][2] = 0;
          COR2[0][3] += INimage2[t].getVal(ii,jj)*GAimage2[t].getVal(ii,jj);
          COR2[0][4] += INimage2[t].getVal(ii,jj)*RGimage2[t].getVal(ii,jj);
          COR2[0][5] += INimage2[t].getVal(ii,jj)*BYimage2[t].getVal(ii,jj);

          COR2[1][0] = 0;
          COR2[1][1] = 0;
          COR2[1][2] = 0;
          COR2[1][3] = 0;
          COR2[1][4] = 0;
          COR2[1][5] = 0;

          COR2[2][0] = 0;
          COR2[2][1] = 0;
          COR2[2][2] = 0;
          COR2[2][3] = 0;
          COR2[2][4] = 0;
          COR2[2][5] = 0;

          COR2[3][0] += GAimage2[t].getVal(ii,jj)*INimage2[t].getVal(ii,jj);
          COR2[3][1] = 0;
          COR2[3][2] = 0;
          COR2[3][3] += pow(GAimage2[t].getVal(ii,jj),2);
          COR2[3][4] += GAimage2[t].getVal(ii,jj)*RGimage2[t].getVal(ii,jj);
          COR2[3][5] += GAimage2[t].getVal(ii,jj)*BYimage2[t].getVal(ii,jj);

          COR2[4][0] += RGimage2[t].getVal(ii,jj)*INimage2[t].getVal(ii,jj);
          COR2[4][1] = 0;
          COR2[4][2] = 0;
          COR2[4][3] += RGimage2[t].getVal(ii,jj)*GAimage2[t].getVal(ii,jj);
          COR2[4][4] += pow(RGimage2[t].getVal(ii,jj),2);
          COR2[4][5] += RGimage2[t].getVal(ii,jj)*BYimage2[t].getVal(ii,jj);

          COR2[5][0] += BYimage2[t].getVal(ii,jj)*INimage2[t].getVal(ii,jj);
          COR2[5][1] = 0;
          COR2[5][2] = 0;
          COR2[5][3] += BYimage2[t].getVal(ii,jj)*GAimage2[t].getVal(ii,jj);
          COR2[5][4] += BYimage2[t].getVal(ii,jj)*RGimage2[t].getVal(ii,jj);
          COR2[5][5] += pow(BYimage2[t].getVal(ii,jj),2);
        }
      }
    }
    for(uint t = 0; t < aframes; t++)
    {
      for(int ii = 0; ii < BYimage[t].getWidth(); ii++)
      {
        for(int jj = 0; jj < BYimage[t].getHeight(); jj++)
        {
          COR[0][0] += pow(INimage[t].getVal(ii,jj),2);
          COR[0][1] = 0;
          COR[0][2] = 0;
          COR[0][3] += INimage[t].getVal(ii,jj)*GAimage[t].getVal(ii,jj);
          COR[0][4] += INimage[t].getVal(ii,jj)*RGimage[t].getVal(ii,jj);
          COR[0][5] += INimage[t].getVal(ii,jj)*BYimage[t].getVal(ii,jj);

          COR[1][0] = 0;
          COR[1][1] = 0;
          COR[1][2] = 0;
          COR[1][3] = 0;
          COR[1][4] = 0;
          COR[1][5] = 0;

          COR[2][0] = 0;
          COR[2][1] = 0;
          COR[2][2] = 0;
          COR[2][3] = 0;
          COR[2][4] = 0;
          COR[2][5] = 0;

          COR[3][0] += GAimage[t].getVal(ii,jj)*INimage[t].getVal(ii,jj);
          COR[3][1] = 0;
          COR[3][2] = 0;
          COR[3][3] += pow(GAimage[t].getVal(ii,jj),2);
          COR[3][4] += GAimage[t].getVal(ii,jj)*RGimage[t].getVal(ii,jj);
          COR[3][5] += GAimage[t].getVal(ii,jj)*BYimage[t].getVal(ii,jj);

          COR[4][0] += RGimage[t].getVal(ii,jj)*INimage[t].getVal(ii,jj);
          COR[4][1] = 0;
          COR[4][2] = 0;
          COR[4][3] += RGimage[t].getVal(ii,jj)*GAimage[t].getVal(ii,jj);
          COR[4][4] += pow(RGimage[t].getVal(ii,jj),2);
          COR[4][5] += RGimage[t].getVal(ii,jj)*BYimage[t].getVal(ii,jj);

          COR[5][0] += BYimage[t].getVal(ii,jj)*INimage[t].getVal(ii,jj);
          COR[5][1] = 0;
          COR[5][2] = 0;
          COR[5][3] += BYimage[t].getVal(ii,jj)*GAimage[t].getVal(ii,jj);
          COR[5][4] += BYimage[t].getVal(ii,jj)*RGimage[t].getVal(ii,jj);
          COR[5][5] += pow(BYimage[t].getVal(ii,jj),2);
        }
      }
    }

    for(int ii = 0; ii < 6; ii++)
    {
      if(active[ii])
      {
        for(int jj = 0; jj < 6; jj++)
        {
          if(active[jj])
          {
            //COR[ii][jj]  = sqrt(COR[ii][jj]/N)/(STD[ii]*STD[jj]);
            //COR[ii][jj]  = (COR[ii][jj]/N)/(STD[ii]*STD[jj]);
            //COR[ii][jj]  = COR[ii][jj]/N;
            //COR[ii][jj]  = pow(sqrt(COR[ii][jj]/N)/(STD[ii]*STD[jj]),4);
            COR[ii][jj] = (((COR[ii][jj]/N) - (MEAN[ii]*MEAN[jj]))/
                          (STD[ii]*STD[jj])) * MEAN[ii];
            Norm  += fabs(COR[ii][jj]);

            //COR2[ii][jj] = sqrt(COR2[ii][jj]/N2)/(STD2[ii]*STD2[jj]);
            //COR2[ii][jj] = (COR2[ii][jj]/N2)/(STD2[ii]*STD2[jj]);
            //COR2[ii][jj] = COR2[ii][jj]/N2;
            //COR2[ii][jj] = pow(sqrt(COR2[ii][jj]/N2)/(STD2[ii]*STD2[jj]),4);
            COR2[ii][jj] = (((COR2[ii][jj]/N2) - (MEAN2[ii]*MEAN2[jj]))/
                           (STD2[ii]*STD2[jj])) * MEAN2[ii];
            Norm2 += fabs(COR2[ii][jj]);
          }
        }
      }
    }

    FLOAT max  = 0;
    FLOAT min  = 10;
    FLOAT max2 = 0;
    FLOAT min2 = 10;


    for(int ii = 0; ii < 6; ii++)
    {
      if(active[ii])
      {
        for(int jj = 0; jj < 6; jj++)
        {
          if(active[jj])
          {
            COR[ii][jj]  = COR[ii][jj]/Norm;
            //COR[ii][jj] += 1;
            if(COR[ii][jj] > max)
              max = COR[ii][jj];
            if(COR[ii][jj] < min)
              min = COR[ii][jj];

            COR2[ii][jj] = COR2[ii][jj]/Norm2;
            //COR2[ii][jj] += 1;
            if(COR2[ii][jj] > max2)
              max2 = COR2[ii][jj];
            if(COR2[ii][jj] < min2)
              min2 = COR2[ii][jj];
          }
        }
      }
    }

    // normalize to range between 0 and 1
    // This stretched the correlation matrix artificially
    /*
    for(int ii = 0; ii < 6; ii++)
    {
      if(active[ii])
      {
        for(int jj = 0; jj < 6; jj++)
        {
          if(active[jj])
          {
              COR[ii][jj]  = (COR[ii][jj]  - min)  * (1/(max - min));
              COR2[ii][jj] = (COR2[ii][jj] - min2) * (1/(max2 - min2));
          }
        }
      }
    }
    */
    COR2[1][1] = 1;
    COR2[2][2] = 1;
    COR[1][1]  = 1;
    COR[2][2]  = 1;

    LINFO("Correlation Base Biasing Matrix");
    for(int ii = 0; ii < 6; ii++)
    {
      for(int jj = 0; jj < 6; jj++)
      {
        std::cerr << COR2[ii][jj] << "\t";
      }
      std::cerr << "\n";
    }

    LINFO("Correlation Anti Biasing Matrix");
    for(int ii = 0; ii < 6; ii++)
    {
      for(int jj = 0; jj < 6; jj++)
      {
        std::cerr << COR[ii][jj] << "\t";
      }
      std::cerr << "\n";
    }

    itsRemoveSurprise[i].RSsetCorrWeightMat(COR,COR2);
  }
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRScomputeBayesFeatureBias(
                                           const uint frames,
                                           const string baseFileNamePrefix,
                                           const string antiFileNamePrefix)
{
  const uint maps        = itsLevMax + itsDelMax;
  const uint features    = 7;
  const uint featureDims = maps * features;

  Image<FLOAT> baseImage;
  std::vector<Image<FLOAT> > baseVec(featureDims,baseImage);
  std::vector<std::vector<Image<FLOAT> > > imageSetBase(frames,baseVec);
  std::vector<std::vector<Image<FLOAT> > > imageSetAnti(frames,baseVec);

  string dash  = "-";
  string dot   = ".";
  string ftype = ".pnm";
  string ANTI  = "../ANTI/";
  string BASE  = "../BASE/";
  string basePrefix = BASE + baseFileNamePrefix;
  string antiPrefix = ANTI + antiFileNamePrefix;

  for(uint i = 0; i < frames; i++)
  {
    uint fnum = 0;
    // for file opening ...
    char frameChar[100];
    const uint itsFrame = i;

    if(itsFrame < 10)
      sprintf(frameChar,"00000%d",itsFrame);
    else if(itsFrame < 100)
      sprintf(frameChar,"0000%d",itsFrame);
    else if(itsFrame < 1000)
      sprintf(frameChar,"000%d",itsFrame);
    else if(itsFrame < 10000)
      sprintf(frameChar,"00%d",itsFrame);
    else if(itsFrame < 100000)
      sprintf(frameChar,"0%d",itsFrame);
    else
      sprintf(frameChar,"%d",itsFrame);

    string feature = "SRby";
    string fenameB = basePrefix + dash + feature + dash;
    string fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }
    /*
    feature = "SRdir_0";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRdir_1";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRdir_2";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRdir_3";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRflicker";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }
    */
    feature = "SRintensity";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRori_0";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRori_1";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRori_2";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRori_3";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }

    feature = "SRrg";
    fenameB  = basePrefix + dash + feature + dash;
    fenameA = antiPrefix + dash + feature + dash;
    for(uint j = 0; j < maps; j++)
    {
      char cmap[10]; sprintf(cmap,"%d",j);
      string file = fenameB + cmap + dash + frameChar + ftype;
      imageSetBase[i][fnum] = Raster::ReadGray(file);
      file        = fenameA + cmap + dash + frameChar + ftype;
      imageSetAnti[i][fnum] = Raster::ReadGray(file);
      fnum++;
    }
  }

  string base = "base";
  string anti = "anti";

  LINFO("RUNNING: corrEigenMatrix");
  corrEigenMatrix(imageSetBase,itsBaseCorr,itsBaseMean,itsBaseSTD,itsBaseSS,
                  itsBaseN,false);

  LINFO("****************************************************");
  LINFO("BaseCorr");
  // look at output if wanted
  for(uint i = 0; i < (uint)itsBaseCorr.getWidth(); i++)
  {
    for(uint j = 0; j < (uint)itsBaseCorr.getHeight(); j++)
    {
      std::cerr << itsBaseCorr.getVal(i,j) << "\t";
    }
    std::cerr << "\n";
  }

  itsBaseR = getPearsonRMatrix(itsBaseCorr,itsBaseSTD);

  for(uint i = 0; i < (uint)itsBaseR.getWidth(); i++)
  {
    for(uint j = 0; j < (uint)itsBaseR.getHeight(); j++)
    {
      std::cerr << itsBaseR.getVal(i,j) << "\t";
    }
    std::cerr << "\n";
  }

  LINFO("****************************************************");
  LINFO("BaseMean");
  for(uint i = 0; i < (uint)itsBaseMean.getWidth(); i++)
  {
    for(uint j = 0; j < (uint)itsBaseMean.getHeight(); j++)
    {
      std::cerr << itsBaseMean.getVal(i,j) << "\t";
    }
    std::cerr << "\n";
  }

  LINFO("****************************************************");
  LINFO("BaseSTD");
  for(uint i = 0; i < (uint)itsBaseSTD.getWidth(); i++)
  {
    for(uint j = 0; j < (uint)itsBaseSTD.getHeight(); j++)
    {
      std::cerr << itsBaseSTD.getVal(i,j) << "\t";
    }
    std::cerr << "\n";
  }


  LINFO("****************************************************");
  LINFO("BaseSS");
  for(uint i = 0; i < (uint)itsBaseSS.getWidth(); i++)
  {
    for(uint j = 0; j < (uint)itsBaseSS.getHeight(); j++)
    {
      std::cerr << itsBaseSS.getVal(i,j) << "\t";
    }
    std::cerr << "\n";
  }

  string type        = "corr.pfz";
  string filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  Raster::WriteFloat(itsBaseCorr,FLOAT_NORM_PRESERVE,filenamePFZ);
  type        = "mean.pfz";
  filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  Raster::WriteFloat(itsBaseMean,FLOAT_NORM_PRESERVE,filenamePFZ);
  type        = "STD.pfz";
  filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  Raster::WriteFloat(itsBaseSTD,FLOAT_NORM_PRESERVE,filenamePFZ);
  type        = "SS.pfz";
  filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  Raster::WriteFloat(itsBaseSTD,FLOAT_NORM_PRESERVE,filenamePFZ);

  type        = "R.png";
  filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  Image<FLOAT> normBaseR = normalizeFloat(itsBaseR,FLOAT_NORM_0_255);
  Image<byte>  byteBaseR = normBaseR;
  Raster::WriteGray(byteBaseR,filenamePFZ);

  corrEigenMatrix(imageSetAnti,itsAntiCorr,itsAntiMean,itsAntiSTD,itsAntiSS,
                  itsAntiN,false);

  // look at output if wanted
  for(uint i = 0; i < (uint)itsAntiCorr.getWidth(); i++)
  {
    for(uint j = 0; j < (uint)itsAntiCorr.getHeight(); j++)
    {
      std::cerr << itsAntiCorr.getVal(i,j) << "\t";
    }
    std::cerr << "\n";
  }

  itsAntiR = getPearsonRMatrix(itsAntiCorr,itsAntiSTD);

  for(uint i = 0; i < (uint)itsAntiR.getWidth(); i++)
  {
    for(uint j = 0; j < (uint)itsAntiR.getHeight(); j++)
    {
      std::cerr << itsAntiR.getVal(i,j) << "\t";
    }
    std::cerr << "\n";
  }

  type        = "corr.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  Raster::WriteFloat(itsAntiCorr,FLOAT_NORM_PRESERVE,filenamePFZ);
  type        = "mean.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  Raster::WriteFloat(itsAntiMean,FLOAT_NORM_PRESERVE,filenamePFZ);
  type        = "STD.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  Raster::WriteFloat(itsAntiSTD,FLOAT_NORM_PRESERVE,filenamePFZ);
  type        = "SS.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  Raster::WriteFloat(itsAntiSS,FLOAT_NORM_PRESERVE,filenamePFZ);

  type        = "R.png";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  Image<FLOAT> normAntiR = normalizeFloat(itsAntiR,FLOAT_NORM_0_255);
  Image<byte>  byteAntiR = normAntiR;
  Raster::WriteGray(byteAntiR,filenamePFZ);

  Image<FLOAT> DiffR;
  DiffR.resize(itsAntiR.getWidth(),itsAntiR.getHeight());

  typename Image<FLOAT>::iterator iAntiR = itsAntiR.beginw();
  typename Image<FLOAT>::iterator iBaseR = itsBaseR.beginw();
  typename Image<FLOAT>::iterator iDiffR = DiffR.beginw();

  while(iAntiR != itsAntiR.endw())
  {
    *iDiffR = *iAntiR - *iBaseR;
    ++iDiffR; ++iAntiR; ++iBaseR;
  }

  Image<PixRGB<FLOAT> > RGDiffR;
  RGDiffR.resize(itsAntiR.getWidth(),itsAntiR.getHeight());
  RGDiffR = normalizeRGPolarAuto(DiffR);

  type = "diffR.png";
  filenamePFZ = baseFileNamePrefix + dot + type;
  Raster::WriteRGB(RGDiffR,filenamePFZ);
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRSopenBayesFeatureBias(
                                           const string baseFileNamePrefix,
                                           const string antiFileNamePrefix)
{
  string dash        = "-";
  string dot         = ".";
  string base        = "base";
  string anti        = "anti";
  string type        = "corr.pfz";
  string filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  itsBaseCorr = Raster::ReadFloat(filenamePFZ);
  type        = "mean.pfz";
  filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  itsBaseMean = Raster::ReadFloat(filenamePFZ);
  type        = "STD.pfz";
  filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  itsBaseSTD  = Raster::ReadFloat(filenamePFZ);
  type        = "SS.pfz";
  filenamePFZ = baseFileNamePrefix + dot + base + dot + type;
  itsBaseSS   = Raster::ReadFloat(filenamePFZ);

  itsBaseR = getPearsonRMatrix(itsBaseCorr,itsBaseSTD);

  type        = "corr.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  itsAntiCorr = Raster::ReadFloat(filenamePFZ);
  type        = "mean.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  itsAntiMean = Raster::ReadFloat(filenamePFZ);
  type        = "STD.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  itsAntiSTD  = Raster::ReadFloat(filenamePFZ);
  type        = "SS.pfz";
  filenamePFZ = antiFileNamePrefix + dot + anti + dot + type;
  itsAntiSS   = Raster::ReadFloat(filenamePFZ);

  itsAntiR = getPearsonRMatrix(itsAntiCorr,itsAntiSTD);

  Image<FLOAT> DiffR;
  DiffR.resize(itsAntiR.getWidth(),itsAntiR.getHeight());

  typename Image<FLOAT>::iterator iAntiR = itsAntiR.beginw();
  typename Image<FLOAT>::iterator iBaseR = itsBaseR.beginw();
  typename Image<FLOAT>::iterator iDiffR = DiffR.beginw();

  while(iAntiR != itsAntiR.endw())
  {
    *iDiffR = *iAntiR - *iBaseR;
    ++iDiffR; ++iAntiR; ++iBaseR;
  }

  Image<PixRGB<FLOAT> > RGDiffR;
  RGDiffR.resize(itsAntiR.getWidth(),itsAntiR.getHeight());
  RGDiffR = normalizeRGPolarAuto(DiffR);

  type = "diffR.png";
  filenamePFZ = baseFileNamePrefix + dot + type;
  Raster::WriteRGB(RGDiffR,filenamePFZ);
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRScomputeBayesFeatureCurrent(
                                     const uint frame,
                                     const string fileNamePrefix)
{
  const uint maps        = itsLevMax + itsDelMax;
  const uint features    = 7;
  const uint featureDims = maps * features;

  Image<FLOAT> baseImage;
  std::vector<Image<FLOAT> > imageSetBase(featureDims,baseImage);

  string dot   = ".";
  string dash  = "-";
  string ftype = ".pnm";
  string basePrefix = fileNamePrefix;

  uint fnum = 0;
  // for file opening ...
  char frameChar[100];
  const uint itsFrame = frame;

  if(itsFrame < 10)
    sprintf(frameChar,"00000%d",itsFrame);
  else if(itsFrame < 100)
    sprintf(frameChar,"0000%d",itsFrame);
  else if(itsFrame < 1000)
    sprintf(frameChar,"000%d",itsFrame);
  else if(itsFrame < 10000)
    sprintf(frameChar,"00%d",itsFrame);
  else if(itsFrame < 100000)
    sprintf(frameChar,"0%d",itsFrame);
  else
    sprintf(frameChar,"%d",itsFrame);

  string feature = "SRby";
  string fenameB = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }
  /*
  feature = "SRdir_0";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRdir_1";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRdir_2";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRdir_3";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRflicker";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }
  */
  feature = "SRintensity";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRori_0";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRori_1";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRori_2";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRori_3";
  fenameB  = basePrefix + dash + feature + dash;
  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  feature = "SRrg";
  fenameB  = basePrefix + dash + feature + dash;

  for(uint j = 0; j < maps; j++)
  {
    char cmap[10]; sprintf(cmap,"%d",j);
    string file = fenameB + cmap + dash + frameChar + ftype;
    imageSetBase[fnum] = Raster::ReadGray(file);
    fnum++;
  }

  getLikelyhoodImage(imageSetBase,itsBaseCorr,itsBaseMean,true,
                     itsBaseLikelyhood,itsNonNormalizedBaseL);

  string post        = "base.likelyhood.png";
  string file        = basePrefix + dot + frameChar + dot + post;
  Image<FLOAT> nbase = normalizeFloat(itsBaseLikelyhood,FLOAT_NORM_0_255);
  Image<byte> bnbase = nbase;
  Raster::WriteGray(bnbase,file);

  getLikelyhoodImage(imageSetBase,itsAntiCorr,itsAntiMean,true,
                     itsAntiLikelyhood,itsNonNormalizedAntiL);

  post   = "anti.likelyhood.png";
  file   = basePrefix + dot + frameChar + dot + post;
  nbase  = normalizeFloat(itsAntiLikelyhood,FLOAT_NORM_0_255);
  bnbase = nbase;
  Raster::WriteGray(bnbase,file);

  FLOAT one  = 1.0;
  FLOAT two  = 2.0;
  FLOAT beta = 0.075;

  itsBayesImage     = getNormalizedBayesImage(itsBaseLikelyhood,
                                              itsAntiLikelyhood,true,beta,
                                              one,one,two);

  post   = "bayes.likelyhood.png";
  file   = basePrefix  + dot + frameChar + dot + post;
  nbase  = normalizeFloat(itsBayesImage,FLOAT_NORM_0_255);
  bnbase = nbase;
  Raster::WriteGray(bnbase,file);

  Image<FLOAT> beliefValues;
  getAugmentedBeliefBayesImage(itsBayesImage,itsNonNormalizedBaseL,
                               itsNonNormalizedAntiL,one,itsBeliefImage,
                               beliefValues);

  post   = "belief.likelyhood.png";
  file   = basePrefix  + dot + frameChar + dot + post;
  nbase  = normalizeFloat(itsBeliefImage,FLOAT_NORM_0_255);
  bnbase = nbase;
  Raster::WriteGray(bnbase,file);

  post   = "belief.likelyhood.NN.png";
  file   = basePrefix  + dot + frameChar + dot + post;
  for(int i = 0; i < itsBeliefImage.getWidth(); i++)
  {
    for(int j = 0; j < itsBeliefImage.getHeight(); j++)
    {
      //std::cerr << itsBeliefImage.getVal(i,j) << "\n";
      nbase.setVal(i,j,itsBeliefImage.getVal(i,j) * 128.0);
    }
  }
  bnbase = nbase;
  Raster::WriteGray(bnbase,file);

  post   = "belief.values.png";
  file   = basePrefix  + dot + frameChar + dot + post;
  nbase  = normalizeFloat(beliefValues,FLOAT_NORM_0_255);
  bnbase = nbase;
  Raster::WriteGray(bnbase,file);

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    itsRemoveSurprise[i].RSinputBayesWeightImage(itsBeliefImage);
  }
}

/*************************************************************************/

template <class FLOAT>
void ScaleRemoveSurprise<FLOAT>::SRSprocessFrame()
{
  // for file opening ...
  char frameChar[100];

  if(itsFrame < 10)
    sprintf(frameChar,"00000%d",itsFrame);
  else if(itsFrame < 100)
    sprintf(frameChar,"0000%d",itsFrame);
  else if(itsFrame < 1000)
    sprintf(frameChar,"000%d",itsFrame);
  else if(itsFrame < 10000)
    sprintf(frameChar,"00%d",itsFrame);
  else if(itsFrame < 100000)
    sprintf(frameChar,"0%d",itsFrame);
  else
    sprintf(frameChar,"%d",itsFrame);


  // process over each scale type from the image pyramid

  //typename Image<PixRGB<FLOAT> >::iterator temp;

  // std::vector<typename Image<PixRGB<FLOAT> >::iterator>
  //  imageItrs(itsMaxIndex,temp);

  for(ushort i = 0; i < itsMaxIndex; i++)
  {

    //char foo[100];
    // sprintf(foo,"%d",i);
    //string scale = foo;
    //string temp  = "temp";
    //string rtemp = "raw";
    //string png   = "png";
    //string dot   = ".";
    //string frame = frameChar;

    //string rname  = rtemp + dot + scale + dot + frame + dot + png;

    //Raster::VisuRGB(itsRawImage,rname);

    char fileName[100];

    itsRemoveSurprise[i].RSinputRawImage(itsRawImage,itsFrame);

    itsRemoveSurprise[i].RSinputSalMap(itsSalMap);

    sprintf(fileName,"IntensitySurprise.%s.%d.SCSimage.float.pfz",frameChar,i);
    string fnameS = fileName;
    const Image<FLOAT> INimage = Raster::ReadFloat(fnameS);
    itsRemoveSurprise[i].RSinputConspicIN(INimage);

    sprintf(fileName,"DirectionSurprise.%s.%d.SCSimage.float.pfz",frameChar,i);
    fnameS = fileName;
    const Image<FLOAT> DRimage = Raster::ReadFloat(fnameS);
    itsRemoveSurprise[i].RSinputConspicDR(DRimage);

    sprintf(fileName,"FlickerSurprise.%s.%d.SCSimage.float.pfz",frameChar,i);
    fnameS = fileName;
    const Image<FLOAT> FLimage = Raster::ReadFloat(fnameS);
    itsRemoveSurprise[i].RSinputConspicFL(FLimage);

    sprintf(fileName,"GaborSurprise.%s.%d.SCSimage.float.pfz",frameChar,i);
    fnameS = fileName;
    const Image<FLOAT> GAimage = Raster::ReadFloat(fnameS);
    itsRemoveSurprise[i].RSinputConspicGA(GAimage);

    sprintf(fileName,"RedGreenSurprise.%s.%d.SCSimage.float.pfz",frameChar,i);
    fnameS = fileName;
    const Image<FLOAT> RGimage = Raster::ReadFloat(fnameS);
    itsRemoveSurprise[i].RSinputConspicRG(RGimage);

    sprintf(fileName,"BlueYellowSurprise.%s.%d.SCSimage.float.pfz",frameChar,i);
    fnameS = fileName;
    const Image<FLOAT> BYimage = Raster::ReadFloat(fnameS);
    itsRemoveSurprise[i].RSinputConspicBY(BYimage);

    // process this scale
    itsRemoveSurprise[i].RSprocessFrameSeperable();

    // get result image for this scale
    itsResultImages[i] = itsRemoveSurprise[i].RSgetFrame();
    //LINFO("Setting iterator");
    //imageItrs[i]       = itsResultImages[i].beginw();
    LINFO("done");

    //string name  = temp + dot + scale + dot + frame + dot + png;
    //Raster::VisuRGB(itsResultImages[i],name);
  }

  FLOAT norm = 0.0F;



  // first form a biased sum over each scale
  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    typename Image<PixRGB<FLOAT> >::iterator finalImageItr =
      itsFinalImage.beginw();
    typename Image<PixRGB<FLOAT> >::iterator resultImageItr =
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
}

/*************************************************************************/

template <class FLOAT>
Image<PixRGB<FLOAT> > ScaleRemoveSurprise<FLOAT>::SRSgetFrame() const
{
  return itsFinalImage;
}

/*************************************************************************/

template <class FLOAT>
Image<PixRGB<FLOAT> > ScaleRemoveSurprise<FLOAT>::SRSgetDiffImage() const
{
  Image<PixRGB<float> > diffImage;
  diffImage.resize(itsImageSizeX,itsImageSizeY);

  typename Image<PixRGB<float> >::const_iterator finalImageItr =
    itsFinalImage.begin();
  typename Image<PixRGB<float> >::const_iterator rawImageItr  =
    itsRawImage.begin();
  typename Image<PixRGB<float> >::iterator diffImageItr =
    diffImage.beginw();

  while(rawImageItr != itsRawImage.end())
  {
    *diffImageItr = abs((*rawImageItr) - (*finalImageItr));
    ++diffImageItr; ++rawImageItr; ++finalImageItr;
  }
  return diffImage;
}

/*************************************************************************/

template <class FLOAT> std::vector<Image<PixRGB<FLOAT> > >
ScaleRemoveSurprise<FLOAT>::SRSgetDiffParts() const
{
  Image<PIX_H2SV_TYPE<FLOAT> > rawDiffImage;
  rawDiffImage.resize(itsImageSizeX,itsImageSizeY);

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    const Image<PIX_H2SV_TYPE<FLOAT> > outImage =
      itsRemoveSurprise[i].RSgetRawOutImage();
    const Image<PIX_H2SV_TYPE<FLOAT> > inImage  =
      itsRemoveSurprise[i].RSgetRawInImage();

    typename Image<PIX_H2SV_TYPE<FLOAT> >::const_iterator outImageItr     =
      outImage.begin();
    typename Image<PIX_H2SV_TYPE<FLOAT> >::const_iterator inImageItr      =
      inImage.begin();
    typename Image<PIX_H2SV_TYPE<FLOAT> >::iterator rawDiffImageItr       =
      rawDiffImage.beginw();

    while(outImageItr != outImage.end())
    {
      (*rawDiffImageItr) += (*outImageItr) - (*inImageItr);
      ++outImageItr; ++inImageItr; ++rawDiffImageItr;
    }
  }

  typename Image<PIX_H2SV_TYPE<FLOAT> >::iterator rawDiffImageItr =
    rawDiffImage.beginw();

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

template <class FLOAT>
std::vector<Image<FLOAT> > ScaleRemoveSurprise<FLOAT>::SRSgetBetaParts() const
{
  Image<PixHyper<FLOAT,6> > outBetaImage;
  outBetaImage.resize(itsImageSizeX,itsImageSizeY);

  for(ushort i = 0; i < itsMaxIndex; i++)
  {
    const Image<PixHyper<FLOAT,6> > betaImage =
      itsRemoveSurprise[i].RSgetBetaImage();

    typename Image<PixHyper<FLOAT,6> >::const_iterator betaImageItr =
      betaImage.begin();
    typename Image<PixHyper<FLOAT,6> >::iterator outBetaImageItr    =
      outBetaImage.beginw();

    PixHyper<FLOAT,6> pIndex((FLOAT)itsMaxIndex);

    while(betaImageItr != betaImage.end())
    {
      (*outBetaImageItr) += (*betaImageItr)/pIndex;
      ++betaImageItr; ++outBetaImageItr;
    }
  }

  Image<FLOAT> baseImage;
  baseImage.resize(itsImageSizeX,itsImageSizeY);
  std::vector<Image<FLOAT> > outImageVec(6,baseImage);

  for(ushort x = 0; x < itsImageSizeX; x++)
  {
    for(ushort y = 0; y < itsImageSizeY; y++)
    {
      outImageVec[0].setVal(x,y,outBetaImage.getVal(x,y).p[0]);
      outImageVec[1].setVal(x,y,outBetaImage.getVal(x,y).p[1]);
      outImageVec[2].setVal(x,y,outBetaImage.getVal(x,y).p[2]);
      outImageVec[3].setVal(x,y,outBetaImage.getVal(x,y).p[3]);
      outImageVec[4].setVal(x,y,outBetaImage.getVal(x,y).p[4]);
      outImageVec[5].setVal(x,y,outBetaImage.getVal(x,y).p[5]);
    }
  }
  return outImageVec;
}


/*************************************************************************/
template class ScaleRemoveSurprise<float>;

#endif
