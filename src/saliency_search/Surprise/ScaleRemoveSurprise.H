/*!@file Surprise/ScaleRemoveSurprise.H attempt to remove surprise from image */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/ScaleRemoveSurprise.H $
// $Id: ScaleRemoveSurprise.H 6795 2006-06-29 20:45:32Z rjpeters $
//

#ifndef SCALE_REMOVE_SURPRISE_H_DEFINED
#define SCALE_REMOVE_SURPRISE_H_DEFINED

#include "Surprise/RemoveSurprise.H"
#include "Util/readConfig.H"
#include "Image/MathOps.H"

#define PIX_H2SV_TYPE PixH2SV1

using namespace std;
//! remove surprise using scales, FLOAT is either float or double for precision
template <class FLOAT> class ScaleRemoveSurprise
{
public:
  //! default constructor, call with base image size for frames
  ScaleRemoveSurprise(const ushort sizeX,
                      const ushort sizeY,
                      const string confFile = "null");
  //! default destructor
  ~ScaleRemoveSurprise();
  //! is called by default constructor, sets stuff up
  void SRSinit(const ushort sizeX,
               const ushort sizeY);
  //! input a raw frame, give the frame number as well
  void SRSinputRawImage(const Image<PixRGB<FLOAT> >& rawImage,
                        const uint frame);
  //! input the base saliency map for this frame
  void SRSinputSalMap(const Image<FLOAT>& salMap);
  //! input an "anti" image set that reduce the process for target features
  void SRSsetAntiWeights();
  //! input an "anti" image set that reduce the process for target features
  void SRSsetAntiWeightsInteract(const uint aframes,
                                 const uint bframes);
  //! set up initial bayes weights for biasing from features
  void SRScomputeBayesFeatureBias(const uint frames,
                                  const string baseFileNamePrefix,
                                  const string antiFileNamePrefix);
  //! Open a bayes feature bias already computed
  void SRSopenBayesFeatureBias(const string baseFileNamePrefix,
                               const string antiFileNamePrefix);
  //! Find weights for biasing from features for current image
  void SRScomputeBayesFeatureCurrent(const uint frame,
                                     const string fileNamePrefix);
  //! process this movie frame
  void SRSprocessFrame();
  //! get the resulting frame processed
  Image<PixRGB<FLOAT> >               SRSgetFrame()     const;
  //! get the difference image between the input and final output
  Image<PixRGB<FLOAT> >               SRSgetDiffImage() const;
  //! compute difference difference parts for this image over rawImage
  std::vector<Image<PixRGB<FLOAT> > > SRSgetDiffParts() const;
  //! compute the combined beta map for all scales
  std::vector<Image<FLOAT> >          SRSgetBetaParts() const;
private:
  //! Vector of surprise removers
  std::vector<RemoveSurprise<PIX_H2SV_TYPE<FLOAT>,PixHyper<FLOAT,6>,FLOAT> >
  itsRemoveSurprise;
  //! base for readConfig
  readConfig itsReadConfig;
  //! result image per scale
  std::vector<Image<PixRGB<FLOAT> > > itsResultImages;
  //! bias to apply to each scale
  std::vector<FLOAT>      itsScaleBias;
  //! Power to the filters at each scale
  std::vector<FLOAT>      itsScalePower;
  //! how much to desaturate surprising color
  std::vector<FLOAT>      itsDesatBias;
  //! store reverse computed filter sizez Z
  std::vector<FLOAT>      itsFilterSizesZ;
  //! store the actual pyramid image sizes reverse computed
  std::vector<ushort>     itsImageSizesX;
  //! store the actual pyramid image sizes reverse computed
  std::vector<ushort>     itsImageSizesY;
  //! store reverse computed filter sizez X
  std::vector<ushort>     itsFilterSizesX;
  //! store reverse computed filter sizez Y
  std::vector<ushort>     itsFilterSizesY;
  //! the raw input image
  Image<PixRGB<FLOAT> >   itsRawImage;
  //! the final image
  Image<PixRGB<FLOAT> >   itsFinalImage;
  //! stored salmap frame
  Image<FLOAT>            itsSalMap;
  //! base correlation image
  Image<FLOAT>            itsBaseCorr;
  //! compute R from Corr for display purposes
  Image<FLOAT>            itsBaseR;
  //! base image mean
  Image<FLOAT>            itsBaseMean;
  //! base image STD
  Image<FLOAT>            itsBaseSTD;
  //! base image SS
  Image<FLOAT>            itsBaseSS;
  //! base likelyhood image
  Image<FLOAT>            itsBaseLikelyhood;
  //! Non normalized likelyhood
  Image<FLOAT>            itsNonNormalizedBaseL;
  //! anti correlation image
  Image<FLOAT>            itsAntiCorr;
  //! compute R from Corr for display purposes
  Image<FLOAT>            itsAntiR;
  //! anti image mean
  Image<FLOAT>            itsAntiMean;
  //! anti image STD
  Image<FLOAT>            itsAntiSTD;
  //! anti image SS
  Image<FLOAT>            itsAntiSS;
  //! anti likelyhood image
  Image<FLOAT>            itsAntiLikelyhood;
  //! Non normalized likelyhood
  Image<FLOAT>            itsNonNormalizedAntiL;
  //! Bayesian P image between anti and base image and input
  Image<FLOAT>            itsBayesImage;
  //! Augmented bayes image for beliefs
  Image<FLOAT>            itsBeliefImage;
  //! bias to apply to X axis convolution
  FLOAT       itsAxisBiasX;
  //! bias to apply to Y axis convolution
  FLOAT       itsAxisBiasY;
  //! bias to apply to Z axis convolution
  FLOAT       itsAxisBiasZ;
  //! bias to apply to intensity channel
  FLOAT       itsINBias;
  //! bias to apply to direction channel
  FLOAT       itsDRBias;
  //! bias to apply to flicker channel
  FLOAT       itsFLBias;
  //! bias to apply to gaussian channel
  FLOAT       itsGABias;
  //! bias to apply to red/green channel
  FLOAT       itsRGBias;
  //! bias to apply to blue/yellow channel
  FLOAT       itsBYBias;
  //! Lambda to smooth reaction of filters over time
  FLOAT       itsLambda;
  //! How big should we convolve in standard deviation
  FLOAT       itsStdSize;
  //! What should be the size of the temporal filter
  FLOAT       itsZSigma;
  //! Bias to H1
  FLOAT       itsH1bias;
  //! Bias to H2
  FLOAT       itsH2bias;
  //! Bias to S
  FLOAT       itsSbias;
  //! Bias to V
  FLOAT       itsVbias;
  //! The current video frame we are looking at
  uint        itsFrame;
  //! base image N
  uint        itsBaseN;
  //! anti image N
  uint        itsAntiN;
  //! base size of the filters used in the image pyramid e.g. 5 or 9
  ushort      itsBaseFilterSize;
  //! LevelSpec LevMin
  ushort      itsLevMin;
  //! LevelSpec LevMax
  ushort      itsLevMax;
  //! LevelSpec DelMin
  ushort      itsDelMin;
  //! LevelSpec DelMax
  ushort      itsDelMax;
  //! LevelSpec itsMapLevel
  ushort      itsMapLevel;
  //! LevelSpec itsMaxIndex
  ushort      itsMaxIndex;
  //! LevelSpec itsMaxDepth
  ushort      itsMaxDepth;
  //! This images size X
  ushort      itsImageSizeX;
  //! This images size Y
  ushort      itsImageSizeY;
  //! This images size X at first pyramid level
  ushort      itsImageBaseX;
  //! This images size Y at first pyramid level
  ushort      itsImageBaseY;
  //! The filter size X in pyramid
  ushort      itsFilterSizeX;
  //! The filter size Y in pyramid
  ushort      itsFilterSizeY;
  //! should we use a true kalman filter
  bool        itsUseKalman;
  //! should we use the max level and not sum for conspicuity maps?
  bool        itsUseMaxLevel;
};


#endif
