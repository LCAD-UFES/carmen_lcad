/*!@file Surprise/RemoveSurprise.H attempt to remove surprise from image */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/RemoveSurprise.H $
// $Id: RemoveSurprise.H 6795 2006-06-29 20:45:32Z rjpeters $
//

#ifndef REMOVE_SURPRISE_H_DEFINED
#define REMOVE_SURPRISE_H_DEFINED

#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <deque>

#include "Raster/Raster.H"
#include "Util/Assert.H"
#include "Util/Timer.H"
#include "Image/Image.H"
#include "Image/Pixels.H"

#define SMALL_SALIENCY 0.005F
#define NEAR_BLACK     0.005F
#define SCALE_CHANNELS 6

using namespace std;

//! smooth surprise in a movie
template <class PIXTYPE, class BETATYPE, class FLOAT> class RemoveSurprise
{
public:
  RemoveSurprise(const ushort sizeX, const ushort sizeY);
  RemoveSurprise();
  ~RemoveSurprise();
  //! Init the basic variables
  void RSinit(const ushort sizeX, const ushort sizeY);
  //! set true to use a more true kalman filter
  void RSuseTrueKalman(const bool useTK);
  //! use max level from conspicuity map rather than sum the levels
  void RSuseMaxLevel(const bool useML);
  //! set the bias to something other than 1.0F
  void RSsetConspicBias(const FLOAT CO, const FLOAT MO,
                        const FLOAT OR, const FLOAT IN);
  //! set the bias to something other than 1.0F
  void RSsetConspicBias(const FLOAT IN, const FLOAT DR,
                        const FLOAT FL, const FLOAT GA,
                        const FLOAT RG, const FLOAT BY);
  //! set the bias over a single axis
  void RSsetAxisBias(const FLOAT X, const FLOAT Y, const FLOAT Z);
  //! set the bias per H2SV channel
  void RSsetH2SVBias(const FLOAT H1, const FLOAT H2,
                     const FLOAT S,  const FLOAT V);
  //! set the decay term over the beta map
  void RSsetLambda(const FLOAT lambda);
  //! set the bias to desaturate color at surprising locations
  void RSsetDesatBias(const FLOAT desat);
  //! input the raw image frame for processing
  void RSinputRawImage(const Image<PixRGB<FLOAT> >& rawImage,
                       const uint frame);
  //! input the salmap for this frame
  void RSinputSalMap(const Image<FLOAT>& salMap);
  //! input the color conspicuity map for this frame
  void RSinputConspicCO(const Image<FLOAT>& conspicCO);
  //! input the motion conspicuity map for this frame
  void RSinputConspicMO(const Image<FLOAT>& conspicMO);
  //! input the orientation conspicuity map for this frame
  void RSinputConspicOR(const Image<FLOAT>& conspicOR);
  //! input the intensity conspicuity map for this frame
  void RSinputConspicIN(const Image<FLOAT>& conspicIN);
  //! input the orientation conspicuity map for this frame
  void RSinputConspicDR(const Image<FLOAT>& conspicDR);
  //! input the orientation conspicuity map for this frame
  void RSinputConspicFL(const Image<FLOAT>& conspicFL);
  //! input the orientation conspicuity map for this frame
  void RSinputConspicGA(const Image<FLOAT>& conspicGA);
  //! input the orientation conspicuity map for this frame
  void RSinputConspicRG(const Image<FLOAT>& conspicRG);
  //! input the orientation conspicuity map for this frame
  void RSinputConspicBY(const Image<FLOAT>& conspicBY);
  //! Input a bayes weight image to bias surprise reduction
  void RSinputBayesWeightImage(const Image<FLOAT> &bayesImage);
  //! Input correlationMatrices for weight augmentation
  void RSsetCorrWeightMat(const std::vector<std::vector<FLOAT> > corAnti,
                          const std::vector<std::vector<FLOAT> > corBase);
  //! create the seperable filters per image
  void RScreateSepFilters(const FLOAT spatSigma,
                          const FLOAT tempSigma,
                          const FLOAT stdDevSize);
  //! find parameters over convolution
  void RSfindConvolutionEndPoints();
  //! process a single frame of video
  void RSprocessFrame();
  //! process a single frame of video using seperable filters
  void RSprocessFrameSeperable();
  //! process each axis by itself
  void RSseperateConv(const char axis);
  //! compute a new weight based on feature correlation matrix, used in f bias
  FLOAT RScomputeCor(const FLOAT IN, const FLOAT DR, const FLOAT FL,
                     const FLOAT GA, const FLOAT RG, const FLOAT BY,
                     const short idx) const;

  //! process and return the final image
  Image<PixRGB<FLOAT> > RSgetFrame();
  //! process and return the filtered image
  Image<PixRGB<FLOAT> > RSgetOutImage() const;
  //! get the raw PIXTYPE output image
  Image<PIXTYPE> RSgetRawOutImage() const;
  //! get the input image converted as PIXTYPE
  Image<PIXTYPE> RSgetRawInImage() const;
  //! get the beta image of smoothed salMaps used in filter
  Image<BETATYPE> RSgetBetaImage() const;
private:
  //! The basic frame buffer, holds either smoothed or non-smoothed
  std::deque<Image<PIXTYPE> >      itsFrameBuffer;
  //! intermediate processed image
  std::vector<Image<PIXTYPE> >     itsInterImage;
  //! intermediate processed image
  std::vector<Image<PIXTYPE> >     itsInterImageNorm;
  //! Anti image elementary correlation matrix
  std::vector<std::vector<FLOAT> > itsAntiCorrelationMat;
  //! Base image elementary correlation matrix
  std::vector<std::vector<FLOAT> > itsBaseCorrelationMat;
  //! The input for this frame
  Image<PIXTYPE>       itsInImage;
  //! The output for this frame
  Image<PIXTYPE>       itsOutImage;
  //! The output for this frame
  Image<BETATYPE>      itsBetaImage;
  //! The final image after we process by the surprise map
  Image<PixRGB<FLOAT> >         itsFinalImage;
  //! the current saliency map
  Image<FLOAT>                  itsSalMap;
  //! color conspicuity map
  Image<FLOAT>                  itsConspicCO;
  //! motion conspicuity map
  Image<FLOAT>                  itsConspicMO;
  //! orientation conspicuty map
  Image<FLOAT>                  itsConspicOR;
  //! intensity conspicuity map
  Image<FLOAT>                  itsConspicIN;
  //! input maps of the post surprise processed image Direction
  Image<FLOAT>                  itsConspicDR;
  //! input maps of the post surprise processed image Flicker
  Image<FLOAT>                  itsConspicFL;
  //! input maps of the post surprise processed image Gaussian
  Image<FLOAT>                  itsConspicGA;
  //! input maps of the post surprise processed image Red/Green
  Image<FLOAT>                  itsConspicRG;
  //! input maps of the post surprise processed image Blue/Yellow
  Image<FLOAT>                  itsConspicBY;
  //! input a bayesian bias image
  Image<FLOAT>                  itsBayesWeightImage;
  //! store the start point for this convolution in X
  Image<ushort>                 itsXStart;
  //! store the start point for this convolution in Y
  Image<ushort>                 itsYStart;
  //! store the start point for this convolution in Z
  Image<ushort>                 itsZStart;
  //! store the stop point for this convolution in X
  Image<ushort>                 itsXStop;
  //! store the stop point for this convolution in Y
  Image<ushort>                 itsYStop;
  //! store the stop point for this convolution in Z
  Image<ushort>                 itsZStop;
  //! store the start point for this convolution in X
  Image<ushort>                 itsKXStart;
  //! store the start point for this convolution in Y
  Image<ushort>                 itsKYStart;
  //! store the start point for this convolution in Z
  Image<ushort>                 itsKZStart;
  //! store messages that tell us it we did not init something
  std::vector<std::string>      itsInitMessage;
  //! store the X part of the kernel
  std::vector<FLOAT>            itsKalmanKernelX;
  //! store the Y part of the kernel
  std::vector<FLOAT>            itsKalmanKernelY;
  //! store the Z part of the kernel
  std::vector<FLOAT>            itsKalmanKernelZ;
  //! store messages that tell us it we did not init something
  std::vector<bool>             itsInit;
  //! blank pixel used in run time typing
  PixHyper<FLOAT,6>             itsHyper6;
   //! blank pixel used in run time typing
  PixHyper<FLOAT,4>             itsHyper4;
  //! The color bias
  FLOAT                         itsCObias;
  //! The motion bias
  FLOAT                         itsMObias;
  //! The orientation bias
  FLOAT                         itsORbias;
  //! The intensity bias
  FLOAT                         itsINbias;
  //! The directional bias
  FLOAT                         itsDRbias;
  //! The flicker bias
  FLOAT                         itsFLbias;
  //! The guassian bias
  FLOAT                         itsGAbias;
  //! The red/green bias
  FLOAT                         itsRGbias;
  //! The blue/yellow bias
  FLOAT                         itsBYbias;
  //! The decay term over beta
  FLOAT                         itsLambda;
  //! Bias over the X axis
  FLOAT                         itsXbias;
  //! Bias over the Y axis
  FLOAT                         itsYbias;
  //! Bias over the Z axis
  FLOAT                         itsZbias;
  //! Bias to H1
  FLOAT                         itsH1bias;
  //! Bias to H2
  FLOAT                         itsH2bias;
  //! Bias to S
  FLOAT                         itsSbias;
  //! Bias to V
  FLOAT                         itsVbias;
  //! Bias to desatuate surprsing color
  FLOAT                         itsDesatbias;
  //! The current frame number
  uint                          itsFrameNumber;
  //! the size of the X kernel
  ushort                        itsKernelSizeX;
  //! the size of the Y kernel
  ushort                        itsKernelSizeY;
  //! the size of the Z kernel
  ushort                        itsKernelSizeZ;
  //! the size of the image in X
  ushort                        itsImageSizeX;
  //! the size of the image in Y
  ushort                        itsImageSizeY;
  bool                          itsUseTrueKalman;
  bool                          itsInitBuffer;
  //! Use max or combined surprise values
  bool                          itsUseMaxLevel;
  //! Use correlation matrix for biased surprise removal
  bool                          itsUseCorrMatrixSet;
  //! turn on using bayesian weighting of surprise reduction
  bool                          itsUseBayesWeightImage;
};
#endif

