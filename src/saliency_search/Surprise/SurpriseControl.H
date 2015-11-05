/*!@file Surprise/SurpriseControl.H attempt to remove surprise from image */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Surprise/SurpriseControl.H $
// $Id: SurpriseControl.H 7063 2006-08-29 18:26:55Z rjpeters $
//

#ifndef SURPRISE_CONTROL_H_DEFINED
#define SURPRISE_CONTROL_H_DEFINED


#include "Image/Image.H"
#include "Image/MathOps.H"
#include "Image/ColorOps.H"
#include "Image/Pixels.H"
#include "Raster/Raster.H"
#include "Util/Assert.H"
#include "Util/Timer.H"

#include <fstream>
#include <iostream>
#include <math.h>
#include <string>
#include <vector>
#include <deque>

#define SC_NEAR_BLACK   0.01F
#define SC_LOW_SAT      0.01F
#define SMALL_SALIENCY  0.0F
#define SCALE_CHANNELS  6
#define SC_ORIENTS      4
#define SC_EXP          20
#define SC_MAX_CHANNELS 16
#define SC_DEBUG        false

using namespace std;

/** @name GlobalEnumerations
 *  These varables and constant arrays help allign the data and keep everything
 *  consistant
 */
//@{

//! channel types enumerated should have same number as SC_MAX_CHANNELS
enum sc_channels
{
  SC_DR0,
  SC_DR1,
  SC_DR2,
  SC_DR3,
  SC_GA0,
  SC_GA1,
  SC_GA2,
  SC_GA3,
  SC_IN,
  SC_FL,
  SC_RG,
  SC_BY,
  SC_H1,
  SC_H2,
  SC_HS,
  SC_HV
};

const std::string sc_channel_name[SC_MAX_CHANNELS] =
{
  "Direction 0",
  "Direction 1",
  "Direction 2",
  "Direction 3",
  "Orientation 0",
  "Orientation 1",
  "Orientation 2",
  "Orientation 3",
  "Intensity",
  "Flicker",
  "Red/Green",
  "Blue/Yellow",
  "H2SV H1",
  "H2SV H2",
  "H2SV S",
  "H2SV V"
};

const std::string sc_channel_name_abv[SC_MAX_CHANNELS] =
{
  "Dir_0",
  "Dir_1",
  "Dir_2",
  "Dir_3",
  "Ori_0",
  "Ori_1",
  "Ori_2",
  "Ori_3",
  "Intens",
  "Flicker",
  "RG",
  "BY",
  "H1",
  "H2",
  "HS",
  "HV"
};
//@}


/*************************************************************************/
//! Control surprise in a movie
template <class PIXTYPE, class BETATYPE, class FLOAT> class SurpriseControl
{
public:
  SurpriseControl(const ushort sizeX, const ushort sizeY);
  SurpriseControl();
  ~SurpriseControl();
  /** @name InputAndInit
   * These methods are used to init the object and input maps and variables
   * that affect the way SurpriseControl will work. These methods only need
   * to be called once
   */
  //@{
  //! Init the basic variables
  void SCinit(const ushort sizeX, const ushort sizeY);
  void SCsetMasterConspicBias(const FLOAT bias);
  //! use max level from conspicuity map rather than sum the levels
  void SCuseMaxLevel(const bool useML);
  //! Shoud we use the temporal component? default is yes
  void SCuseTemporal(const bool useTMP);
  //! normalize the biases with scale
  void SCnormalizeBiasWithScale(const bool useNBS);
  //! Tell us we are going to use this channel and set the bias
  void SCsetConspicBias(const FLOAT chan,
                        const int chan_enum);
  //! give me my scale if any
  void SCsetMyScale(const ushort scale);
  //! set the bias over a single axis
  void SCsetAxisBias(const FLOAT X, const FLOAT Y, const FLOAT Z);
  //! set the bias per H2SV channel on output
  void SCsetH2SVBias(const FLOAT H1, const FLOAT H2,
                     const FLOAT S,  const FLOAT V);
  //! set the decay term over the beta map
  void SCsetLambda(const FLOAT lambda);
  //! set which frame if any is a target frame
  void SCsetTargetFrame(const uint frame);
  //! set how much the original image will be combined back
  void SCsetOriginalImageWeight(const FLOAT origImageBias);
  //! create the seperable filters per image
  void SCcreateAndersonSepFilters(const ushort size);
  //! create the seperable filters per image
  void SCcreateSepFilters(const FLOAT spatSigma,
                          const FLOAT tempSigma,
                          const FLOAT stdDevSize);
  //! find parameters over convolution
  void SCfindConvolutionEndPoints();
  //@}

  /** @name InputPerFrame
   * These methods need to be feed data per frame such as the saliency map
   * and the raw image for each frame. These should be called each frame.
   */
  //@{
  //! input the raw image frame for processing
  void SCinputRawImage(const Image<PixRGB<FLOAT> >& rawImage);
  //! input the salmap for this frame
  void SCinputSalMap(const Image<FLOAT>& salMap);
  //! input the intensity conspicuity map for this frame
  void SCinputConspicMap(const Image<FLOAT>& cmap,const int cmap_enum);
  //@}

  /** @name InputOptional
   *  These methods allow optional masks or bayes weight images to be
   *  set
   */
  //@{
  //! Input a bayes weight image to bias surprise reduction
  void SCinputBayesWeightImage(const Image<FLOAT> &bayesImage);
  //! Input an independant mask to control where we apply the filter
  void SCinputMaskImage(const Image<FLOAT> &maskImage);
  //@}

  /** @name RunPerFrame
   *  Call to SCprocessFrameSeperable to run each frame. It should call
   *  SCcomputeNewBeta and SCseperateConv for you.
   */
  //@{
  //! compute new betaImage values
  void SCcomputeNewBeta();
  //! process a single frame of video using seperable filters
  void SCprocessFrameSeperable();
  //! find the local channel biases
  void SCcomputeLocalBias();
  //! Helper method to call SCseperateConv over x,y and z
  void SCseperateConvXYZ();
  //! Helper method to call SCseperateConv over x,y but not z
  void SCseperateConvXY();
  //! process each axis by itself
  void SCseperateConv(const char axis);
  //@}

  /** @name Output
   * Output from each frame can be obtained by calling these methods
   */
  //@{
  //! get the alternative surprise increased version of the image
  Image<PixRGB<FLOAT> > SCgetSharpened(const PIXTYPE scale_factor) const;
  //! process and return the final image
  Image<PixRGB<FLOAT> > SCgetFrame() const;
  //! process and return the filtered image
  Image<PixRGB<FLOAT> > SCgetOutImage() const;
  //! Return the temporal offset input image
  Image<PixRGB<FLOAT> > SCgetInImage() const;
  //! get the raw PIXTYPE output image
  Image<PIXTYPE> SCgetRawOutImage() const;
  //! get the input image converted as PIXTYPE
  Image<PIXTYPE> SCgetRawInImage() const;
  //! get the beta image of smoothed salMaps used in filter
  Image<BETATYPE> SCgetBetaImage() const;
  //! Get the temporal offset we are using
  ushort SCgetTemporalOffset() const;
  //! Is the output ready?
  bool   SCisOutputReady() const;
  //! Get the local bias maps proportional to effect
  void   SCgetLocalBiasImages(Image<FLOAT> &H1, Image<FLOAT> &H2,
                              Image<FLOAT> &S,  Image<FLOAT>  &V) const;
  //! Get the y and z parts of the seperable filter
  void   SCgetSeperableParts(Image<PIXTYPE> &Zimg, Image<PIXTYPE> &Yimg) const;

  //@}
private:
  /** @name FrameBuffers
   *  These hold full frames to allow the temporal component to work
   *  The frame buffer holds raw frames while the beta image stores
   *  the fully weighted surprise bias that should correspond with the
   *  image
   */
  //@{
  //! The basic frame buffer, holds either smoothed or non-smoothed
  std::deque<Image<PIXTYPE> >  itsFrameBuffer;
  //! Pointer to the current frame in the frame buffer
  Image<PIXTYPE>               *itsFrameCurrent;
  //! Temporal smoothed surprise maps
  std::deque<Image<BETATYPE> > itsBetaImage;
  //! Pointer to the current frame in the beta image
  Image<BETATYPE>              *itsBetaCurrent;
  //@}
  /** @name MiscImageHolders
   *  These are images that hold a variety of intermediate images,
   *  biases and input-output images
   */
  //@{
  //! intermediate processed image
  std::vector<Image<PIXTYPE> >     itsInterImage;
  //! input maps of the post surprise processed image Direction
  Image<FLOAT>          itsConspicMap[SC_MAX_CHANNELS];
  //! Set of biases for each channel conspicuity map
  FLOAT                 itsConspicMapBias[SC_MAX_CHANNELS];
  //! are we using this channel, set by setting the bias
  bool                  itsUseConspicMap[SC_MAX_CHANNELS];
  //! The input for this frame
  Image<PIXTYPE> itsInImage;
  //! The output for this frame
  Image<PIXTYPE>        itsOutImage;
  //! The final image after we process by the surprise map
  Image<PixRGB<FLOAT> >         itsFinalImage;
  //@}

  /** @name BasicBiasImages
   *  These are images that store masks and biases used to weight the
      operations of the convolution operations. Per channel weights are
      stored as a betatype image.
  */
  //@{
  //! the current saliency map
  Image<FLOAT>           itsSalMap;
  //! Special independant mask image in addition to surprise mask
  Image<FLOAT>           itsMaskImage;
  //! input a bayesian bias image
  Image<FLOAT>                  itsBayesWeightImage;
  //! Local bias applied after accounting for channels
  Image<FLOAT>                  itsLocalBiasH1;
  //! Local bias applied after accounting for channels
  Image<FLOAT>                  itsLocalBiasH2;
  //! Local bias applied after accounting for channels
  Image<FLOAT>                  itsLocalBiasS;
  //! Local bias applied after accounting for channels
  Image<FLOAT>                  itsLocalBiasV;
  //@}

  /** @name ConvolutionGroup
   *  These are private members for storing pointers and indexes that
   *  provide a basis for convolution so that things such as offsets
   *  and kernel placement do not have to be computed on the fly
  */
  //@{
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
  //@}

  //! is true if saliency is very small at this location
  Image<bool>                   itsSmallSaliency;
  //! store messages that tell us it we did not init something
  std::vector<std::string>      itsInitMessage;
  /** @name KernelParts
   *  These are the three dimensions of a seperable convolution kernel
   */
  //@{
  //! store the X part of the kernel
  std::vector<FLOAT>            itsKalmanKernelX;
  //! store the Y part of the kernel
  std::vector<FLOAT>            itsKalmanKernelY;
  //! store the Z part of the kernel
  std::vector<FLOAT>            itsKalmanKernelZ;
  //@}

  //! store messages that tell us it we did not init something
  std::vector<bool>             itsInit;
  //! blank pixel used in run time typing
  PixHyper<FLOAT,SC_MAX_CHANNELS> itsHyper;
  //! set to true when the frame buffer is full
  bool                   itsBufferFull;
  bool                   itsInitBuffer;
  /** @name ConstantBiases
   *  These are the constant biases used to affect the whole process
   *  For instance, we can bias each axis of x,y and z independantly if we want
   *  or we can bias each channel.
   *
   */
  //@{
  //! the channel biases
  FLOAT                         itsConBias[SC_MAX_CHANNELS];
  //! A master bias which is applied equally to all channels
  FLOAT                         itsMasterConspicBias;
  //! The decay term over beta
  FLOAT                         itsLambda;
  //! Bias over the X axis
  FLOAT                         itsXBias;
  //! Bias over the Y axis
  FLOAT                         itsYBias;
  //! Bias over the Z axis
  FLOAT                         itsZBias;
  //! Bias to H1
  FLOAT                         itsH1Bias;
  //! Bias to H2
  FLOAT                         itsH2Bias;
  //! Bias to S
  FLOAT                         itsSBias;
  //! Bias to V
  FLOAT                         itsVBias;
  //! how much of the original image should we add back
  FLOAT                         itsOriginalImageWeight;
  //@}

  /** @name ConstantSizesAndOffsets
   *  These define constant sizes for the kernel and input image as well as
   *  offsets to target frame.
   */
  //@{
  //! What is the target frame if any
  uint                          itsTargetFrame;
  //! Internal Iteration counter
  uint                          itsIterCounter;
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
  //! the size of the image in Z (deque size)
  ushort                        itsImageSizeZ;
  //! offset on when we start the temporal component
  ushort                        itsTemporalOffset;
  //! My scale if known
  ushort                        itsScale;
  //@}

  //! Use max or combined surprise values
  bool                          itsUseMaxLevel;
  //! Use correlation matrix for biased surprise removal
  bool                          itsUseCorrMatrixSet;
  //! turn on using bayesian weighting of surprise reduction
  bool                          itsUseBayesWeightImage;
  //! use another independant mask image
  bool                          itsUseMaskImage;
  //! Set to true if you have a target frame
  bool                          itsUseTargetFrame;
  //! are we ready to give output
  bool                          itsOutputReady;
  //! Should we use the temporal component or not?
  bool                          itsUseTemporal;
  //! Should we normalize the H1,H2,S and V bias with scale?
  bool                          itsNormalizeBiasWithScale;
};
#endif

