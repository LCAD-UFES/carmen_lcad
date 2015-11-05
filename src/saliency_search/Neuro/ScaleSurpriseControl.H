/*!@file Neuro/ScaleSurpriseControl.H attempt to remove surprise from image */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/ScaleSurpriseControl.H $
// $Id: ScaleSurpriseControl.H 10772 2009-02-05 17:28:49Z itti $
//

#ifndef SCALE_SURPRISE_CONTROL_H_DEFINED
#define SCALE_SURPRISE_CONTROL_H_DEFINED

#include "Surprise/SurpriseControl.H"

#include "Channels/BlueYellowChannel.H"
#include "Channels/ChannelBase.H"
#include "Channels/ColorChannel.H"
#include "Channels/DirectionChannel.H"
#include "Channels/FlickerChannel.H"
#include "Channels/GaborChannel.H"
#include "Channels/H2SVChannel.H"
#include "Channels/Hue1Channel.H"
#include "Channels/Hue2Channel.H"
#include "Channels/IntensityChannel.H"
#include "Channels/MotionChannel.H"
#include "Channels/OrientationChannel.H"
#include "Channels/RedGreenChannel.H"
#include "Channels/SaturationChannel.H"
#include "Channels/ValueIntensityChannel.H"
#include "Component/OptionManager.H"
#include "Image/MathOps.H"
#include "Image/ShapeOps.H"
#include "Image/ColorOps.H"
#include "Image/Normalize.H"
#include "Neuro/Brain.H"
#include "Neuro/SaliencyMap.H"
#include "Neuro/VisualCortex.H"
#include "Transport/FrameInfo.H"
#include "Transport/FrameOstream.H"
#include "Util/readConfig.H"
#include "rutz/shared_ptr.h"



#define PIX_H2SV_TYPE PixH2SV2

using namespace std;
//! remove surprise using scales, FLOAT is either float or double for precision
template <class FLOAT> class ScaleSurpriseControl
{
public:
  //! default constructor, call with base image size for frames
  ScaleSurpriseControl(const ushort sizeX,
                       const ushort sizeY,
                       const string confFile = "null");
  //! default constructor need to call SSCinit and SSCreadConfig
  ScaleSurpriseControl();
  //! default destructor
  ~ScaleSurpriseControl();

  /** @name InputAndInit
   * These methods are used to init the object and input maps and variables
   * that affect the way ScaleSurpriseControl will work. These methods only
   *  need to be called once
   */
  //@{
  //! Input LevelSpec info directly insted of from a config file
  void SSCsetLevelSpecInfo(const uint levMin,   const uint levMax,
                           const uint delMin,   const uint delMax,
                           const uint mapLevel, const uint maxIndex,
                           const uint maxDepth);
  //! read in config values
  void SSCreadConfig(const string confFile);
  //! is called by default constructor, sets stuff up
  void SSCinit(const ushort sizeX, const ushort sizeY);
  //@}

  /** @name InputPerFrame
   * These methods need to be feed data per frame such as the saliency map
   * and the raw image for each frame. These should be called each frame.
   */
  //@{
  //! input a raw frame, give the frame number as well
  void SSCinputRawImage(const Image<PixRGB<FLOAT> >& rawImage);
  //! input the base saliency map for this frame
  void SSCinputSalMap(const Image<FLOAT>& salMap);
  //@}

  /** @name InputOptional
   *  These methods allow optional masks or bayes weight images to be
   *  set
   */
  //@{
  //! input and independant mask image if desired
  void SSCinputMaskImage(const Image<FLOAT>& maskImage);
  //! input a bayes weight image if desired
  void SSCinputBayesWeightImage(const Image<FLOAT>& bayesImage);
  //@}

  /** @name RunPerFrame
   *  Call to SSCprocessFrame to run each frame. It should call
   *  each SurpriseControl at each scale for you.
   */
  //@{
  //! process this movie frame using a brain
  void SSCprocessFrame(Brain* brain);
  //! process this movie frame, no brain NO-OP
  void SSCprocessFrame(const uint frame);
  //@}

  /** @name Output
   * Output from each frame can be obtained by calling these methods
   */
  //@{
  //! get the resulting frame processed
  Image<PixRGB<FLOAT> >               SSCgetFrame()     const;
  //! get the difference image between the input and final output
  Image<PixRGB<FLOAT> >               SSCgetDiffImage(
                                      const bool normalize = false) const;
  //! compute difference difference parts for this image over rawImage
  std::vector<Image<PixRGB<FLOAT> > > SSCgetDiffParts() const;
  //! compute the combined beta map for all scales
  std::vector<Image<FLOAT> >          SSCgetBetaParts(
                                      const bool normalize = false) const;
  //! return the bias images used in smoothing etc.
  void SSCgetBiasParts(std::vector<Image<PixRGB<FLOAT> > > &H1,
                       std::vector<Image<PixRGB<FLOAT> > > &H2,
                       std::vector<Image<PixRGB<FLOAT> > > &S,
                       std::vector<Image<PixRGB<FLOAT> > > &V) const;
  //! Get the y and z parts of the seperable filter
  void SSCgetSeperableParts(std::vector<Image<PixRGB<FLOAT> > > &Zimgs,
                            std::vector<Image<PixRGB<FLOAT> > > &Yimgs,
                            const bool normalize = false) const;
  //@}
private:
  //! Internal frame counter
  unsigned long itsFrameCounter;
  //! Vector of surprise removers
  std::vector<SurpriseControl<PIX_H2SV_TYPE<FLOAT>,
                              PixHyper<FLOAT,SC_MAX_CHANNELS>,FLOAT> >
  itsSurpriseControl;
  //! base for readConfig
  readConfig itsReadConfig;
  //! result image per scale
  std::vector<Image<PixRGB<FLOAT> > > itsResultImages;
  //! bias to apply to each scale
  std::vector<FLOAT>      itsScaleBias;
  //! Power to the filters at each scale
  std::vector<FLOAT>      itsScalePower;
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
  //! a bayes weight image if desired
  Image<FLOAT>            itsBayesWeightImage;
  //! an independant mask if desired
  Image<FLOAT>            itsMaskImage;
  //! bias to apply to each channel
  FLOAT       itsConspicMapBias[SC_MAX_CHANNELS];
  //! bias to apply to X axis convolution
  FLOAT       itsAxisBiasX;
  //! bias to apply to Y axis convolution
  FLOAT       itsAxisBiasY;
  //! bias to apply to Z axis convolution
  FLOAT       itsAxisBiasZ;
  //! Lambda to smooth reaction of filters over time
  FLOAT       itsLambda;
  //! How big should we convolve in standard deviation
  FLOAT       itsStdSize;
  //! What should be the size of the temporal filter
  FLOAT       itsZSigma;
  //! Bias to H1
  FLOAT       itsH1Bias;
  //! Bias to H2
  FLOAT       itsH2Bias;
  //! Bias to S
  FLOAT       itsSBias;
  //! Bias to V
  FLOAT       itsVBias;
  //! Master bias over all conspicuity maps
  FLOAT       itsMasterConspicBias;
  //! how much of the original image should we add back
  FLOAT       itsOriginalImageWeight;
  //! If we sharpen the image, what factor should we use?
  FLOAT       itsSharpFactorH1;
  //! If we sharpen the image, what factor should we use?
  FLOAT       itsSharpFactorH2;
  //! If we sharpen the image, what factor should we use?
  FLOAT       itsSharpFactorS;
  //! If we sharpen the image, what factor should we use?
  FLOAT       itsSharpFactorV;
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
  //! should we use the max level and not sum for conspicuity maps?
  bool        itsUseMaxLevel;
  //! Have we set the levelspec?
  bool        itsLevelSpecSet;
  //! should we get a reduced or sharpened image?
  bool        itsGetReduced;
  //! Use an emulation of Andersons seperable filter for the kernel
  bool        itsUseAndersonSeperable;
  //! Should we use the temporal component when smoothing out surprise
  bool        itsUseTemporal;
  //! Should we normalize the H1,H2,S and V bias with scale?
  bool        itsNormalizeBiasWithScale;
};


#endif
