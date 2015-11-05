/*!@file Psycho/StimMaker.H make different kind of visual test stimuli
 */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimMaker.H $
// $Id: StimMaker.H 6262 2006-02-17 22:40:07Z rjpeters $
//

#ifndef STIM_MAKER_H_DEFINED
#define STIM_MAKER_H_DEFINED

#include "Psycho/StimMakerParam.H"

//! class for making visual stimuli
class StimMaker
{
public:
  //! Init the stim maker with the image size and number of frames
  StimMaker(const unsigned short ImageSizeX,
            const unsigned short ImageSizeY,
            const unsigned short frameCount,
            const char bgColor);
  //! default destructor
  ~StimMaker();
  //! re-init the frames to the background color
  /*!This should be called every time you want a completely new stim */
  void SM_init(const char bgColor);
  //! reset the frames and back ground truth to the background color
  void SM_setStandardBackGround(const unsigned char bgColor);
  //! override the built in blink rates of stimuli
  void SM_overRideRates(const unsigned char slowRate,
                        const unsigned char fastRate,
                        const unsigned char stopRate);
  //! overide the built in color sets for target and distractors
  void SM_setCustomColors(const PixRGB<float> BGcolor,
                          const PixRGB<float> TargetColor,
                          const PixRGB<float> DistColor);
  //! check if we need to speed up the blink rate
  inline ushort SM_speedUpCheck(ushort currBlinkRate,
                                const ushort frame,
                                const ushort changeFrame,
                                const unsigned char stimRate,
                                const unsigned char useSmoothRateChange) const;
  //! check if we need to slow down the blink rate
  inline ushort SM_speedDownCheck(ushort currBlinkRate,
                                  const ushort frame,
                                  const ushort changeFrame,
                                  const unsigned char stimRate,
                                  const unsigned char useSmoothRateChange) const;
  //! draw a steady patch where the ground truth is
  inline void SM_drawGroundTruth(const ushort frame,
                                 const unsigned char stimSizeX,
                                 const unsigned char stimSizeY,
                                 const ushort PosX,
                                 const ushort PosY,
                                 const bool targetOn);

  //! draw a single object in the frame and ground truth image
  inline void SM_drawSingleTarget(const ushort frame,
                               const unsigned char stimShape,
                               const unsigned char stimColor,
                               const unsigned char stimSizeX,
                               const unsigned char stimSizeY,
                               const ushort PosX,
                               const ushort PosY,
                               const float stimOri,
                               const float shapePositionJitter,
                               const float shapeOrientationJitter,
                               const bool target);
  //! create a new stim movie with the provided params
  /*! This will create a stimulus movie with one target and nxm-1
      distractors. The shape and color of the target and distractors can
      be specified or they can be randomly chosen by the algorith. Additionally
      you can select whether to offset the blinking of stims or just start
      each one uniform at the start.
  */
  void SM_makeUniformStim(const StimMakerParam &stim);
  //! get the stim series back
  std::vector<Image<PixRGB<float> > > SM_getStim() const;
  //! get back the ground truth image
  std::vector<Image<PixRGB<float> > > SM_getGroundTruth() const;
private:
  bool           itsUseCustomColors;
  unsigned char  itsBGcolor;
  unsigned char  itsSlowRate;
  unsigned char  itsFastRate;
  unsigned char  itsStopRate;
  unsigned short itsSizeX;
  unsigned short itsSizeY;
  unsigned short itsFrameCount;
  unsigned int   itsRandomSeed;
  PixRGB<float>  itsRed;
  PixRGB<float>  itsOrange;
  PixRGB<float>  itsYellow;
  PixRGB<float>  itsGreen;
  PixRGB<float>  itsBlue;
  PixRGB<float>  itsPurple;
  PixRGB<float>  itsBlack;
  PixRGB<float>  itsWhite;
  PixRGB<float>  itsCustomBG;
  PixRGB<float>  itsCustomTarg;
  PixRGB<float>  itsCustomDist;
  PixRGB<float>  itsGTtargetColor;
  PixRGB<float>  itsGTtargetColorPatch1;
  PixRGB<float>  itsGTtargetColorPatch2;
  PixRGB<float>  itsGTtargetColorPatch1off;
  PixRGB<float>  itsGTtargetColorPatch2off;
  PixRGB<float>  itsGTdistColor;
  std::vector<PixRGB<float> >         itsColorVec;
  std::vector<Image<PixRGB<float> > > itsFrames;
  std::vector<Image<PixRGB<float> > > itsGroundTruth;
};

#endif // STIM_MAKER_H_DEFINED
