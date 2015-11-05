/*!@file Devices/DeviceOpts.H */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/DeviceOpts.H $
// $Id: DeviceOpts.H 15147 2012-02-06 23:22:31Z kai $
//

#ifndef DEVICES_DEVICEOPTS_H_DEFINED
#define DEVICES_DEVICEOPTS_H_DEFINED

struct ModelOptionCateg;
struct ModelOptionDef;

//! Audio-related options
extern const ModelOptionCateg MOC_AUDIO;

//! Command-line options for V4Lgrabber/IEEE1394grabber
//@{

//! Frame-grabbing-related options
extern const ModelOptionCateg MOC_FRAMEGRAB;

extern const ModelOptionDef OPT_FrameGrabberType;
extern const ModelOptionDef OPT_DeinterlacerType;
extern const ModelOptionDef OPT_FrameGrabberDevice;
extern const ModelOptionDef OPT_FrameGrabberChannel;
extern const ModelOptionDef OPT_FrameGrabberSubChan;
extern const ModelOptionDef OPT_FrameGrabberDims;
extern const ModelOptionDef OPT_FrameGrabberOffset;
extern const ModelOptionDef OPT_FrameGrabberMode;
extern const ModelOptionDef OPT_FrameGrabberByteSwap;
extern const ModelOptionDef OPT_FrameGrabberFPS;
extern const ModelOptionDef OPT_FrameGrabberNbuf;
extern const ModelOptionDef OPT_FrameGrabberBrightness;
extern const ModelOptionDef OPT_FrameGrabberColour;
extern const ModelOptionDef OPT_FrameGrabberHue;
extern const ModelOptionDef OPT_FrameGrabberSaturation;
extern const ModelOptionDef OPT_FrameGrabberContrast;
extern const ModelOptionDef OPT_FrameGrabberWhiteness;
extern const ModelOptionDef OPT_FrameGrabberExposure;
extern const ModelOptionDef OPT_FrameGrabberSharpness;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalBU;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalRV;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalTargetR;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalTargetG;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalTargetB;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalReferenceR;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalReferenceG;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalReferenceB;
extern const ModelOptionDef OPT_FrameGrabberGamma;
extern const ModelOptionDef OPT_XCFrameGrabberGamma;
extern const ModelOptionDef OPT_FrameGrabberShutter;
extern const ModelOptionDef OPT_FrameGrabberGain;
extern const ModelOptionDef OPT_FrameGrabberStreaming;
extern const ModelOptionDef OPT_FrameGrabberAudioVolume;
extern const ModelOptionDef OPT_FrameGrabberAudioBalance;
extern const ModelOptionDef OPT_FrameGrabberAudioBass;
extern const ModelOptionDef OPT_FrameGrabberAudioTreble;
extern const ModelOptionDef OPT_FrameGrabberAudioMute;
extern const ModelOptionDef OPT_FrameGrabberAudioLoudness;
extern const ModelOptionDef OPT_FrameGrabberAutoWhiteBal;
extern const ModelOptionDef OPT_FrameGrabberDoWhiteBal;
extern const ModelOptionDef OPT_FrameGrabberAutoGain;
extern const ModelOptionDef OPT_FrameGrabberHFlip;
extern const ModelOptionDef OPT_FrameGrabberVFlip;
extern const ModelOptionDef OPT_FrameGrabberHueAuto;

extern const ModelOptionDef OPT_FrameGrabberFocusAuto;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalTempAuto;
extern const ModelOptionDef OPT_FrameGrabberWhiteBalTemp;
extern const ModelOptionDef OPT_FrameGrabberPowerLineFreq;
extern const ModelOptionDef OPT_FrameGrabberBacklightComp;
extern const ModelOptionDef OPT_FrameGrabberExposureMode;
extern const ModelOptionDef OPT_FrameGrabberExposureAutoPri;
extern const ModelOptionDef OPT_FrameGrabberFocus;
extern const ModelOptionDef OPT_FrameGrabberZoom;
extern const ModelOptionDef OPT_FrameGrabberExposureAbs;

//! Aliases for various cameras:
extern const ModelOptionDef OPT_ALIAScamMacbook;
extern const ModelOptionDef OPT_ALIAScamBttv;
extern const ModelOptionDef OPT_ALIAScamiSight;
extern const ModelOptionDef OPT_ALIAScamLifeCam;
extern const ModelOptionDef OPT_ALIAScamLifeCamManual;
extern const ModelOptionDef OPT_ALIAScamLifeCamNX6000;
extern const ModelOptionDef OPT_ALIAScamHPpremAF;
extern const ModelOptionDef OPT_ALIAScamHPpremAFmanual;
extern const ModelOptionDef OPT_ALIAScamC910;
extern const ModelOptionDef OPT_ALIAScamC910manual;
extern const ModelOptionDef OPT_ALIAScamC910turntable;
//@}

//! Command-line options for VCC4
//@{

//! VCC4 Pan/Tilt Camera Options
extern const ModelOptionCateg MOC_VCC4;

extern const ModelOptionDef OPT_VCC4serialDevice;
extern const ModelOptionDef OPT_VCC4unitNo;
extern const ModelOptionDef OPT_VCC4pulseRatio;
extern const ModelOptionDef OPT_VCC4maxAngle;
//@}

//! Command-line options for AudioMixer
//@{
extern const ModelOptionDef OPT_AudioMixerDevice;
extern const ModelOptionDef OPT_AudioMixerLineIn;
extern const ModelOptionDef OPT_AudioMixerCdIn;
extern const ModelOptionDef OPT_AudioMixerMicIn;
//@}

//! Command-line options for AudioGrabber
//@{
extern const ModelOptionDef OPT_AudioGrabberDevice;
extern const ModelOptionDef OPT_AudioGrabberBits;
extern const ModelOptionDef OPT_AudioGrabberFreq;
extern const ModelOptionDef OPT_AudioGrabberBufSamples;
extern const ModelOptionDef OPT_AudioGrabberChans;
extern const ModelOptionDef OPT_AudioGrabberInterleaved;
//@}

//! Command-line options for DiskDataStream
//@{
extern const ModelOptionDef OPT_DiskDataStreamSavePath;
extern const ModelOptionDef OPT_DiskDataStreamUseMmap;
extern const ModelOptionDef OPT_DiskDataStreamNumThreads;
extern const ModelOptionDef OPT_DiskDataStreamSleepUsecs;
extern const ModelOptionDef OPT_DiskDataStreamSavePeriod;
//@}

//! Command-line options for XC cameralink HD camera
//@{
extern const ModelOptionDef OPT_XCFormatFileName;
//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_DEVICEOPTS_H_DEFINED
