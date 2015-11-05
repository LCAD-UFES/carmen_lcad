/*!@file Devices/DeviceOpts.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/DeviceOpts.C $
// $Id: DeviceOpts.C 15444 2012-12-01 04:06:55Z kai $
//

#ifndef DEVICES_DEVICEOPTS_C_DEFINED
#define DEVICES_DEVICEOPTS_C_DEFINED

#include "Devices/DeviceOpts.H"

#include "Component/ModelOptionDef.H"
#include "Image/Dims.H"
#include "Transport/TransportOpts.H"
#include "Video/VideoFormat.H"

const ModelOptionCateg MOC_AUDIO = {
  MOC_SORTPRI_2,   "Audio-Related Options" };

// Format here is:
//
// { MODOPT_TYPE, "name", &MOC_CATEG, OPTEXP_CORE,
//   "description of what option does",
//   "long option name", 'short option name', "valid values", "default value" }
//

// alternatively, for MODOPT_ALIAS option types, format is:
//
// { MODOPT_ALIAS, "", &MOC_ALIAS, OPTEXP_CORE,
//   "description of what alias does",
//   "long option name", 'short option name', "", "list of options" }
//

// NOTE: do not change the default value of any existing option unless
// you really know what you are doing!  Many components will determine
// their default behavior from that default value, so you may break
// lots of executables if you change it.

// #################### Video framegrabbing options

const ModelOptionCateg MOC_FRAMEGRAB = {
  MOC_SORTPRI_2, "Video FrameGrabber-Related Options" };

// Used by: FrameGrabberConfigurator
const ModelOptionDef OPT_FrameGrabberType =
  { MODOPT_ARG_STRING, "FrameGrabberType", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber type name",
    "fg-type", '\0', "<None|V4L|V4L2|1394|XC|KINECT|BB2>", "None" };

// Used by: FrameGrabberConfigurator
const ModelOptionDef OPT_DeinterlacerType =
  { MODOPT_ARG_STRING, "DeinterlacerType", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Deinterlacer type name",
    "deinterlacer", '\0', "<None|Bob>", "None" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberDevice =
  { MODOPT_ARG_STRING, "FrameGrabberDevice", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber device name",
    "framegrabber-dev", '\0', "<file>", "/dev/video0" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberChannel =
  { MODOPT_ARG(int), "FrameGrabberChannel", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber input channel (typically, 0=TV, 1=Composite)",
    "framegrabber-chan", '\0', "<int>", "1" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberSubChan =
  { MODOPT_ARG(int), "FrameGrabberSubChan", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber input sub-channel",
    "framegrabber-subchan", '\0', "<int>", "0" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberDims =
  { MODOPT_ARG(Dims), "FrameGrabberDims", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber grab image dimensions",
    "framegrabber-dims", '\0', "<width>x<height>", "320x240" };

// Used by: XCgrabber
const ModelOptionDef OPT_FrameGrabberOffset =
  { MODOPT_ARG(Dims), "FrameGrabberOffset", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber grab image offset (top-left corner) within the device's full "
    "frame dims. This is used by XCgrabber to capture a subwindow of the "
    "camera's native frames, by specifying a non-zero offset and dims smaller "
    "than the camera's native dims.",
    "framegrabber-offset", '\0', "<xoff>x<yoff>", "0x0" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberMode =
  { MODOPT_ARG(VideoFormat), "FrameGrabberMode", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber grab mode",
    "framegrabber-mode", '\0', "<GREY|RAW|RGB555|RGB565|RGB24|RGB32|YUV24|YUYV|"
    "UYVY|YUV444|YUV422|YUV411|YUV420|YUV410|YUV444P|YUV422P|YUV411P|YUV420P|"
    "YUV410P|HM12|MJPEG|BAYER_GB|BAYER_GR|"
    "BAYER_RG|BAYER_BG|BAYER_GB12|BAYER_GR12|BAYER_RG12|BAYER_BG12>",
    "YUV422" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberByteSwap =
  { MODOPT_FLAG, "FrameGrabberByteSwap", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber use byte-swapping on grabbed data",
    "framegrabber-bswap", '\0', "", "false" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberFPS =
  { MODOPT_ARG(float), "FrameGrabberFPS", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber grabbing frames per seconds",
    "framegrabber-fps", '\0', "<float>", "30.0" };

// Used by: V4Lgrabber/IEEE1394grabber
const ModelOptionDef OPT_FrameGrabberNbuf =
  { MODOPT_ARG(int), "FrameGrabberNbuf", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Framegrabber number fo grab buffers",
    "framegrabber-nbuf", '\0', "<int>", "2" };

// Used by: XC HD Camera
const ModelOptionDef OPT_XCFormatFileName =
  { MODOPT_ARG_STRING, "XCGrabberFormatFile", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "XC camera configure format file",
    "xc-format", '\0', "<file>", ""
  };

const ModelOptionDef OPT_FrameGrabberBrightness =
  { MODOPT_ARG(int), "FrameGrabberBrightness", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Brightness of the grabbed picture",
    "framegrabber-brightness", '\0', "<int>", "32768" };

const ModelOptionDef OPT_FrameGrabberColour =
  { MODOPT_ARG(int), "FrameGrabberColour", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Brightness of the grabbed picture",
    "framegrabber-colour", '\0', "<int>", "32768" };

const ModelOptionDef OPT_FrameGrabberHue =
  { MODOPT_ARG(int), "FrameGrabberHue", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Hue of the grabbed picture",
    "framegrabber-hue", '\0', "<int>", "32768" };

const ModelOptionDef OPT_FrameGrabberSaturation =
  { MODOPT_ARG(int), "FrameGrabberSaturation", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Saturation of the grabbed picture",
    "framegrabber-saturation", '\0', "<int>", "90" };

const ModelOptionDef OPT_FrameGrabberContrast =
  { MODOPT_ARG(int), "FrameGrabberContrast", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Contrast of the grabbed picture",
    "framegrabber-contrast", '\0', "<int>", "32768" };

const ModelOptionDef OPT_FrameGrabberWhiteness =
  { MODOPT_ARG(int), "FrameGrabberWhiteness", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Whiteness of the grabbed picture",
    "framegrabber-whiteness", '\0', "<int>", "32768" };

const ModelOptionDef OPT_FrameGrabberExposure =
  { MODOPT_ARG(int), "FrameGrabberExposure", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Exposure of the frame grabber",
    "framegrabber-exposure", '\0', "<int>", "511" };

const ModelOptionDef OPT_FrameGrabberSharpness =
  { MODOPT_ARG(int), "FrameGrabberSharpness", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Sharpness of the grabbed picture",
    "framegrabber-sharpness", '\0', "<int>", "80" };

const ModelOptionDef OPT_FrameGrabberWhiteBalBU =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalBU", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "B/U white balance of the grabbed picture",
    "framegrabber-whitebalBU", '\0', "<int>", "95" };

const ModelOptionDef OPT_FrameGrabberWhiteBalRV =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalRV", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "R/V white balance of the grabbed picture",
    "framegrabber-whitebalRV", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberWhiteBalTargetR =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalTargetR", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "target R value for white balance of the grabbed picture",
    "xcgrabber-whiteTargetR", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberWhiteBalTargetG =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalTargetG", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "target G value for white balance of the grabbed picture",
    "xcgrabber-whiteTargetG", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberWhiteBalTargetB =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalTargetB", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "target B value for white balance of the grabbed picture",
    "xcgrabber-whiteTargetB", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberWhiteBalReferenceR =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalReferenceR", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "target R value for white balance of the grabbed picture",
    "xcgrabber-whiteReferenceR", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberWhiteBalReferenceG =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalReferenceG", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "target G value for white balance of the grabbed picture",
    "xcgrabber-whiteReferenceG", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberWhiteBalReferenceB =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalReferenceB", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "target B value for white balance of the grabbed picture",
    "xcgrabber-whiteReferenceB", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberGamma =
  { MODOPT_ARG(int), "FrameGrabberGamma", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Gamma correction of the frame grabber",
    "framegrabber-gamma", '\0', "<int>", "1" };

const ModelOptionDef OPT_XCFrameGrabberGamma =
  { MODOPT_ARG(float), "XCFrameGrabberGamma", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Gamma correction of the frame grabber",
    "xc-gamma", '\0', "<float>", "1.0" };

const ModelOptionDef OPT_FrameGrabberShutter =
  { MODOPT_ARG(int), "FrameGrabberShutter", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Shutter speed of the frame grabber",
    "framegrabber-shutter", '\0', "<int>", "6" };

const ModelOptionDef OPT_FrameGrabberGain =
  { MODOPT_ARG(int), "FrameGrabberGain", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Gain of the frame grabber",
    "framegrabber-gain", '\0', "<int>", "87" };

const ModelOptionDef OPT_FrameGrabberStreaming =
  { MODOPT_FLAG, "FrameGrabberStreaming", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Turn streaming mode on/off for frame-grabbers used through the "
    "generic FrameIstream interface. Compared with single-frame mode, "
    "streaming mode gives higher frame rates, but also requires that "
    "the application be able to keep up with the frame rate; otherwise "
    "the grabber will start to lose frames. In general, if the program "
    "can operate at a consistently high frame rate, then streaming is "
    "the way to go, but if the program requires lengthy per-frame "
    "processing, or if the processing time varies widely from one "
    "frame to the next, then single-frame mode is the way to go.",
    "framegrabber-streaming", '\0', "", "true" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAudioVolume =
  { MODOPT_ARG(int), "FrameGrabberAudioVolume", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Overall audio volume. Note some drivers also provide an OSS or ALSA mixer interface",
    "framegrabber-audio-vol", '\0', "<int>", "100" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAudioBalance =
  { MODOPT_ARG(int), "FrameGrabberAudioBalance", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Audio stereo balance. Minimum corresponds to all the way left, maximum to right",
    "framegrabber-audio-bal", '\0', "<int>", "0" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAudioBass =
  { MODOPT_ARG(int), "FrameGrabberAudioBass", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Audio bass adjustment",
    "framegrabber-audio-bass", '\0', "<int>", "0" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAudioTreble =
  { MODOPT_ARG(int), "FrameGrabberAudioTreble", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Audio treble adjustment",
    "framegrabber-audio-treb", '\0', "<int>", "0" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAudioMute =
  { MODOPT_FLAG, "FrameGrabberAudioMute", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Mute audio, i. e. set the volume to zero, however without affecting the audio volume. Like ALSA drivers, "
    "V4L2 drivers must mute at load time to avoid excessive noise. Actually the entire device should be reset "
    "to a low power consumption state",
    "framegrabber-audio-mute", '\0', "", "false" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAudioLoudness =
  { MODOPT_FLAG, "FrameGrabberAudioLoudness", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Loudness mode (bass boost)",
    "framegrabber-audio-loudness", '\0', "", "false" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAutoWhiteBal =
  { MODOPT_FLAG, "FrameGrabberAutoWhiteBal", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Automatic white balance",
    "framegrabber-auto-whitebal", '\0', "", "true" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberDoWhiteBal =
  { MODOPT_FLAG, "FrameGrabberDoWhiteBal", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "This is an action control. When set (the value is ignored), the device will do a white balance and "
    "then hold the current setting. Contrast this with the boolean --framegrabber-auto-whitebal, which, "
    "when activated, keeps adjusting the white balance.",
    "framegrabber-do-whitebal", '\0', "", "false" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberAutoGain =
  { MODOPT_FLAG, "FrameGrabberAutoGain", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Automatic gain/exposure control",
    "framegrabber-autogain", '\0', "", "true" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberHFlip =
  { MODOPT_FLAG, "FrameGrabberHFlip", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Mirror the picture horizontally",
    "framegrabber-hflip", '\0', "", "false" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberVFlip =
  { MODOPT_FLAG, "FrameGrabberVFlip", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Mirror the picture vertically",
    "framegrabber-vflip", '\0', "", "false" };

// V4L2 control
const ModelOptionDef OPT_FrameGrabberHueAuto =
  { MODOPT_FLAG, "FrameGrabberHueAuto", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Enables automatic hue control by the device. The effect of setting --framegrabber-hue while automatic hue "
    "control is enabled is undefined, drivers should ignore such request",
    "framegrabber-hue-auto", '\0', "", "true" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberFocusAuto =
  { MODOPT_FLAG, "FrameGrabberFocusAuto", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Whether to use built-in auto-focus",
    "framegrabber-autofocus", '\0', "", "true" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberWhiteBalTempAuto =
  { MODOPT_FLAG, "FrameGrabberWhiteBalTempAuto", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Whether to use built-in auto white balance temperature",
    "framegrabber-auto-whitebaltemp", '\0', "", "true" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberWhiteBalTemp =
  { MODOPT_ARG(int), "FrameGrabberWhiteBalTemp", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Manual white balance temperature (make sure auto white balance is disabled)",
    "framegrabber-whitebaltemp", '\0', "<int>", "4500" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberPowerLineFreq =
  { MODOPT_ARG(int), "FrameGrabberPowerLineFreq", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Power line frequency compensation",
    "framegrabber-powerlinefreq", '\0', "<int>", "2" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberBacklightComp =
  { MODOPT_ARG(int), "FrameGrabberBacklightComp", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Backlight compensation",
    "framegrabber-backlight", '\0', "<int>", "0" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberExposureMode =
  { MODOPT_ARG(int), "FrameGrabberExposureAuto", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Exposure control mode",
    "framegrabber-exposuremode", '\0', "<int>", "0" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberExposureAutoPri =
  { MODOPT_FLAG, "FrameGrabberExposureAutoPri", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Exposure auto priority control flag",
    "framegrabber-exposure-autopri", '\0', "", "false" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberFocus =
  { MODOPT_ARG(int), "FrameGrabberFocus", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Manual focus value (make sure you disabled auto-focus)",
    "framegrabber-focus", '\0', "<int>", "10" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberZoom =
  { MODOPT_ARG(int), "FrameGrabberZoom", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Manual zoom value",
    "framegrabber-zoom", '\0', "<int>", "0" };

// V4L2 extended control
const ModelOptionDef OPT_FrameGrabberExposureAbs =
  { MODOPT_ARG(int), "FrameGrabberExposureAbs", &MOC_FRAMEGRAB, OPTEXP_CORE,
    "Exposure (absolute) of the frame grabber (make sure you selected manual exposure mode). Note that on some cameras "
    "only a few discrete values in the range are valid; for example, 9, 19, 39, 78, 156, 312 and similar for "
    "the Microsoft Lifecam Cinema 720p",
    "framegrabber-exposure-abs", '\0', "<int>", "511" };

// Framegrabber aliases for various cameras:

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamMacbook =
  { MODOPT_ALIAS, "ALIAScamMacbook", &MOC_ALIAS, OPTEXP_CORE,
    "Use builtin camera of Apple Macbook (Core 2 Duo version). This only "
    "works with V4L2 and uses the uvcvideo kernel driver. Only resolutions "
    "supported seem to be 320x240 and 640x480 (both at 30.00 fps). The "
    "camera seems to support setting the gamma (which under V4L2 currently "
    "is the same as whiteness), but apparently this has no effect and the "
    "value remains at its default",
    "camera-macbook", '\0', "",
    "--framegrabber-mode=UYVY --framegrabber-brightness=100 "
    "--noframegrabber-bswap --framegrabber-saturation=5 "
    "--framegrabber-chan=0 --framegrabber-whiteness=150" };

// Instantiated by V4Lgrabber and V4L2grabber
const ModelOptionDef OPT_ALIAScamBttv =
  { MODOPT_ALIAS, "ALIAScamBttv", &MOC_ALIAS, OPTEXP_CORE,
    "Use camera connected to a BTTV framegrabber. This both "
    "works with V4L and V4L2. Typically, these boards support many "
    "resolutions and modes, as listed by the V4L2 driver",
    "camera-bttv", '\0', "",
    "--framegrabber-mode=RGB24 --framegrabber-brightness=32768 "
    "--framegrabber-bswap --framegrabber-saturation=32768 "
    "--framegrabber-contrast=32768 --framegrabber-hue=32768" };

// Instantiated by IEEE1394grabber
const ModelOptionDef OPT_ALIAScamiSight =
  { MODOPT_ALIAS, "ALIAScamiSight", &MOC_ALIAS, OPTEXP_CORE,
    "Use Apple iSight camera (external IEEE1394 model, not the USB model "
    "integrated into Macbook laptops). Surprsingly, even though the "
    "camera seems to stream at 30fps, the video looks like it may only "
    "capture at 15fps and then send each frame twice",
    "camera-isight", '\0', "",
    "--framegrabber-brightness=127 --framegrabber-hue=180" };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamLifeCam =
  { MODOPT_ALIAS, "ALIAScamLifeCam", &MOC_ALIAS, OPTEXP_CORE,
    "Use Microsoft LifeCam 1280x720 USB camera.",
    "camera-lifecam", '\0', "",
    "--framegrabber-dims=1280x720 --framegrabber-mode=YUYV --noframegrabber-bswap "
    "--framegrabber-brightness=133 --framegrabber-contrast=5 --framegrabber-saturation=83 "
    "--framegrabber-auto-whitebaltemp=true --framegrabber-powerlinefreq=2 --framegrabber-whitebaltemp=4500 "
    "--framegrabber-sharpness=25 --framegrabber-backlight=0 --framegrabber-exposuremode=3 "
    "--framegrabber-exposure-abs=156 --framegrabber-focus=0 --framegrabber-autofocus=true --framegrabber-zoom=0"
    // note: some values will be ignored, like the focus value since we use autofocus. We set them here to the camera's
    // default, in case autofocus gets later turned off, the focus value will be a reasonable one.
  };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamLifeCamManual =
  { MODOPT_ALIAS, "ALIAScamLifeCamManual", &MOC_ALIAS, OPTEXP_CORE,
    "Use Microsoft LifeCam 1280x720 USB camera with most settings set to manual (focus, exposure, "
    "white balance temperature, etc). Useful for robotics applications. Note: exposure-abs values are funky on "
    "this camera. 9, 19, 39, 78, 156, 312 work, probably some other vals too but in-betweens look over-exposed.",
    "camera-lifecam-manual", '\0', "",
    "--framegrabber-dims=1280x720 --framegrabber-mode=YUYV --noframegrabber-bswap "
    "--framegrabber-brightness=130 --framegrabber-contrast=5 --framegrabber-saturation=83 "
    "--framegrabber-auto-whitebaltemp=false --framegrabber-powerlinefreq=0 --framegrabber-whitebaltemp=3000 "
    "--framegrabber-sharpness=25 --framegrabber-backlight=0 --framegrabber-exposuremode=1 "
    "--framegrabber-exposure-abs=78 --framegrabber-focus=10 --framegrabber-autofocus=false --framegrabber-zoom=0"
 };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamLifeCamNX6000 =
  { MODOPT_ALIAS, "ALIASLifeCamNX6000", &MOC_ALIAS, OPTEXP_CORE,
    "Use Microsoft LifeCam NX-6000 2MP USB camera.",
    "camera-lifecam-nx6000", '\0', "",
    "--framegrabber-dims=1600x1200 --framegrabber-mode=MJPEG"
  };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamHPpremAF =
  { MODOPT_ALIAS, "ALIAScamHPpremAF", &MOC_ALIAS, OPTEXP_CORE,
    "Use H.P. Premium Autofocus (KQ245AA) 960x720 USB camera.",
    "camera-hpprem", '\0', "",
    "--framegrabber-dims=960x720 --framegrabber-mode=YUYV --noframegrabber-bswap "
    "--framegrabber-brightness=0 --framegrabber-contrast=32 --framegrabber-saturation=64 --framegrabber-hue=0 "
    "--framegrabber-auto-whitebaltemp=true --framegrabber-gamma=110 --framegrabber-gain=0 "
    "--framegrabber-powerlinefreq=2 --framegrabber-exposuremode=3 --framegrabber-exposure-abs=300 "
    "--framegrabber-exposure-autopri=false --framegrabber-autofocus=true --framegrabber-focus=1"
  };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamHPpremAFmanual =
  { MODOPT_ALIAS, "ALIAScamHPpremAFmanual", &MOC_ALIAS, OPTEXP_CORE,
    "Use H.P. Premium Autofocus (KQ245AA) 960x720 USB camera with most settings set to manual (focus, exposure, "
    "white balance temperature, etc). Useful for robotics applications.",
    "camera-hpprem-manual", '\0', "",
    "--framegrabber-dims=960x720 --framegrabber-mode=YUYV --noframegrabber-bswap "
    "--framegrabber-brightness=0 --framegrabber-contrast=32 --framegrabber-saturation=64 --framegrabber-hue=0 "
    "--framegrabber-auto-whitebaltemp=false --framegrabber-gamma=110 --framegrabber-gain=0 "
    "--framegrabber-powerlinefreq=0 --framegrabber-exposuremode=1 --framegrabber-exposure-abs=800 "
    "--framegrabber-exposure-autopri=true --framegrabber-autofocus=false --framegrabber-focus=15"
 };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamC910 =
  { MODOPT_ALIAS, "ALIAScamC910", &MOC_ALIAS, OPTEXP_CORE,
    "Use Logitech C910 5MP USB camera.",
    "camera-c910", '\0', "",
    "--framegrabber-dims=2592x1944 --framegrabber-mode=YUYV --noframegrabber-bswap "
    "--framegrabber-brightness=128 --framegrabber-contrast=32 --framegrabber-saturation=32 "
    "--framegrabber-auto-whitebaltemp=true --framegrabber-gain=64 --framegrabber-powerlinefreq=2 "
    "--framegrabber-whitebaltemp=4000 --framegrabber-sharpness=72 --framegrabber-backlight=1 "
    "--framegrabber-exposuremode=3 --framegrabber-exposure-abs=166 --framegrabber-exposure-autopri "
    "--framegrabber-focus=68 --framegrabber-autofocus=true --framegrabber-zoom=1"
    // note: some values will be ignored, like the focus value since we use autofocus. We set them here to the camera's
    // default, in case autofocus gets later turned off, the focus value will be a reasonable one.
  };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamC910manual =
  { MODOPT_ALIAS, "ALIAScamC910manual", &MOC_ALIAS, OPTEXP_CORE,
    "Use Logitech C910 5MP USB camera with most settings set to manual (focus, exposure, "
    "white balance temperature, etc). Useful for robotics applications. Note: focus is in steps of 17.",
    "camera-c910-manual", '\0', "",
    "--framegrabber-dims=2592x1944 --framegrabber-mode=YUYV --noframegrabber-bswap "
    "--framegrabber-brightness=128 --framegrabber-contrast=32 --framegrabber-saturation=32 "
    "--framegrabber-auto-whitebaltemp=false --framegrabber-gain=64 --framegrabber-powerlinefreq=0 "
    "--framegrabber-whitebaltemp=4000 --framegrabber-sharpness=72 --framegrabber-backlight=0 "
    "--framegrabber-exposuremode=1 --framegrabber-exposure-abs=166 --noframegrabber-exposure-autopri "
    "--framegrabber-focus=68 --framegrabber-autofocus=false --framegrabber-zoom=1"
 };

// Instantiated by V4L2grabber
const ModelOptionDef OPT_ALIAScamC910turntable =
  { MODOPT_ALIAS, "ALIAScamC910turntable", &MOC_ALIAS, OPTEXP_CORE,
    "Use Logitech C910 5MP USB camera with most settings set to manual (focus, exposure, "
    "white balance temperature, etc) and tuned for the iLab 11-camera turntable rig. Note: focus is "
    "in steps of 17.",
    "camera-c910-turntable", '\0', "",
    // DO NOT MODIFY SETTINGS HERE! We rely on them for consistency of our turntable shots.
    // You can modify ALIAScamC910manual above if you find better defaults for it.
    "--framegrabber-dims=2592x1944 --framegrabber-mode=YUYV --noframegrabber-bswap "
    "--framegrabber-brightness=128 --framegrabber-contrast=32 --framegrabber-saturation=32 "
    "--framegrabber-auto-whitebaltemp=false --framegrabber-gain=64 --framegrabber-powerlinefreq=0 "
    "--framegrabber-whitebaltemp=4000 --framegrabber-sharpness=72 --framegrabber-backlight=0 "
    "--framegrabber-exposuremode=1 --framegrabber-exposure-abs=166 --noframegrabber-exposure-autopri "
    "--framegrabber-focus=68 --framegrabber-autofocus=false --framegrabber-zoom=1"
 };

// #################### Special hardware options

const ModelOptionCateg MOC_VCC4 = {
  MOC_SORTPRI_2, "VCC4 Pan/Tilt Camera Options" };

// Used by: VCC4
const ModelOptionDef OPT_VCC4serialDevice =
  { MODOPT_ARG_STRING, "VCC4serialDevice", &MOC_VCC4, OPTEXP_CORE,
    "the serial device for the VCC4 camera",
    "vcc4-serial-device", '\0', "<dev>", "/dev/usb/tts/0" };

// Used by: VCC4
const ModelOptionDef OPT_VCC4unitNo =
  { MODOPT_ARG(int), "VCC4unitNo", &MOC_VCC4, OPTEXP_CORE,
    "VCC4 unit number",
    "vcc4-unit-no", '\0', "<0..9>", "0" };

// Used by: VCC4
const ModelOptionDef OPT_VCC4pulseRatio =
  { MODOPT_ARG(float), "VCC4pulseRatio", &MOC_VCC4, OPTEXP_CORE,
    "VCC4 pulse ratio, or 0.0 to query it from the VCC4 unit",
    "vcc4-pulse-ratio", '\0', "<float>", "0.0" };

// Used by: VCC4
const ModelOptionDef OPT_VCC4maxAngle =
  { MODOPT_ARG(float), "VCC4maxAngle", &MOC_VCC4, OPTEXP_CORE,
    "VCC4 max angle, or 0.0 to query it from the VCC4 unit",
    "vcc4-max-angle", '\0', "<float>", "0.0" };


// #################### Audio recording options
// Used by: AudioMixer
const ModelOptionDef OPT_AudioMixerDevice =
  { MODOPT_ARG_STRING, "AudioMixerDevice", &MOC_AUDIO, OPTEXP_CORE,
    "Audio mixer device name",
    "audio-mix-dev", '\0', "<file>", "/dev/mixer" };

// Used by: AudioMixer
const ModelOptionDef OPT_AudioMixerLineIn =
  { MODOPT_FLAG, "AudioMixerLineIn", &MOC_AUDIO, OPTEXP_CORE,
    "Audio mixer Line input",
    "audio-mix-linein", '\0', "", "true" };

// Used by: AudioMixer
const ModelOptionDef OPT_AudioMixerCdIn =
  { MODOPT_FLAG, "AudioMixerCdIn", &MOC_AUDIO, OPTEXP_CORE,
    "Audio mixer CD input",
    "audio-mix-cdin", '\0', "", "false" };

// Used by: AudioMixer
const ModelOptionDef OPT_AudioMixerMicIn =
  { MODOPT_FLAG, "AudioMixerMicIn", &MOC_AUDIO, OPTEXP_CORE,
    "Audio mixer Mic input",
    "audio-mix-micin", '\0', "", "false" };


// Used by: AudioGrabber
const ModelOptionDef OPT_AudioGrabberDevice =
  { MODOPT_ARG_STRING, "AudioGrabberDevice", &MOC_AUDIO, OPTEXP_CORE,
    "Audio grabber device name",
    "audio-grab-dev", '\0', "<file>", "/dev/dsp" };

// Used by: AudioGrabber
const ModelOptionDef OPT_AudioGrabberBits =
  { MODOPT_ARG(unsigned int), "AudioGrabberBits", &MOC_AUDIO, OPTEXP_CORE,
    "Audio grabber bits per sample",
    "audio-grab-bits", '\0', "<8|16>", "8" };

// Used by: AudioGrabber
const ModelOptionDef OPT_AudioGrabberFreq =
  { MODOPT_ARG(unsigned int), "AudioGrabberFreq", &MOC_AUDIO, OPTEXP_CORE,
    "Audio grabber sampling frequency (Hz)",
    "audio-grab-freq", '\0', "<int>", "44100" };

// Used by: AudioGrabber
const ModelOptionDef OPT_AudioGrabberBufSamples =
  { MODOPT_ARG(unsigned int), "AudioGrabberBufSamples", &MOC_AUDIO, OPTEXP_CORE,
    "Audio grabber buffer length (in samples)",
    "audio-grab-bufsamp", '\0', "<int>", "256" };

// Used by: AudioGrabber
const ModelOptionDef OPT_AudioGrabberChans =
  { MODOPT_ARG(unsigned int), "AudioGrabberChans", &MOC_AUDIO, OPTEXP_CORE,
    "Audio grabber number of channels to record",
    "audio-grab-chans", '\0', "<int>", "1" };

// Used by: AudioGrabber
const ModelOptionDef OPT_AudioGrabberInterleaved =
  { MODOPT_FLAG, "AudioGrabberInterleaved", &MOC_AUDIO, OPTEXP_CORE,
    "True if channel data is interleaved (as opposed to chunked)",
    "audio-grab-interleaved", '\0', "<true|false>", "false" };


// Used by: DiskDataStream
const ModelOptionDef OPT_DiskDataStreamSavePath =
  { MODOPT_ARG_STRING, "DiskDataStreamSavePath", &MOC_OUTPUT, OPTEXP_CORE,
    "Comma-separated list of filename stems for where to save files; "
    "e.g., 'dir0/pfx0,dir1/pfx1' will save even-numbered files with "
    "the prefix 'dir0/pfx0' and will save odd-numbered files with "
    "the prefix 'dir1/pfx1'",
    "datastream-save-path", '\0', "<stem1,stem2,...,stemN>", "./" };

// Used by: DiskDataStream
const ModelOptionDef OPT_DiskDataStreamUseMmap =
  { MODOPT_ARG(bool), "DiskDataStreamUseMmap", &MOC_OUTPUT, OPTEXP_CORE,
    "Whether to use mmap() instead of write() to stream data to disk. The"
    "default is to use mmap(), which may be significantly faster, but may"
    "be somewhat less portable",
    "datastream-use-mmap", '\0', "<true|false>", "true" };

// Used by: DiskDataStream
const ModelOptionDef OPT_DiskDataStreamNumThreads =
  { MODOPT_ARG(uint), "DiskDataStreamNumThreads", &MOC_OUTPUT, OPTEXP_CORE,
    "How many worker threads to use for streaming files to disk.",
    "datastream-num-threads", '\0', "<uint>", "1" };

// Used by: DiskDataStream
const ModelOptionDef OPT_DiskDataStreamSleepUsecs =
  { MODOPT_ARG(uint), "DiskDataStreamSleepUsecs", &MOC_OUTPUT, OPTEXP_CORE,
    "How many microseconds each worker thread should sleep "
    "after writing each frame to disk.",
    "datastream-sleep-usecs", '\0', "<uint>", "0" };

// Used by: DiskDataStream
const ModelOptionDef OPT_DiskDataStreamSavePeriod =
  { MODOPT_ARG(uint), "DiskDataStreamSavePeriod", &MOC_OUTPUT, OPTEXP_CORE,
    "How often to save frames (1 = save every frame; N = save every "
    "Nth frame).",
    "datastream-save-period", '\0', "<uint>", "1" };

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_DEVICEOPTS_C_DEFINED
