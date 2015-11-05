/*!@file Devices/V4L2grabber.C Interface with a video4linux2 frame grabber */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/V4L2grabber.C $
// $Id: V4L2grabber.C 15430 2012-11-06 05:12:00Z kai $
//

#ifdef HAVE_LINUX_VIDEODEV2_H

#include "Devices/V4L2grabber.H"

#include "Component/OptionManager.H" // for option alias requests
#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Image/ColorOps.H"
#include "Raster/GenericFrame.H"
#include "Util/Assert.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"

#include <cerrno>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

//! for id-logging; see log.H:
#define MYLOGID itsFd

namespace
{
  struct V4L2Palette
  {
    VideoFormat vidformat;
    uint32 v4l2format; // V4L2_PIX_FMT_...
  };

  const V4L2Palette v4l2tab[VIDFMT_AUTO] =
    {
      { VIDFMT_GREY    , V4L2_PIX_FMT_GREY },
      { VIDFMT_RGB555  , V4L2_PIX_FMT_RGB555 },
      { VIDFMT_RGB565  , V4L2_PIX_FMT_RGB565 },
      { VIDFMT_RGB24   , V4L2_PIX_FMT_BGR24 },
      { VIDFMT_RGB32   , V4L2_PIX_FMT_BGR32 },
      { VIDFMT_YUYV    , V4L2_PIX_FMT_YUYV },
      { VIDFMT_UYVY    , V4L2_PIX_FMT_UYVY },
      { VIDFMT_YUV422  , V4L2_PIX_FMT_UYVY },
      { VIDFMT_YUV411  , V4L2_PIX_FMT_Y41P },
      { VIDFMT_YUV420  , V4L2_PIX_FMT_YUV420 },
      { VIDFMT_YUV410  , V4L2_PIX_FMT_YUV410 },
      { VIDFMT_YUV422P , V4L2_PIX_FMT_YUV422P },
      { VIDFMT_YUV411P , V4L2_PIX_FMT_YUV411P },
      { VIDFMT_YUV420P , V4L2_PIX_FMT_YUV420 },
      { VIDFMT_YUV410P , V4L2_PIX_FMT_YUV410 },
      { VIDFMT_MJPEG   , V4L2_PIX_FMT_MJPEG },
      { VIDFMT_HM12    , V4L2_PIX_FMT_HM12 }
    };

  //! Mapping between our VideoFormat and V4L2 pixfmt:
  // See Video/VideoFormat.H
  uint32 VideoFormat_to_V4L2Format(const VideoFormat vidformat)
  {
    for (size_t i = 0; i < sizeof(v4l2tab) / sizeof(v4l2tab[0]); ++i)
      if (vidformat == v4l2tab[i].vidformat)
        return v4l2tab[i].v4l2format;

    // not found
    return 0xffffffff;
  }

  VideoFormat V4L2Format_to_VideoFormat(const uint32 v4l2format)
  {
    for (size_t i = 0; i < sizeof(v4l2tab) / sizeof(v4l2tab[0]); ++i)
      if (v4l2format == v4l2tab[i].v4l2format)
        return v4l2tab[i].vidformat;

    // not found
    return VIDFMT_AUTO;
  }

  int ioctl_nointr(int d, int req, void* mem)
  {
    int result = 0;
    do { result = ioctl(d, req, mem); }
    while ((result < 0) && (errno == EINTR));
    return result;
  }

  // mapping of V4L2 controls to our command-line options:
  struct ControlMapping {
    uint v4l2;
    const ModelOptionDef *opt;
  };

  // see here for definitions: http://v4l2spec.bytesex.org/spec/x542.htm
  ControlMapping controlmapping[] = {
    // standard controls, in V4L2_CID_USER_CLASS
    { V4L2_CID_BRIGHTNESS, &OPT_FrameGrabberBrightness },
    { V4L2_CID_CONTRAST, &OPT_FrameGrabberContrast },
    { V4L2_CID_SATURATION, &OPT_FrameGrabberSaturation },
    { V4L2_CID_HUE, &OPT_FrameGrabberHue },
    { V4L2_CID_AUDIO_VOLUME, &OPT_FrameGrabberAudioVolume },
    { V4L2_CID_AUDIO_BALANCE, &OPT_FrameGrabberAudioBalance },
    { V4L2_CID_AUDIO_BASS, &OPT_FrameGrabberAudioBass },
    { V4L2_CID_AUDIO_TREBLE, &OPT_FrameGrabberAudioTreble },
    { V4L2_CID_AUDIO_MUTE, &OPT_FrameGrabberAudioMute },
    { V4L2_CID_AUDIO_LOUDNESS, &OPT_FrameGrabberAudioLoudness },
    { V4L2_CID_AUTO_WHITE_BALANCE, &OPT_FrameGrabberWhiteBalTempAuto }, // ??
    { V4L2_CID_DO_WHITE_BALANCE, &OPT_FrameGrabberDoWhiteBal },
    { V4L2_CID_RED_BALANCE, &OPT_FrameGrabberWhiteBalBU }, //?
    { V4L2_CID_BLUE_BALANCE, &OPT_FrameGrabberWhiteBalRV }, //?
    { V4L2_CID_GAMMA, &OPT_FrameGrabberGamma },
    { V4L2_CID_EXPOSURE, &OPT_FrameGrabberExposure },
    { V4L2_CID_AUTOGAIN, &OPT_FrameGrabberAutoGain },
    { V4L2_CID_GAIN, &OPT_FrameGrabberGain },
    { V4L2_CID_HFLIP, &OPT_FrameGrabberHFlip },
    { V4L2_CID_VFLIP, &OPT_FrameGrabberVFlip },
    { V4L2_CID_POWER_LINE_FREQUENCY, &OPT_FrameGrabberPowerLineFreq },
    { V4L2_CID_HUE_AUTO, &OPT_FrameGrabberHueAuto },
    { V4L2_CID_WHITE_BALANCE_TEMPERATURE, &OPT_FrameGrabberWhiteBalTemp },
    { V4L2_CID_SHARPNESS, &OPT_FrameGrabberSharpness },
    { V4L2_CID_BACKLIGHT_COMPENSATION, &OPT_FrameGrabberBacklightComp },



    { V4L2_CID_FOCUS_ABSOLUTE, &OPT_FrameGrabberFocus },
    { V4L2_CID_EXPOSURE_ABSOLUTE, &OPT_FrameGrabberExposureAbs },
    { V4L2_CID_EXPOSURE_AUTO_PRIORITY, &OPT_FrameGrabberExposureAutoPri },
    { V4L2_CID_EXPOSURE_AUTO, &OPT_FrameGrabberExposureMode },
    { V4L2_CID_FOCUS_AUTO, &OPT_FrameGrabberFocusAuto },
    { V4L2_CID_ZOOM_ABSOLUTE, &OPT_FrameGrabberZoom },
    //Note: put all the NULL controls in the last
    //The controlmap only look up to first NULL and stop
    { V4L2_CID_CHROMA_AGC, NULL }, //FIXME
    { V4L2_CID_COLOR_KILLER, NULL }, //FIXME

#ifdef V4L2_CID_COLORFX
    { V4L2_CID_COLORFX, NULL }, //FIXME
#endif

#ifdef V4L2_CID_AUTOBRIGHTNESS
    { V4L2_CID_AUTOBRIGHTNESS, NULL }, //FIXME
#endif

#ifdef V4L2_CID_BAND_STOP_FILTER
    { V4L2_CID_BAND_STOP_FILTER, NULL }, //FIXME
#endif

    // extended controls in V4L2_CID_CAMERA_CLASS
    { V4L2_CID_PAN_RELATIVE, NULL }, //FIXME
    { V4L2_CID_TILT_RELATIVE, NULL }, //FIXME
    { V4L2_CID_PAN_RESET, NULL }, //FIXME
    { V4L2_CID_TILT_RESET, NULL }, //FIXME
    { V4L2_CID_PAN_ABSOLUTE, NULL }, //FIXME
    { V4L2_CID_TILT_ABSOLUTE, NULL }, //FIXME
    { V4L2_CID_FOCUS_RELATIVE, NULL }, //FIXME
    { V4L2_CID_ZOOM_RELATIVE, NULL }, //FIXME
    { V4L2_CID_ZOOM_CONTINUOUS, NULL }, //FIXME
    { V4L2_CID_PRIVACY, NULL }, //FIXME

    { 0, NULL } // keep this as the last entry
  };

} // namespace

// ######################################################################
V4L2grabber::V4L2grabber(OptionManager& mgr, const std::string& descrName,
                         const std::string& tagName, const ParamFlag flags) :
  FrameIstream(mgr, descrName, tagName),
  // NOTE that contrary to the common case, we may give (by default
  // value of 'flags') USE_MY_VAL here when we construct the
  // OModelParam objects; that means that we push our values into the
  // ModelManager as the new default values, rather than having our
  // param take its value from the ModelManager's default
  itsDevName(&OPT_FrameGrabberDevice, this, "/dev/video0", flags),
  itsChannel(&OPT_FrameGrabberChannel, this, 1, flags | ALLOW_ONLINE_CHANGES),
  itsDims(&OPT_FrameGrabberDims, this, Dims(320, 240), flags),
  itsGrabMode(&OPT_FrameGrabberMode, this, VIDFMT_RGB24, flags),
  itsByteSwap(&OPT_FrameGrabberByteSwap, this, true, flags),
  itsStreamingMode(&OPT_FrameGrabberStreaming, this),
  itsNbuf(&OPT_FrameGrabberNbuf, this, 4, flags),
	itsVflip(&OPT_FrameGrabberVFlip,this, false, flags),
	itsHflip(&OPT_FrameGrabberHFlip,this, false, flags),
  itsFd(-1),
  itsMmapBuf(NULL),
  itsMmapBufSize(NULL),
  itsReadBuf(),
  itsCurrentFrame(0),
  itsGrabbing(NULL),
  itsFrameTime(SimTime::ZERO()),
  itsListener(),
  itsStreamStarted(false),
  itsCanMMap(false),
  itsCanRW(false),
  itsOptionFlags(flags)
{
  openDevice();

  // request a bunch of camera aliases which work with V4L2:
  mgr.requestOptionAlias(&OPT_ALIAScamBttv);
  mgr.requestOptionAlias(&OPT_ALIAScamMacbook);
  mgr.requestOptionAlias(&OPT_ALIAScamLifeCam);
  mgr.requestOptionAlias(&OPT_ALIAScamLifeCamManual);
  mgr.requestOptionAlias(&OPT_ALIAScamLifeCamNX6000);
  mgr.requestOptionAlias(&OPT_ALIAScamHPpremAF);
  mgr.requestOptionAlias(&OPT_ALIAScamHPpremAFmanual);
  mgr.requestOptionAlias(&OPT_ALIAScamC910);
  mgr.requestOptionAlias(&OPT_ALIAScamC910manual);
  mgr.requestOptionAlias(&OPT_ALIAScamC910turntable);
}

// ######################################################################
void V4L2grabber::openDevice()
{
  if (itsFd != -1) closeDevice();

  itsFd = open(itsDevName.getVal().c_str(), O_RDWR);
  // since we will be called upon construction of the grabber with whichever default device name, do not LFATAL here if
  // the device does not open. We may be called again with a different device name if that name is changed at the
  // command line. In the end, during start1() we will check that the final selected device did open and that it has the
  // right capabilities (itsCanRW or itsCanMMap at the minimum):
  if (itsFd == -1) { PLERROR("Failed to open V4L2 device %s", itsDevName.getVal().c_str()); return; }

  LINFO("Opened V4L2 device named: %s", itsDevName.getVal().c_str());

  // get frame grabber capabilities:
  struct v4l2_capability vc; itsCanRW = false; itsCanMMap = false;
  if (ioctl_nointr(itsFd, VIDIOC_QUERYCAP, &vc) < 0) IDPLFATAL("Cannot get V4L2 device capabilities");
  IDLINFO("V4L2 kernel driver: %s", vc.driver);
  IDLINFO("FrameGrabber board name is: %s", vc.card);
  IDLINFO("FrameGrabber board bus info: %s", vc.bus_info);

  // note: these defs from /usr/include/linux/videodev2.h as of kernel 2.6.32, some may not be defined in older kernels
  // and should then be commented out:
  if (vc.capabilities & V4L2_CAP_VIDEO_CAPTURE) IDLINFO("    > Supports video capture");
  else IDLFATAL("Not a video capture device.");
  if (vc.capabilities & V4L2_CAP_VIDEO_OUTPUT) IDLINFO("    > Supports video output");
  if (vc.capabilities & V4L2_CAP_VIDEO_OVERLAY) IDLINFO("    > Supports video overlay");
  if (vc.capabilities & V4L2_CAP_VBI_CAPTURE) IDLINFO("    > Supports raw VBI capture");
  if (vc.capabilities & V4L2_CAP_VBI_OUTPUT) IDLINFO("    > Supports raw VBI output");
  if (vc.capabilities & V4L2_CAP_SLICED_VBI_CAPTURE) IDLINFO("    > Supports sliced VBI capture");
  if (vc.capabilities & V4L2_CAP_SLICED_VBI_OUTPUT) IDLINFO("    > Supports sliced VBI_OUTPUT");
  if (vc.capabilities & V4L2_CAP_RDS_CAPTURE) IDLINFO("    > Supports RDS capture");
  if (vc.capabilities & V4L2_CAP_VIDEO_OUTPUT_OVERLAY) IDLINFO("    > Supports video output overlay");
  //if (vc.capabilities & V4L2_CAP_HW_FREQ_SEEK) IDLINFO("    > Supports hardware frequency seek");
#ifdef V4L2_CAP_RDS_OUTPUT
  if (vc.capabilities & V4L2_CAP_RDS_OUTPUT) IDLINFO("    > Supports RDS output");
#endif
  if (vc.capabilities & V4L2_CAP_TUNER) IDLINFO("    > Has an RF tuner and/or modulator");
  if (vc.capabilities & V4L2_CAP_AUDIO) IDLINFO("    > Supports audio input and/or output");
  if (vc.capabilities & V4L2_CAP_RADIO) IDLINFO("    > Supports radio");
#ifdef V4L2_CAP_MODULATOR
  if (vc.capabilities & V4L2_CAP_MODULATOR) IDLINFO("    > Supports a modulator");
#endif

  if (vc.capabilities & V4L2_CAP_READWRITE) { IDLINFO("    > Supports read/write I/O method"); itsCanRW = true; }
  else itsCanRW = false;
  if (vc.capabilities & V4L2_CAP_ASYNCIO) IDLINFO("    > Supports asynchronous I/O method");
  if (vc.capabilities & V4L2_CAP_STREAMING) { IDLINFO("    > Supports streaming I/O (MMAP) method"); itsCanMMap=true; }
  else itsCanMMap = false;

  // List available controls and their current settings. First try to do this using the new method, for devices that
  // support the new extended controls API. If that fails we will fall back to the older method for standard controls:
  struct v4l2_queryctrl ctrl; memset(&ctrl, 0, sizeof(ctrl)); ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL;

  if (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0) {
    IDLINFO("    > Supports Extended Controls API");
    do {
      // add a command-line option for the control, and describe it:
      addControl(ctrl);

      // get ready to query the next control:
      ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL;
    } while (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0);

    // on some cameras (e.g., Microsoft LifeCam), we just got all the controls and we are done. On others, more controls
    // are available by excplicitly setting the class (e.g., HP Premium AF). So let's try that too, addControl() will
    // need to make sure we don't try to add a control twice:
    memset(&ctrl, 0, sizeof(ctrl)); ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_CLASS_MPEG;
    if (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0)
      do { addControl(ctrl); ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL; }
      while (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0);

//    memset(&ctrl, 0, sizeof(ctrl)); ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_CLASS_CAMERA;
//    if (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0)
//      do { addControl(ctrl); ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL; }
//      while (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0);

#ifdef V4L2_CTRL_CLASS_FM_TX
    memset(&ctrl, 0, sizeof(ctrl)); ctrl.id = V4L2_CTRL_FLAG_NEXT_CTRL | V4L2_CTRL_CLASS_FM_TX;
    if (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0)
      do { addControl(ctrl); ctrl.id |= V4L2_CTRL_FLAG_NEXT_CTRL; }
      while (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0);
#endif

  } else {
    IDLINFO("    > Does not support Extended Controls API");
    memset(&ctrl, 0, sizeof(ctrl));

    for (ctrl.id = V4L2_CID_BASE; ctrl.id < V4L2_CID_LASTP1; ++ctrl.id)
      if (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0)
        addControl(ctrl);  // add a command-line option for the control, and describe it:

    for (ctrl.id = V4L2_CID_PRIVATE_BASE; ; ++ctrl.id)
      if (ioctl_nointr(itsFd, VIDIOC_QUERYCTRL, &ctrl) == 0)
        addControl(ctrl);  // add a command-line option for the control, and describe it:
      else break;
  }
}

// ######################################################################
void V4L2grabber::addControl(const struct v4l2_queryctrl& ctrl)
{
  // make sure we have not already added the control. This is because on some cameras the enumeration of extended
  // controls is broken, see comments in openDevice():
  for (size_t i = 0; i < itsControls.size(); ++i) if (itsControls[i]->cid == ctrl.id) return;

  std::string info = "CONTROL: "; std::vector<std::string> infomenu;

  // find a command-line option for this control:
  int ii = 0; const ModelOptionDef *option = NULL;
  while (controlmapping[ii].opt != NULL)
    if (controlmapping[ii].v4l2 == ctrl.id) { option = controlmapping[ii].opt; break; } else ++ ii;

  // display some info about this control:
  switch(ctrl.type)
    {
    case V4L2_CTRL_TYPE_INTEGER:
      info += sformat("%s [int] Def=%d, Rng=[%d ..(%d).. %d]",
                      ctrl.name, ctrl.default_value, ctrl.minimum, ctrl.step, ctrl.maximum);
      if (option) {
        rutz::shared_ptr<V4L2grabberControl<int> >
          param(new V4L2grabberControl<int>(option, this, ctrl.default_value, itsOptionFlags, ctrl.id, ctrl.type));
        itsControls.push_back(param);
        getManager().requestOption(param->param, true); // always use my val (the default) so change is not rejected
        info += " (use --" + std::string(option->longoptname) + "=<int>)";
      } else info = "UNSUPPORTED INT" + info;

      break;

    case V4L2_CTRL_TYPE_BOOLEAN:
      info += sformat("%s [boolean] Def=%s", ctrl.name, ctrl.default_value ? "true" : "false");
      if (option) {
        rutz::shared_ptr<V4L2grabberControl<bool> >
          param(new V4L2grabberControl<bool>(option, this, ctrl.default_value, itsOptionFlags, ctrl.id, ctrl.type));
        itsControls.push_back(param);
        getManager().requestOption(param->param, true);
        info += " (use --" + std::string(option->longoptname) + "=<bool>)";
      } else info = "UNSUPPORTED BOOLEAN" + info;

      break;

    case V4L2_CTRL_TYPE_MENU:
      info += sformat("%s [menu] Def=%d", ctrl.name, ctrl.default_value);
      {
        // enumerate the menu items:
        struct v4l2_querymenu querymenu; memset(&querymenu, 0, sizeof(querymenu));
        querymenu.id = ctrl.id;

        for (querymenu.index = ctrl.minimum; int(querymenu.index) <= ctrl.maximum; ++querymenu.index) {
          if (ioctl_nointr(itsFd, VIDIOC_QUERYMENU, &querymenu) == 0)
            infomenu.push_back(sformat("CONTROL:            - %d = %s", querymenu.index, querymenu.name));
          else IDPLERROR("VIDIOC_QUERYMENU");
        }
      }
      if (option) {

        rutz::shared_ptr<V4L2grabberControl<int> >
          param(new V4L2grabberControl<int>(option, this, ctrl.default_value, itsOptionFlags, ctrl.id, ctrl.type));
        itsControls.push_back(param);
        getManager().requestOption(param->param, true);
        info += " (use --" + std::string(option->longoptname) + "=<int>)";

      } else info = "UNSUPPORTED MENU" + info;

      break;

    case V4L2_CTRL_TYPE_BUTTON:
      info += sformat("%s [button]", ctrl.name);

      if (option) {
        rutz::shared_ptr<V4L2grabberControl<bool> >
          param(new V4L2grabberControl<bool>(option, this, ctrl.default_value, itsOptionFlags, ctrl.id, ctrl.type));
        itsControls.push_back(param);
        getManager().requestOption(param->param, true);
        info += " (use --" + std::string(option->longoptname) + "=<bool>)";
      } else info = "UNSUPPORTED BUTTON" + info;

      break;

    case V4L2_CTRL_TYPE_INTEGER64:
      info += sformat("%s [int64]", ctrl.name);

      if (option) {
        rutz::shared_ptr<V4L2grabberControl<int64> >
          param(new V4L2grabberControl<int64>(option, this, ctrl.default_value, itsOptionFlags, ctrl.id, ctrl.type));
        itsControls.push_back(param);
        getManager().requestOption(param->param, true);
        info += " (use --" + std::string(option->longoptname) + "=<int64>)";
      } else info = "UNSUPPORTED INT64" + info;

      break;

    case V4L2_CTRL_TYPE_CTRL_CLASS: // Note: this should never happen
      info += sformat("%s [control class]", ctrl.name);
      break;

#ifdef V4L2_CTRL_TYPE_STRING
    case V4L2_CTRL_TYPE_STRING:
      info += sformat("%s [string]", ctrl.name);

      if (option) {
        rutz::shared_ptr<V4L2grabberControl<std::string> >
          param(new V4L2grabberControl<std::string>(option, this, "", itsOptionFlags, ctrl.id, ctrl.type));
        itsControls.push_back(param);
        getManager().requestOption(param->param, true);
        info += " (use --" + std::string(option->longoptname) + "=<string>)";
      } else info = "UNSUPPORTED STRING" + info;

      break;
#endif

    default:
      info += sformat("%s [unknown control 0x%x]", ctrl.name, ctrl.id);
      break;
    }

  std::vector<std::string> infoflags;
  if (ctrl.flags & V4L2_CTRL_FLAG_DISABLED) infoflags.push_back("DISABLED");
  if (ctrl.flags & V4L2_CTRL_FLAG_GRABBED) infoflags.push_back("GRABBED");
  if (ctrl.flags & V4L2_CTRL_FLAG_READ_ONLY) infoflags.push_back("READONLY");
  if (ctrl.flags & V4L2_CTRL_FLAG_UPDATE) infoflags.push_back("UPDATE");
  if (ctrl.flags & V4L2_CTRL_FLAG_INACTIVE) infoflags.push_back("INACTIVE");
  if (ctrl.flags & V4L2_CTRL_FLAG_SLIDER) infoflags.push_back("SLIDER");

  if (infoflags.size() > 0) {
    info += " (" + infoflags[0];
    for (size_t ii = 1; ii < infoflags.size(); ++ii) info += ", " + infoflags[ii];
    info += ")";
  }

  // print the info:
  IDLINFO("%s", info.c_str());
  for (size_t ii = 0; ii < infomenu.size(); ++ii) IDLINFO("%s", infomenu[ii].c_str());

  // now set the value. If it is an extended control, we will change it using the extended control interface. Note that
  // here, some settings may fail, e.g., setting the manual focus value when autofocus is on. So we are going to be very
  // quiet about errors and just show everything at the LDEBUG level:
  if (V4L2_CTRL_ID2CLASS(ctrl.id) != V4L2_CTRL_CLASS_USER) {
    struct v4l2_ext_controls extctrls; memset(&extctrls, 0, sizeof(extctrls));
    extctrls.count = 1; extctrls.ctrl_class = V4L2_CTRL_ID2CLASS(ctrl.id);

    struct v4l2_ext_control xc; memset(&xc, 0, sizeof(xc)); extctrls.controls = &xc;

    xc.id = ctrl.id; xc.value = ctrl.default_value;

    if (ioctl_nointr(itsFd, VIDIOC_S_EXT_CTRLS, &extctrls) == -1)
      IDPLDEBUG("NOTE: Failed to set '%s' to %d (VIDIOC_S_EXT_CTRLS)", ctrl.name, ctrl.default_value);
    else IDLDEBUG("Suceesfully set control '%s' to value %d", ctrl.name, ctrl.default_value);
  } else {
    // old-style interface:
    struct v4l2_control c; memset(&c, 0, sizeof(c));
    c.id = ctrl.id; c.value = ctrl.default_value;

    if (ioctl_nointr(itsFd, VIDIOC_S_CTRL, &c) == -1)
      IDPLDEBUG("NOTE: Failed to set '%s' to %d (VIDIOC_S_EXT_CTRLS)", ctrl.name, ctrl.default_value);
    else IDLDEBUG("Successfully set control '%s' to value %d", ctrl.name, ctrl.default_value);
  }
}

// ######################################################################
void V4L2grabber::closeDevice()
{
  if (itsStreamStarted)
    {
      enum v4l2_buf_type typ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (ioctl_nointr(itsFd, VIDIOC_STREAMOFF, &typ)) IDPLERROR("VIDIOC_STREAMOFF");
      itsStreamStarted = false;
    }

  if (itsMmapBuf)
    {
      for (int i = 0; i < itsNbuf.getVal(); i ++) munmap(itsMmapBuf[i], itsMmapBufSize[i]);
      delete [] itsMmapBuf; itsMmapBuf = NULL;
      delete [] itsMmapBufSize; itsMmapBufSize = NULL;
    }

  if (itsFd >= 0) { close(itsFd); itsFd = -1; }
  if (itsGrabbing) { delete [] itsGrabbing; itsGrabbing = NULL; }
  itsReadBuf = Image<byte>();
  itsStreamStarted = false;
}

// ######################################################################
void V4L2grabber::start1()
{
  if (itsFd == -1) openDevice();

  if (itsCanMMap == false && itsCanRW == false) IDLFATAL("No known frame grabbing method supported by hardware");

  // list available channels:
  for (int i = 0; ; i++)
    {
      struct v4l2_input inp; memset(&inp, 0, sizeof(inp)); inp.index = i;
      if (ioctl_nointr(itsFd, VIDIOC_ENUMINPUT, &inp) != 0) break;
      IDLINFO("Video input %d is '%s'", i, inp.name);
    }

  // select desired channel:
  int channel = itsChannel.getVal();
  if (ioctl_nointr(itsFd, VIDIOC_S_INPUT, &channel) < 0) IDPLERROR("Cannot select video input %d", itsChannel.getVal());
  else IDLINFO("Selected video input %d", itsChannel.getVal());

  // Reset cropping parameters and get resolution capabilities. NOTE: just open()'ing the device does not reset it,
  // according to the unix toolchain philosophy. Hence, although here we do not provide support for cropping, we still
  // need to ensure that it is properly reset:
  struct v4l2_cropcap cropcap;
  memset (&cropcap, 0, sizeof(cropcap));
  cropcap.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl_nointr(itsFd, VIDIOC_CROPCAP, &cropcap) == -1) IDPLERROR("Failed to get cropping capabilities -- IGNORED.");
  else {
    IDLINFO("Video capture bounds: (%d, %d) -> (%d, %d)", cropcap.bounds.left, cropcap.bounds.top,
            cropcap.bounds.left + cropcap.bounds.width - 1, cropcap.bounds.top + cropcap.bounds.height - 1);
    IDLINFO("Video default capture rectangle: (%d, %d) -> (%d, %d)", cropcap.defrect.left, cropcap.defrect.top,
            cropcap.defrect.left + cropcap.defrect.width - 1, cropcap.defrect.top + cropcap.defrect.height - 1);
  }

  struct v4l2_crop crop; memset (&crop, 0, sizeof(crop));
  crop.type = V4L2_BUF_TYPE_VIDEO_CAPTURE; crop.c = cropcap.defrect;
  if (ioctl_nointr(itsFd, VIDIOC_S_CROP, &crop) == -1) IDPLERROR("Failed to reset cropping settings -- IGNORED.");

  // list available video formats and see which ones would work with the requested resolution:
  struct v4l2_format fmt; memset(&fmt, 0, sizeof(fmt));
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = itsDims.getVal().w();
  fmt.fmt.pix.height = itsDims.getVal().h();
  fmt.fmt.pix.field = V4L2_FIELD_INTERLACED;
  int fmt_to_use = -1; // in case user selected "auto" format

  for (int i = 0; ; ++ i) {
    struct v4l2_fmtdesc fdesc; memset(&fdesc, 0, sizeof(fdesc));
    fdesc.index = i; fdesc.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    if (ioctl_nointr(itsFd, VIDIOC_ENUM_FMT, &fdesc) != 0) break;

    const VideoFormat f = V4L2Format_to_VideoFormat(fdesc.pixelformat);

    // try to see whether that would work with our given image dims:
    fmt.fmt.pix.pixelformat = fdesc.pixelformat;
    bool worked = false;
    if (ioctl_nointr(itsFd, VIDIOC_TRY_FMT, &fmt) == 0) {
      worked = true; if (fmt_to_use == -1) fmt_to_use = fdesc.pixelformat;
    }
    IDLINFO("Video Format: Use '%s' for '%s (Fourcc: %c%c%c%c)'%s%s",
            f == VIDFMT_AUTO ?
            "????" : convertToString(f).c_str(),
            fdesc.description,
            ((char*)(&fdesc.pixelformat))[0],
            ((char*)(&fdesc.pixelformat))[1],
            ((char*)(&fdesc.pixelformat))[2],
            ((char*)(&fdesc.pixelformat))[3],
            fdesc.flags & V4L2_FMT_FLAG_COMPRESSED ? " (COMPRESSED)" : "",
            worked ? " [OK]" : " [FAILED TO SET]");
  }

  // Get the frame time, according to current video standard:
  struct v4l2_streamparm sparm; memset(&sparm, 0, sizeof(sparm));
  sparm.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl_nointr(itsFd, VIDIOC_G_PARM, &sparm) == -1) {
    IDPLERROR("Cannot get video standard params - ASSUMING NTSC");
    itsFrameTime = SimTime::HERTZ(29.97);
  }
  else itsFrameTime = SimTime::SECS(float(sparm.parm.capture.timeperframe.numerator) /
                                    float(sparm.parm.capture.timeperframe.denominator));
  IDLINFO("Capture at %.2f fps", itsFrameTime.hertz());

  // try to select a video mode and image format:
  if (itsGrabMode.getVal() == VIDFMT_AUTO) {
    if (fmt_to_use == -1) IDLFATAL("Could not find a working video mode for given resolution.");
    fmt.fmt.pix.pixelformat = fmt_to_use;
  } else fmt.fmt.pix.pixelformat = VideoFormat_to_V4L2Format(itsGrabMode.getVal());

  if (ioctl_nointr(itsFd, VIDIOC_S_FMT, &fmt) == -1)
    IDPLFATAL("Cannot set requested video mode/resolution %s [%dx%d], probably unsupported by hardware.",
              convertToString(itsGrabMode.getVal()).c_str(), itsDims.getVal().w(), itsDims.getVal().h());

  if (ioctl_nointr(itsFd, VIDIOC_G_FMT, &fmt) == 0)
    IDLINFO("Video mode/resolution set to %s [%dx%d]", convertToString(itsGrabMode.getVal()).c_str(),
            fmt.fmt.pix.width, fmt.fmt.pix.height);

  if (int(fmt.fmt.pix.width) != itsDims.getVal().w() || int(fmt.fmt.pix.height) != itsDims.getVal().h())
    IDLFATAL("Hardware suggests changing input dims to %dx%d instead of requested %dx%d",
             fmt.fmt.pix.width, fmt.fmt.pix.height, itsDims.getVal().w(), itsDims.getVal().h());

  // if MMAP is supported, allocate a mmap'ed buffer:
  if (itsCanMMap) {
    IDLINFO("Using mmap'ed image capture with %d buffers", itsNbuf.getVal());
    struct v4l2_requestbuffers req; memset(&req, 0, sizeof(req));
    req.count = itsNbuf.getVal();
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl_nointr(itsFd, VIDIOC_REQBUFS, &req) == -1)
      IDPLFATAL("Cannot allocate %d mmap'ed video frame buffers", itsNbuf.getVal());
    if (int(req.count) != itsNbuf.getVal())
      IDLFATAL("Hardware can only support %d video frame buffers (vs. %d requested)", req.count, itsNbuf.getVal());

    itsMmapBuf = new byte*[req.count];
    itsMmapBufSize = new int[req.count];

    for (uint i = 0; i < req.count; ++i) {
      struct v4l2_buffer buf; memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = i;

      if (ioctl_nointr(itsFd, VIDIOC_QUERYBUF, &buf) == -1) IDPLFATAL("Could not query for MMAP buffer");

      itsMmapBufSize[i] = buf.length;
      itsMmapBuf[i] = static_cast<byte*>(mmap(NULL, buf.length, PROT_READ|PROT_WRITE, MAP_SHARED, itsFd, buf.m.offset));

      if (itsMmapBuf[i] == MAP_FAILED) IDPLFATAL("Error MMAP'ing video buffer number %d", i);
    }

    // we don't need the read() buffer:
    itsReadBuf = Image<byte>();
  } else {
    // get ready for read() access method:
    IDLINFO("Using read() image capture");
    itsReadBuf = Image<byte>(getFrameSize(itsGrabMode.getVal(), itsDims.getVal()), 1, NO_INIT);
    itsMmapBuf = NULL;
    itsMmapBufSize = NULL;
  }

  // get ready to grab frames, starting with buffer/frame 0:
  IDLDEBUG("Using %d grab buffers", itsNbuf.getVal());
  itsCurrentFrame = 0;
  itsGrabbing = new bool[itsNbuf.getVal()];
  for (int i = 0; i < itsNbuf.getVal(); ++i) itsGrabbing[i] = false;
}

// ######################################################################
void V4L2grabber::stop2()
{
  closeDevice();
}

// ######################################################################
V4L2grabber::~V4L2grabber()
{  }

// ######################################################################
void V4L2grabber::setListener(rutz::shared_ptr<FrameListener> listener)
{ itsListener = listener; }

// ######################################################################
void V4L2grabber::paramChanged(ModelParamBase* const param, const bool valueChanged,
                               ParamClient::ChangeStatus* status)
{
  // re-open the device if its name was changed:
  if (valueChanged && param == &itsDevName) {
    if (started()) IDLFATAL("Cannot change device while started");
    closeDevice(); openDevice();
  }

  if (valueChanged && param == &itsChannel) {
    int channel = itsChannel.getVal();
    if (ioctl_nointr(itsFd, VIDIOC_S_INPUT, &channel) < 0)
      IDPLERROR("Cannot select video input %d", itsChannel.getVal());
    else
      IDLINFO("Selected video input %d", itsChannel.getVal());
  }

  // is that one of our camera controls?
  if (valueChanged)
    for (size_t i = 0; i < itsControls.size(); ++i) {
      int val = 0; const char *name = NULL;

      // match the param and get its value:
      switch(itsControls[i]->ctype) {
      case V4L2_CTRL_TYPE_INTEGER:
        {
          const V4L2grabberControl<int>& c = dynamic_cast<V4L2grabberControl<int>&>(*itsControls[i]);
          const OModelParam<int>& p = c.param;
          if (param == &p) { val = p.getVal(); name = p.getOptionDef()->longoptname; }
        }
        break;
      case V4L2_CTRL_TYPE_BOOLEAN:
        {
          const V4L2grabberControl<bool>& c = dynamic_cast<V4L2grabberControl<bool>&>(*itsControls[i]);
          const OModelParam<bool>& p = c.param;
          if (param == &p) { val = p.getVal(); name = p.getOptionDef()->longoptname; }
        }
        break;
      case V4L2_CTRL_TYPE_MENU:
        {
          const V4L2grabberControl<int>& c = dynamic_cast<V4L2grabberControl<int>&>(*itsControls[i]);
          const OModelParam<int>& p = c.param;
          if (param == &p) { val = p.getVal(); name = p.getOptionDef()->longoptname; }
        }
        break;
      case V4L2_CTRL_TYPE_BUTTON:
        {
          const V4L2grabberControl<bool>& c = dynamic_cast<V4L2grabberControl<bool>&>(*itsControls[i]);
          const OModelParam<bool>& p = c.param;
          if (param == &p) { val = p.getVal(); name = p.getOptionDef()->longoptname; }
        }
        break;
      case V4L2_CTRL_TYPE_INTEGER64:
        {
          const V4L2grabberControl<int64>& c = dynamic_cast<V4L2grabberControl<int64>&>(*itsControls[i]);
          const OModelParam<int64>& p = c.param;
          if (param == &p) { val = static_cast<int>(p.getVal()); name = p.getOptionDef()->longoptname; }
        }
        break;

#ifdef V4L2_CTRL_TYPE_STRING
      case V4L2_CTRL_TYPE_STRING:
        {
          const V4L2grabberControl<std::string>& c = dynamic_cast<V4L2grabberControl<std::string>&>(*itsControls[i]);
          const OModelParam<std::string>& p = c.param;
          if (param == &p) IDLFATAL("Changing value of string control not yet supported");
        }
        break;
#endif
      default: IDLFATAL("Changing value of control of type 0x%x not supported", itsControls[i]->ctype); break;
      }

      if (name) {
        // ok, we got the param. If it is extended, we will change it using the extended control interface:
        if (V4L2_CTRL_ID2CLASS(itsControls[i]->cid) != V4L2_CTRL_CLASS_USER) {
          struct v4l2_ext_controls extctrls; memset(&extctrls, 0, sizeof(extctrls));
          extctrls.count = 1; // we'll set one extended control at a time
          extctrls.ctrl_class = V4L2_CTRL_ID2CLASS(itsControls[i]->cid);

          struct v4l2_ext_control xctrl; memset(&xctrl, 0, sizeof(xctrl));
          extctrls.controls = &xctrl; // our one extended control to set

          xctrl.id = itsControls[i]->cid;
          xctrl.value = val;

          if (ioctl_nointr(itsFd, VIDIOC_S_EXT_CTRLS, &extctrls) == -1) {
            IDPLERROR("NOTE: Failed to set --%s to %d (VIDIOC_S_EXT_CTRLS)", name, val);
            //*status = ParamClient::CHANGE_REJECTED;
          } else IDLDEBUG("Changed control --%s to value %d", name, val);
        } else {
          // old-style interface:
          struct v4l2_control ctrl; memset(&ctrl, 0, sizeof(ctrl));
          ctrl.id = itsControls[i]->cid;
          ctrl.value = val;

          if (ioctl_nointr(itsFd, VIDIOC_S_CTRL, &ctrl) == -1) {
            IDPLERROR("NOTE: Failed to set --%s to %d (VIDIOC_S_EXT_CTRLS)", name, val);
            //*status = ParamClient::CHANGE_REJECTED;
          } else IDLDEBUG("Changed control --%s to value %d", name, val);
        }
      }
    }

  FrameIstream::paramChanged(param, valueChanged, status);
}

// ######################################################################
void V4L2grabber::startStream()
{
  // reset itsStreamStarted so that we wait for any pending frames to
  // be grabbed, then start fresh grabbing requests:
  itsStreamStarted = false;

  this->restartStream();
}

// ######################################################################
SimTime V4L2grabber::getNaturalFrameTime() const
{
  return itsFrameTime;
}

// ######################################################################
GenericFrameSpec V4L2grabber::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::VIDEO;
  result.videoFormat = itsGrabMode.getVal();
  result.videoByteSwap = itsByteSwap.getVal();
  result.dims = itsDims.getVal();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame V4L2grabber::readFrame()
{
  const GenericFrame frame =
    itsStreamingMode.getVal()
    ? GenericFrame(this->grabRaw())
    : GenericFrame(this->grabSingleRaw());

  if (itsListener.get() != 0)
    itsListener->onRawFrame(frame);

  return frame;
}

// ######################################################################
VideoFrame V4L2grabber::grabRaw()
{
  byte* result = 0; int siz = 0;
  if (itsMmapBuf) // mmap interface
    {
      this->restartStream();
      result = itsMmapBuf[itsCurrentFrame];
      siz = itsMmapBufSize[itsCurrentFrame];
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = itsCurrentFrame;

      // are we already grabbing buffer 'itsCurrentFrame'? otherwise,
      // start the grab now:
      if (itsGrabbing[itsCurrentFrame] == false)
        {
          if (ioctl_nointr(itsFd, VIDIOC_QBUF, &buf))
            IDPLFATAL("VIDIOC_QBUF (frame %d)", itsCurrentFrame);
          itsGrabbing[itsCurrentFrame] = true;
        }

      // wait until buffer 'itsCurrentFrame' has been fully captured:
      if (ioctl_nointr(itsFd, VIDIOC_DQBUF, &buf) == -1)
        IDPLFATAL("VIDIOC_DQBUF (frame %d)", itsCurrentFrame);
      itsGrabbing[itsCurrentFrame] = false;

      // get ready for capture of that frame again (for later):
      itsGrabbing[itsCurrentFrame] = true;
      if (ioctl_nointr(itsFd, VIDIOC_QBUF, &buf) < 0)
        IDPLFATAL("VIDIOC_QBUF (frame %d)", itsCurrentFrame);
    }
  else // read() interface
    {
      ASSERT(itsReadBuf.initialized());
      const ssize_t nbytes =
        read(itsFd, itsReadBuf.getArrayPtr(), itsReadBuf.getSize());
      if (nbytes < 0) IDPLFATAL("read() failed (frame %d)", itsCurrentFrame);
      IDLDEBUG("Got %zd bytes", nbytes);
      result = itsReadBuf.getArrayPtr();
      siz = nbytes;
    }

  // switch to another frame:
  ++itsCurrentFrame;
  if (itsCurrentFrame >= itsNbuf.getVal()) itsCurrentFrame = 0;

  // return pointer to last-grabbed frame. You have a bit of time to
  // get a hold of the data but beware that an order to capture that
  // buffer has already been issued, so as soon as the grabber gets to
  // it, it will overwrite this buffer with a new frame:
  ASSERT(result != 0);
  VideoFrame frame(result, siz, itsDims.getVal(),
                   itsGrabMode.getVal(), itsByteSwap.getVal(),
                   /* strictLength = */ false);
  return frame;
}

// ######################################################################
VideoFrame V4L2grabber::grabSingleRaw()
{
  byte* result = 0; int siz = 0;
  if (itsMmapBuf) // mmap interface
    {
      itsCurrentFrame = 0;
      result = itsMmapBuf[itsCurrentFrame];
      siz = itsMmapBufSize[itsCurrentFrame];
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));
      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;
      buf.index = itsCurrentFrame;

      // are we already grabbing buffer 'itsCurrentFrame'? otherwise,
      // start the grab now:
      if (itsGrabbing[itsCurrentFrame] == false)
        {
          if (ioctl_nointr(itsFd, VIDIOC_QBUF, &buf))
            IDPLFATAL("VIDIOC_QBUF (frame %d)", itsCurrentFrame);
          itsGrabbing[itsCurrentFrame] = true;
        }

      // wait until buffer 'itsCurrentFrame' has been fully captured:
      if (ioctl_nointr(itsFd, VIDIOC_DQBUF, &buf) == -1)
        IDPLFATAL("VIDIOC_DQBUF (frame %d)", itsCurrentFrame);
      itsGrabbing[itsCurrentFrame] = false;
    }
  else // read() interface
    {
      ASSERT(itsReadBuf.initialized());
      const ssize_t nbytes =
        read(itsFd, itsReadBuf.getArrayPtr(), itsReadBuf.getSize());
      if (nbytes < 0) IDPLFATAL("read() failed (frame %d)", itsCurrentFrame);
      IDLDEBUG("Got %zd bytes", nbytes);
      result = itsReadBuf.getArrayPtr();
      siz = nbytes;
    }

  // return grabbed & converted frame:
  VideoFrame frame(result, siz, itsDims.getVal(),
                   itsGrabMode.getVal(), itsByteSwap.getVal(),
                   /* strictLength = */ false);
  return frame;
}

// ######################################################################
void V4L2grabber::restartStream()
{
  if (itsStreamingMode.getVal()
      && itsMmapBuf
      && !itsStreamStarted)
    {
      struct v4l2_buffer buf;
      memset(&buf, 0, sizeof(buf));

      buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      buf.memory = V4L2_MEMORY_MMAP;

      for (int i = 0; i < itsNbuf.getVal(); ++i)
        if (itsGrabbing[i] == true)
          {
            // are we already grabbing buffer i?  if so, wait for grab
            // to finished if we've requested resync (if we don't do
            // this, then the first few frames after a restart will be
            // wrong -- they'll come too quickly and they'll be too old,
            // since the v4l driver doesn't grab new frames into buffers
            // until the old frames have been retrieved):
            buf.index = i;
            if (ioctl_nointr(itsFd, VIDIOC_DQBUF, &buf) == -1)
              IDPLFATAL("VIDIOC_DQBUF (frame %d)", i);
            itsGrabbing[i] = false;
          }

      for (int i = 0; i < itsNbuf.getVal(); ++i)
        {
          // now start a fresh grab for buffer i:
          buf.index = i;
          if (ioctl_nointr(itsFd, VIDIOC_QBUF, &buf))
            IDPLFATAL("VIDIOC_QBUF (frame %d)", i);
          itsGrabbing[i] = true;
        }

      // tell grabber to stream:
      enum v4l2_buf_type typ = V4L2_BUF_TYPE_VIDEO_CAPTURE;
      if (ioctl_nointr(itsFd, VIDIOC_STREAMON, &typ))
        PLFATAL("VIDIOC_STREAMON");

      itsCurrentFrame = 0;
      itsStreamStarted = true;
    }
}

#endif // HAVE_LINUX_VIDEODEV2_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
