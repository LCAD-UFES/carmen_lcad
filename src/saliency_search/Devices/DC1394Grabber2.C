/*!@file Devices/DC1394Grabber2.C FireWire grabber class based on libdc1394 version 2.x */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/DC1394Grabber2.C $
// $Id: DC1394Grabber2.C 14376 2011-01-11 02:44:34Z pez $
//

#ifndef DEVICES_DC1394GRABBER2_C_DEFINED
#define DEVICES_DC1394GRABBER2_C_DEFINED

#include "Devices/DC1394Grabber2.H"

#include "Devices/DeviceOpts.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Transport/TransportOpts.H"
#include "Util/Assert.H"
#include "Util/SimTime.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"

#include <unistd.h>

// wait when polling for a frame (in us):
#define IEEE1394WAIT 1

// This code is somewhat inspired from the grab_gray_image.c example
// code provided with the libdc1394 distribution by Gord Peters, and
// from the Coriander source code.
#ifdef HAVE_DC1394V2

namespace
{
  std::string convertToString(dc1394video_mode_t mode)
  {
    switch (mode)
      {
      case DC1394_VIDEO_MODE_160x120_YUV444: return "160x120_YUV444";
      case DC1394_VIDEO_MODE_320x240_YUV422: return "320x240_YUV422";
      case DC1394_VIDEO_MODE_640x480_YUV411: return "640x480_YUV411";
      case DC1394_VIDEO_MODE_640x480_YUV422: return "640x480_YUV422";
      case DC1394_VIDEO_MODE_640x480_RGB8: return "640x480_RGB8";
      case DC1394_VIDEO_MODE_640x480_MONO8: return "640x480_MONO8";
      case DC1394_VIDEO_MODE_640x480_MONO16: return "640x480_MONO16";
      case DC1394_VIDEO_MODE_800x600_YUV422: return "800x600_YUV422";
      case DC1394_VIDEO_MODE_800x600_RGB8: return "800x600_RGB8";
      case DC1394_VIDEO_MODE_800x600_MONO8: return "800x600_MONO8";
      case DC1394_VIDEO_MODE_1024x768_YUV422: return "1024x768_YUV422";
      case DC1394_VIDEO_MODE_1024x768_RGB8: return "1024x768_RGB8";
      case DC1394_VIDEO_MODE_1024x768_MONO8: return "1024x768_MONO8";
      case DC1394_VIDEO_MODE_800x600_MONO16: return "800x600_MONO16";
      case DC1394_VIDEO_MODE_1024x768_MONO16: return "1024x768_MONO16";
      case DC1394_VIDEO_MODE_1280x960_YUV422: return "1280x960_YUV422";
      case DC1394_VIDEO_MODE_1280x960_RGB8: return "1280x960_RGB8";
      case DC1394_VIDEO_MODE_1280x960_MONO8: return "1280x960_MONO8";
      case DC1394_VIDEO_MODE_1600x1200_YUV422: return "1600x1200_YUV422";
      case DC1394_VIDEO_MODE_1600x1200_RGB8: return "1600x1200_RGB8";
      case DC1394_VIDEO_MODE_1600x1200_MONO8: return "1600x1200_MONO8";
      case DC1394_VIDEO_MODE_1280x960_MONO16: return "1280x960_MONO16";
      case DC1394_VIDEO_MODE_1600x1200_MONO16: return "1600x1200_MONO16";
      case DC1394_VIDEO_MODE_EXIF: return "EXIF";
      case DC1394_VIDEO_MODE_FORMAT7_0: return "FORMAT7_0";
      case DC1394_VIDEO_MODE_FORMAT7_1: return "FORMAT7_1";
      case DC1394_VIDEO_MODE_FORMAT7_2: return "FORMAT7_2";
      case DC1394_VIDEO_MODE_FORMAT7_3: return "FORMAT7_3";
      case DC1394_VIDEO_MODE_FORMAT7_4: return "FORMAT7_4";
      case DC1394_VIDEO_MODE_FORMAT7_5: return "FORMAT7_5";
      case DC1394_VIDEO_MODE_FORMAT7_6: return "FORMAT7_6";
      case DC1394_VIDEO_MODE_FORMAT7_7: return "FORMAT7_7";
      default: break;
      }
    return "unknown";
  }

  std::string convertToString(dc1394framerate_t rate)
  {
    switch (rate)
      {
      case DC1394_FRAMERATE_1_875: return "1.875 fps";
      case DC1394_FRAMERATE_3_75: return "3.75 fps";
      case DC1394_FRAMERATE_7_5: return "7.5 fps";
      case DC1394_FRAMERATE_15: return "15 fps";
      case DC1394_FRAMERATE_30: return "30 fps";
      case DC1394_FRAMERATE_60: return "60 fps";
      case DC1394_FRAMERATE_120: return "120 fps";
      case DC1394_FRAMERATE_240: return "240 fps";
      default: break;
      }
    return "unknown";
  }

  std::string convertToString(dc1394speed_t speed)
  {
    switch (speed)
      {
      case DC1394_ISO_SPEED_100: return "100";
      case DC1394_ISO_SPEED_200: return "200";
      case DC1394_ISO_SPEED_400: return "400";
      case DC1394_ISO_SPEED_800: return "800";
      case DC1394_ISO_SPEED_1600: return "1600";
      case DC1394_ISO_SPEED_3200: return "3200";
      default: break;
      }
    return "unknown";
  }

  struct CameraModeInfo
  {
    VideoFormat vidfmt;
    int w;
    int h;
    dc1394video_mode_t dc1394mode;
    dc1394speed_t dc1394speed;
  };

  // Figure out which of the IEEE1394 grab modes to use. We don't
  // support the various IEEE1394 MONO16 modes.
  void find1394mode(const VideoFormat vidfmt, const Dims& dims,
                    dc1394video_mode_t* dc1394mode,
                    dc1394speed_t* dc1394speed)
  {
    static const CameraModeInfo modes[] =
      {
        { VIDFMT_YUV444,  160,  120, DC1394_VIDEO_MODE_160x120_YUV444   , DC1394_ISO_SPEED_400 },
        { VIDFMT_YUV422,  320,  240, DC1394_VIDEO_MODE_320x240_YUV422   , DC1394_ISO_SPEED_400 },
        { VIDFMT_YUV411,  640,  480, DC1394_VIDEO_MODE_640x480_YUV411   , DC1394_ISO_SPEED_400 },
        { VIDFMT_YUV422,  640,  480, DC1394_VIDEO_MODE_640x480_YUV422   , DC1394_ISO_SPEED_400 },
        { VIDFMT_RGB24,   640,  480, DC1394_VIDEO_MODE_640x480_RGB8     , DC1394_ISO_SPEED_400 },
        { VIDFMT_GREY,    640,  480, DC1394_VIDEO_MODE_640x480_MONO8    , DC1394_ISO_SPEED_400 },
        { VIDFMT_YUV422,  800,  600, DC1394_VIDEO_MODE_800x600_YUV422   , DC1394_ISO_SPEED_800 },
        { VIDFMT_RGB24,   800,  600, DC1394_VIDEO_MODE_800x600_RGB8     , DC1394_ISO_SPEED_800 },
        { VIDFMT_GREY,    800,  600, DC1394_VIDEO_MODE_800x600_MONO8    , DC1394_ISO_SPEED_800 },
        { VIDFMT_YUV422, 1024,  768, DC1394_VIDEO_MODE_1024x768_YUV422  , DC1394_ISO_SPEED_800 },
        { VIDFMT_RGB24,  1024,  768, DC1394_VIDEO_MODE_1024x768_RGB8    , DC1394_ISO_SPEED_800 },
        { VIDFMT_GREY,   1024,  768, DC1394_VIDEO_MODE_1024x768_MONO8   , DC1394_ISO_SPEED_800 },
        { VIDFMT_YUV422, 1280,  960, DC1394_VIDEO_MODE_1280x960_YUV422  , DC1394_ISO_SPEED_800 },
        { VIDFMT_RGB24,  1280,  960, DC1394_VIDEO_MODE_1280x960_RGB8    , DC1394_ISO_SPEED_800 },
        { VIDFMT_GREY,   1280,  960, DC1394_VIDEO_MODE_1280x960_MONO8   , DC1394_ISO_SPEED_800 },
        { VIDFMT_YUV422, 1600, 1200, DC1394_VIDEO_MODE_1600x1200_YUV422 , DC1394_ISO_SPEED_800 },
        { VIDFMT_RGB24,  1600, 1200, DC1394_VIDEO_MODE_1600x1200_RGB8   , DC1394_ISO_SPEED_800 },
        { VIDFMT_GREY,   1600, 1200, DC1394_VIDEO_MODE_1600x1200_MONO8  , DC1394_ISO_SPEED_800 }
      };

    for (size_t i = 0; i < sizeof(modes) / sizeof(CameraModeInfo); ++i)
      {
        if (modes[i].vidfmt == vidfmt
            && modes[i].w == dims.w()
            && modes[i].h == dims.h())
          {
            *dc1394mode = modes[i].dc1394mode;
            *dc1394speed = modes[i].dc1394speed;
            return;
          }
      }

    std::string msg =
      sformat("Unsupported resolution/format combination %s @ %dx%d"
              "\nSupported combinations are:",
              convertToString(vidfmt).c_str(), dims.w(), dims.h());
    for (size_t i = 0; i < sizeof(modes) / sizeof(CameraModeInfo); ++i)
      {
        msg += sformat("\n\t%7s @ %dx%d",
                       convertToString(modes[i].vidfmt).c_str(),
                       modes[i].w, modes[i].h);
      }

    LFATAL("%s", msg.c_str());
  }

  struct FramerateInfo
  {
    float fps;
    dc1394framerate_t dc1394framerate;
  };

  // Figure out which framerate to use
  dc1394framerate_t find1394framerate(const float fps)
  {
    static const FramerateInfo framerates[] =
      {
        { 1.875F, DC1394_FRAMERATE_1_875 },
        { 3.75F, DC1394_FRAMERATE_3_75 },
        { 7.5F, DC1394_FRAMERATE_7_5 },
        { 15.0F, DC1394_FRAMERATE_15 },
        { 30.0F, DC1394_FRAMERATE_30 },
        { 60.0F, DC1394_FRAMERATE_60 },
        { 120.0F, DC1394_FRAMERATE_120 },
        { 240.0F, DC1394_FRAMERATE_240 }
      };

    for (size_t i = 0; i < sizeof(framerates) / sizeof(FramerateInfo); ++i)
      if (framerates[i].fps == fps)
        return framerates[i].dc1394framerate;

    std::string msg =
      sformat("Unsupported framerate %f fps"
              "\nSupported framerates are:", fps);
    for (size_t i = 0; i < sizeof(framerates) / sizeof(FramerateInfo); ++i)
      msg += sformat("\n\t%.3f fps", framerates[i].fps);

    LFATAL("%s", msg.c_str());
    /* can't happen */ return (dc1394framerate_t)-1;
  }

}
#endif

// ######################################################################
DC1394Grabber2::DC1394Grabber2(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  FrameIstream(mgr, descrName, tagName),
  // NOTE that contrary to the common case, we give USE_MY_VAL here
  // when we construct the OModelParam objects; that means that we
  // push our values into the ModelManager as the new default values,
  // rather than having our param take its value from the
  // ModelManager's default
  itsShowInputDetails(&OPT_ShowInputDetails, this),
  itsDevName(&OPT_FrameGrabberDevice, this, "/dev/video1394/0", USE_MY_VAL), // device
  itsChannel(&OPT_FrameGrabberChannel, this, 0, USE_MY_VAL), // first firewire card
  itsSubChan(&OPT_FrameGrabberSubChan, this, 0, USE_MY_VAL),
  itsDims(&OPT_FrameGrabberDims, this, Dims(320, 240), USE_MY_VAL),
  itsGrabMode(&OPT_FrameGrabberMode, this, VIDFMT_YUV422, USE_MY_VAL), // grab mode
  itsByteSwap(&OPT_FrameGrabberByteSwap, this, false, USE_MY_VAL), // no byte-swapping
  itsFPS(&OPT_FrameGrabberFPS, this, 30.0, USE_MY_VAL),
  itsNbuf(&OPT_FrameGrabberNbuf, this, 4, USE_MY_VAL), // number of grab buffers
  itsBrightness(&OPT_FrameGrabberBrightness, this, 32768, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsHue(&OPT_FrameGrabberHue, this, 32768, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsSaturation(&OPT_FrameGrabberSaturation, this, 90, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsExposure(&OPT_FrameGrabberExposure, this, 511, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsSharpness(&OPT_FrameGrabberSharpness, this, 80, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsWhiteBalBU(&OPT_FrameGrabberWhiteBalBU, this, 95, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsWhiteBalRV(&OPT_FrameGrabberWhiteBalRV, this, 87, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsGamma(&OPT_FrameGrabberGamma, this, 1, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsShutter(&OPT_FrameGrabberShutter, this, 6, USE_MY_VAL | ALLOW_ONLINE_CHANGES),
  itsGain(&OPT_FrameGrabberGain, this, 87, USE_MY_VAL | ALLOW_ONLINE_CHANGES)
#ifdef HAVE_DC1394V2
  , itsDC1394(NULL)
  , itsCam(NULL)
#endif
{}

// ######################################################################
void DC1394Grabber2::start1()
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use DC1394Grabber2");
#else
  ASSERT(itsDC1394 == 0);

  itsDC1394 = dc1394_new();

  if (itsDC1394 == 0)
    LFATAL("dc1394_new() failed");

  {
    dc1394camera_list_t* list = 0;
    const dc1394error_t err = dc1394_camera_enumerate(itsDC1394, &list);
    if (err != DC1394_SUCCESS)
      LFATAL("dc1394_camera_enumerate() failed (%s)",
             dc1394_error_get_string(err));

    if (list->num == 0)
      LFATAL("no cameras found by dc1394_camera_enumerate()");

    itsCam = dc1394_camera_new(itsDC1394, list->ids[0].guid);
    if (itsCam == 0)
      LFATAL("failed to initialize camera with guid %llx",
             (long long unsigned int)list->ids[0].guid);

    LINFO("using the first camera on the bus");

    dc1394_camera_free_list(list);
  }

  // setup capture
  dc1394video_mode_t videomode = dc1394video_mode_t();
  dc1394speed_t speed = dc1394speed_t();
  find1394mode(itsGrabMode.getVal(), itsDims.getVal(),
               &videomode, &speed);

  dc1394error_t code = DC1394_SUCCESS;

  if (speed < DC1394_ISO_SPEED_800)
    {
      code = dc1394_video_set_operation_mode(itsCam, DC1394_OPERATION_MODE_LEGACY);
      if (code != DC1394_SUCCESS) LFATAL("couldn't set camera operation mode to 'legacy'");
    }
  else
    {
      code = dc1394_video_set_operation_mode(itsCam, DC1394_OPERATION_MODE_1394B);
      if (code != DC1394_SUCCESS) LFATAL("couldn't set camera operation mode to '1394B'");
    }

  code = dc1394_video_set_iso_speed(itsCam, speed);
  if (code != DC1394_SUCCESS) LFATAL("couldn't set camera ISO speed");

  code = dc1394_video_set_mode(itsCam, videomode);
  if (code != DC1394_SUCCESS) LFATAL("couldn't set camera video mode");

  const dc1394framerate_t framerate = find1394framerate(itsFPS.getVal());

  code = dc1394_video_set_framerate(itsCam, framerate);
  if (code != DC1394_SUCCESS) LFATAL("couldn't set camera framerate");

  code = dc1394_capture_setup(itsCam, 4, DC1394_CAPTURE_FLAGS_DEFAULT);
  if (code != DC1394_SUCCESS) LFATAL("capture setup failed; check settings");

  // set features based on ModelParam values

#define SET_FEATURE(fval, pname, fname)                                 \
  do {                                                                  \
    if (dc1394_feature_set_value(itsCam, DC1394_FEATURE_ ## fval,       \
                                 pname.getVal()) != DC1394_SUCCESS)     \
      LERROR("Unable to set " fname);                                   \
  } while (0)

  SET_FEATURE(BRIGHTNESS, itsBrightness, "brightness");
  SET_FEATURE(EXPOSURE, itsExposure, "exposure");
  SET_FEATURE(SHARPNESS, itsSharpness, "sharpness");
  SET_FEATURE(HUE, itsHue, "hue");
  SET_FEATURE(SATURATION, itsSaturation, "saturation");
  SET_FEATURE(GAMMA, itsGamma, "gamma");
  SET_FEATURE(SHUTTER, itsShutter, "shutter");
  SET_FEATURE(GAIN, itsGain, "gain");

#undef SET_FEATURE

  if (dc1394_feature_whitebalance_set_value(itsCam,
                                            itsWhiteBalBU.getVal(),
                                            itsWhiteBalRV.getVal())
      != DC1394_SUCCESS)
    LERROR("Unable to set white balance");

  // start ISO transmission
  code = dc1394_video_set_transmission(itsCam, DC1394_ON);
  if (code != DC1394_SUCCESS) LINFO("unable to start camera iso transmission");

  // report camera features if the user asked for details:
  if (itsShowInputDetails.getVal())
    {
      dc1394_camera_print_info(itsCam, stderr);

      dc1394featureset_t features;
      if (dc1394_feature_get_all(itsCam, &features) == DC1394_SUCCESS)
        dc1394_feature_print_all(&features, stderr);
    }

  {
    std::string info = sformat("vendor: %s; model: %s; ", itsCam->vendor, itsCam->model);

    dc1394video_mode_t videomode;
    if (dc1394_video_get_mode(itsCam, &videomode) == DC1394_SUCCESS)
      info += sformat("video mode: %s; ", convertToString(videomode).c_str());
    else
      info += "video mode: unknown; ";

    dc1394framerate_t framerate;
    if (dc1394_video_get_framerate(itsCam, &framerate) == DC1394_SUCCESS)
      info += sformat("framerate: %s; ", convertToString(framerate).c_str());
    else
      info += "framerate: unknown; ";

    dc1394speed_t speed;
    if (dc1394_video_get_iso_speed(itsCam, &speed) == DC1394_SUCCESS)
      info += sformat("speed: %s; ", convertToString(speed).c_str());
    else
      info += "speed: unknown; ";

    uint32_t chan = 0;
    if (dc1394_video_get_iso_channel(itsCam, &chan) == DC1394_SUCCESS)
      info += sformat("ISO channel: %u; ", (unsigned int) chan);
    else
      info += "ISO channel: unknown; ";

    LINFO("%s", info.c_str());
  }
#endif // HAVE_DC1394V2
}

// ######################################################################
void DC1394Grabber2::stop2()
{
#ifndef HAVE_DC1394V2
  // don't LFATAL() in stop() since it may be called in a destructor chain
  LERROR("you must have libdc1394 version 2.x in order to use DC1394Grabber2");
#else
  if (itsCam != 0)
    {
      dc1394_video_set_transmission(itsCam, DC1394_OFF);
      dc1394_capture_stop(itsCam);
      dc1394_camera_free(itsCam);
      itsCam = 0;
    }
  if (itsDC1394 != 0)
    {
      dc1394_free(itsDC1394);
      itsDC1394 = 0;
    }
#endif // HAVE_DC1394V2
}

// ######################################################################
DC1394Grabber2::~DC1394Grabber2()
{  }

// ######################################################################
GenericFrameSpec DC1394Grabber2::peekFrameSpec()
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
SimTime DC1394Grabber2::getNaturalFrameTime() const
{
  return SimTime::HERTZ(itsFPS.getVal());
}

// ######################################################################
GenericFrame DC1394Grabber2::readFrame()
{
  return GenericFrame(this->grabRaw());
}

// ######################################################################
VideoFrame DC1394Grabber2::grabRaw()
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use DC1394Grabber2");
  /* can't happen */ return VideoFrame();
#else
  dc1394video_frame_t* frame = 0;

  int code = dc1394_capture_dequeue(itsCam, DC1394_CAPTURE_POLICY_WAIT, &frame);
  if (code != DC1394_SUCCESS) LFATAL("unable to capture video frame");

  VideoFrame result(frame->image, frame->total_bytes, itsDims.getVal(),
                    itsGrabMode.getVal(), itsByteSwap.getVal(), false);

  code = dc1394_capture_enqueue(itsCam, frame);
  if (code != DC1394_SUCCESS) LFATAL("couldn't re-enqueue frame");

  return result;
#endif // HAVE_DC1394V2
}

// ######################################################################
void DC1394Grabber2::paramChanged(ModelParamBase* const param,
                                  const bool valueChanged,
                                  ParamClient::ChangeStatus* status)
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use DC1394Grabber2");
#else

  FrameIstream::paramChanged(param, valueChanged, status);

  // just handle online changes here (i.e. changes that come while we
  // are already started); if the change happens before start() then
  // we will pick up the proper dc1394 settings in start() so we don't
  // need to handle the change here

#define HANDLE_PARAM(FVAL, MODELPARAM, FNAME)           \
      if (valueChanged && param == &MODELPARAM)         \
        {                                               \
          LDEBUG("online change of " FNAME " from %s",  \
                 MODELPARAM.getName().c_str());         \
          if (dc1394_feature_set_value                  \
              (itsCam, DC1394_FEATURE_ ## FVAL,         \
               MODELPARAM.getVal())                     \
              != DC1394_SUCCESS)                        \
            {                                           \
              *status = ParamClient::CHANGE_REJECTED;   \
              LERROR("Unable to set " FNAME);           \
            }                                           \
        }

  HANDLE_PARAM(BRIGHTNESS, itsBrightness, "brightness");
  HANDLE_PARAM(EXPOSURE, itsExposure, "exposure");
  HANDLE_PARAM(SHARPNESS, itsSharpness, "sharpness");
  HANDLE_PARAM(HUE, itsHue, "hue");
  HANDLE_PARAM(SATURATION, itsSaturation, "saturation");
  HANDLE_PARAM(GAMMA, itsGamma, "gamma");
  HANDLE_PARAM(SHUTTER, itsShutter, "shutter");
  HANDLE_PARAM(GAIN, itsGain, "gain");

#undef HANDLE_PARAM

  if (valueChanged
      && (param == &itsWhiteBalBU || param == &itsWhiteBalRV))
    {
      LDEBUG("online change of white balance from %s",
             param->getName().c_str());

      if (dc1394_feature_whitebalance_set_value
          (itsCam, itsWhiteBalBU.getVal(), itsWhiteBalRV.getVal())
          != DC1394_SUCCESS)
        {
          *status = ParamClient::CHANGE_REJECTED;
          LERROR("Unable to set white balance");
        }
    }

#endif // HAVE_DC1394V2
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_DC1394GRABBER2_C_DEFINED
