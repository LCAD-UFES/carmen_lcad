/*!@file Devices/IEEE1394grabber.C Interface with a FireWire digital camera */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/IEEE1394grabber.C $
// $Id: IEEE1394grabber.C 10345 2008-10-15 17:27:10Z icore $
//

#include "Devices/IEEE1394grabber.H"

#include "Component/OptionManager.H" // for option alias requests
#include "Devices/DeviceOpts.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Raster/GenericFrame.H"
#include "Util/Assert.H"
#include "Util/SimTime.H"
#include "Util/sformat.H"
#include "Video/VideoFrame.H"

#ifdef HAVE_IEEE1394
#include <libraw1394/raw1394.h>
#endif
#include <unistd.h>

// wait when polling for a frame (in us):
#define IEEE1394WAIT 1

// This code is somewhat inspired from the grab_gray_image.c example
// code provided with the libdc1394 distribution by Gord Peters, and
// from the Coriander source code.

namespace
{

#ifdef HAVE_IEEE1394

  struct CameraModeInfo
  {
    VideoFormat vidfmt;
    int w;
    int h;
    int dc1394mode;
    int dc1394format;
  };

  // Figure out which of the IEEE1394 grab modes to use. We don't
  // support the various IEEE1394 MONO16 modes.
  void find1394mode(const VideoFormat vidfmt, const Dims& dims,
                    int* dc1394mode, int* dc1394format)
  {
    static const CameraModeInfo modes[] =
      {
        { VIDFMT_YUV444,  160,  120, MODE_160x120_YUV444   , FORMAT_VGA_NONCOMPRESSED },
        { VIDFMT_YUV422,  320,  240, MODE_320x240_YUV422   , FORMAT_VGA_NONCOMPRESSED },
        { VIDFMT_YUV411,  640,  480, MODE_640x480_YUV411   , FORMAT_VGA_NONCOMPRESSED },
        { VIDFMT_YUV422,  640,  480, MODE_640x480_YUV422   , FORMAT_VGA_NONCOMPRESSED },
        { VIDFMT_RGB24,   640,  480, MODE_640x480_RGB      , FORMAT_VGA_NONCOMPRESSED },
        { VIDFMT_GREY,    640,  480, MODE_640x480_MONO     , FORMAT_VGA_NONCOMPRESSED },
        { VIDFMT_YUV422,  800,  600, MODE_800x600_YUV422   , FORMAT_SVGA_NONCOMPRESSED_1 },
        { VIDFMT_RGB24,   800,  600, MODE_800x600_RGB      , FORMAT_SVGA_NONCOMPRESSED_1 },
        { VIDFMT_GREY,    800,  600, MODE_800x600_MONO     , FORMAT_SVGA_NONCOMPRESSED_1 },
        { VIDFMT_YUV422, 1024,  768, MODE_1024x768_YUV422  , FORMAT_SVGA_NONCOMPRESSED_1 },
        { VIDFMT_RGB24,  1024,  768, MODE_1024x768_RGB     , FORMAT_SVGA_NONCOMPRESSED_1 },
        { VIDFMT_GREY,   1024,  768, MODE_1024x768_MONO    , FORMAT_SVGA_NONCOMPRESSED_1 },
        { VIDFMT_YUV422, 1280,  960, MODE_1280x960_YUV422  , FORMAT_SVGA_NONCOMPRESSED_2 },
        { VIDFMT_RGB24,  1280,  960, MODE_1280x960_RGB     , FORMAT_SVGA_NONCOMPRESSED_2 },
        { VIDFMT_GREY,   1280,  960, MODE_1280x960_MONO    , FORMAT_SVGA_NONCOMPRESSED_2 },
        { VIDFMT_YUV422, 1600, 1200, MODE_1600x1200_YUV422 , FORMAT_SVGA_NONCOMPRESSED_2 },
        { VIDFMT_RGB24,  1600, 1200, MODE_1600x1200_RGB    , FORMAT_SVGA_NONCOMPRESSED_2 },
        { VIDFMT_GREY,   1600, 1200, MODE_1600x1200_MONO   , FORMAT_SVGA_NONCOMPRESSED_2 }
      };

    for (size_t i = 0; i < sizeof(modes) / sizeof(CameraModeInfo); ++i)
      {
        if (modes[i].vidfmt == vidfmt
            && modes[i].w == dims.w()
            && modes[i].h == dims.h())
          {
            *dc1394mode = modes[i].dc1394mode;
            *dc1394format = modes[i].dc1394format;
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
    int dc1394framerate;
  };

  // Figure out which framerate to use
  int find1394framerate(const float fps)
  {
    static const FramerateInfo framerates[] =
      {
        { 1.875F, FRAMERATE_1_875 },
        { 3.75F, FRAMERATE_3_75 },
        { 7.5F, FRAMERATE_7_5 },
        { 15.0F, FRAMERATE_15 },
        { 30.0F, FRAMERATE_30 },
        { 60.0F, FRAMERATE_60 }
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
    /* can't happen */ return -1;
  }

#endif // HAVE_IEEE1394

}

// ######################################################################
IEEE1394grabber::IEEE1394grabber(OptionManager& mgr,
                                 const std::string& descrName,
                                 const std::string& tagName,
                                 const ParamFlag flags) :
  FrameIstream(mgr, descrName, tagName),
  // NOTE that contrary to the common case, we may give (by default
  // value of the 'flags' param) USE_MY_VAL here when we construct the
  // OModelParam objects; that means that we push our values into the
  // ModelManager as the new default values, rather than having our
  // param take its value from the ModelManager's default
  itsDevName(&OPT_FrameGrabberDevice, this, "/dev/video1394/0", flags), // device
  itsChannel(&OPT_FrameGrabberChannel, this, 0, flags), // first firewire card
  itsSubChan(&OPT_FrameGrabberSubChan, this, 0, flags),
  itsDims(&OPT_FrameGrabberDims, this, Dims(320, 240), flags),
  itsGrabMode(&OPT_FrameGrabberMode, this, VIDFMT_YUV422, flags), // grab mode
  itsByteSwap(&OPT_FrameGrabberByteSwap, this, false, flags), // no byte-swapping
  itsFPS(&OPT_FrameGrabberFPS, this, 30.0, flags),
  itsNbuf(&OPT_FrameGrabberNbuf, this, 4, flags), // number of grab buffers
  itsBrightness(&OPT_FrameGrabberBrightness, this, 32768, flags | ALLOW_ONLINE_CHANGES),
  itsHue(&OPT_FrameGrabberHue, this, 32768, flags | ALLOW_ONLINE_CHANGES),
  itsSaturation(&OPT_FrameGrabberSaturation, this, 90, flags | ALLOW_ONLINE_CHANGES),
  itsExposure(&OPT_FrameGrabberExposure, this, 511, flags | ALLOW_ONLINE_CHANGES),
  itsSharpness(&OPT_FrameGrabberSharpness, this, 80, flags | ALLOW_ONLINE_CHANGES),
  itsWhiteBalBU(&OPT_FrameGrabberWhiteBalBU, this, 95, flags | ALLOW_ONLINE_CHANGES),
  itsWhiteBalRV(&OPT_FrameGrabberWhiteBalRV, this, 87, flags | ALLOW_ONLINE_CHANGES),
  itsGamma(&OPT_FrameGrabberGamma, this, 1, flags | ALLOW_ONLINE_CHANGES),
  itsShutter(&OPT_FrameGrabberShutter, this, 6, flags | ALLOW_ONLINE_CHANGES),
  itsGain(&OPT_FrameGrabberGain, this, 87, flags | ALLOW_ONLINE_CHANGES)
#ifdef HAVE_IEEE1394
  ,itsCameraOk(false), itsHandle(NULL)
#endif
{
  // request a bunch of camera aliases which work with IEEE1394:
  mgr.requestOptionAlias(&OPT_ALIAScamiSight);
}

// ######################################################################
void IEEE1394grabber::start1()
{
#ifndef HAVE_IEEE1394
  LFATAL("you must have ieee1394 (firewire) support and the libdc1394 "
         "library in order to use IEEE1394grabber");
#else
  int ieeegrabmode, ieeegrabformat;
  find1394mode(itsGrabMode.getVal(), itsDims.getVal(),
               &ieeegrabmode, &ieeegrabformat);

  const int framerate = find1394framerate(itsFPS.getVal());

  // create raw1394 handle:
  itsHandle = dc1394_create_handle(itsChannel.getVal());
  if (itsHandle == 0) LFATAL("Cannot create raw1394 handle");

  // list camera nodes available on bus:
  int nb = raw1394_get_nodecount(itsHandle);
  LDEBUG("Found %d nodes on port %d", nb, itsChannel.getVal());
  int nc;  // parameter 1 below is to provide terminal debug info
  nodeid_t *node = dc1394_get_camera_nodes(itsHandle, &nc, 1);
  fflush(stdout);
  int subchan = itsSubChan.getVal();
  if (nc < 1) LFATAL("No camera on bus?");
  else LDEBUG("Found %d camera(s)", nc);
  if (subchan < 0 || subchan >= nc)
    LFATAL("Invalid camera number %d", subchan);

  // check if camera is the highest node (see iso transfer bug):
  if (node[subchan] == nb - 1)
    LFATAL("You need to insmod ohci1394 attempt_root=1");

#ifdef IEEE1394NEW
  // newer versions have an extra arg do_extra_buffering (here set to 0):
  if (dc1394_dma_setup_capture(itsHandle, node[subchan], subchan,
                               ieeegrabformat, ieeegrabmode,
                               IEEE1394GRABSPEED, framerate,
                               itsNbuf.getVal(), 0, 1,
                               itsDevName.getVal().c_str(), &itsCam) !=
      DC1394_SUCCESS)
    LFATAL("Camera setup failed; see preceding error message from libdc1394.");
#else
  // old syntax:
  if (dc1394_dma_setup_capture(itsHandle, node[subchan], subchan,
                               ieeegrabformat, ieeegrabmode,
                               IEEE1394GRABSPEED, framerate,
                               itsNbuf.getVal(), 1,
                               itsDevName.getVal().c_str(), &itsCam) !=
      DC1394_SUCCESS)
    LFATAL("Camera setup failed; see preceding error message from libdc1394.");
#endif
  itsCameraOk = true;

  // set features based on ModelParam values

  if( dc1394_set_brightness( itsHandle, itsCam.node,
                             itsBrightness.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set brightness");
  if( dc1394_set_exposure( itsHandle, itsCam.node,
                           itsExposure.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set exposure");
  if( dc1394_set_sharpness( itsHandle, itsCam.node,
                            itsSharpness.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set sharpness");
  if( dc1394_set_white_balance( itsHandle, itsCam.node,
                                itsWhiteBalBU.getVal(),
                                itsWhiteBalRV.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set white balance");
  if( dc1394_set_hue( itsHandle, itsCam.node,
                      itsHue.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set hue");
  if( dc1394_set_saturation( itsHandle, itsCam.node,
                             itsSaturation.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set saturation");
  if( dc1394_set_gamma( itsHandle, itsCam.node,
                        itsGamma.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set gamma");
  if( dc1394_set_shutter( itsHandle, itsCam.node,
                          itsShutter.getVal() ) != DC1394_SUCCESS )
    LERROR("Unable to set shutter");
  if( dc1394_set_gain( itsHandle, itsCam.node,
                       itsGain.getVal() != DC1394_SUCCESS ) )
    LERROR("Unable to set gain");

  // report camera features:
  dc1394_feature_set features;
  if (dc1394_get_camera_feature_set(itsHandle, itsCam.node, &features)
      != DC1394_SUCCESS)
    LDEBUG("Unable to get camera feature set");
  else
    dc1394_print_feature_set(&features);

  // report additional features such as supported modes:
  quadlet_t val;
  if (dc1394_query_supported_formats(itsHandle, itsCam.node, &val) != DC1394_SUCCESS)
    LERROR("Cannot query supported formats");
  else
    LDEBUG("Supported formats: %lx", (unsigned long) val);

  if (dc1394_query_supported_modes(itsHandle, itsCam.node, ieeegrabformat, &val)
      != DC1394_SUCCESS)
    LERROR("Cannot query supported modes");
  else
    LDEBUG("Supported modes for format %d: %lx",
           ieeegrabformat, (unsigned long) val);

  if (dc1394_query_supported_framerates(itsHandle, itsCam.node,
                                        ieeegrabformat, ieeegrabmode, &val)
      != DC1394_SUCCESS)
    LERROR("Cannot query supported framerates");
  else
    LDEBUG("Supported framerates for format %d, mode %d: %lx",
           ieeegrabformat, ieeegrabmode, (unsigned long) val);

  // start data transfers:
  if (dc1394_start_iso_transmission(itsHandle, itsCam.node) != DC1394_SUCCESS)
    LFATAL("Cannot start data transmission");

  // just double-checking that all settings are correct:
  unsigned int ret;
  if (dc1394_get_video_format(itsHandle, itsCam.node, &ret) != DC1394_SUCCESS)
    LERROR("Cannot get video format");
  else
    LDEBUG("Current video format: %x", ret);

  if (dc1394_get_video_mode(itsHandle, itsCam.node, &ret) != DC1394_SUCCESS)
    LERROR("Cannot get video mode");
  else
    LDEBUG("Current video mode: %x", ret);

  if (dc1394_get_video_framerate(itsHandle, itsCam.node, &ret) != DC1394_SUCCESS)
    LERROR("Cannot get video framerate");
  else
    LDEBUG("Current video framerate: %x", ret);

  unsigned int ret2;
  if (dc1394_get_iso_channel_and_speed(itsHandle, itsCam.node, &ret, &ret2)
      != DC1394_SUCCESS)
    LERROR("Cannot get ISO channel and speed");
  else
    LDEBUG("Current ISO channel: %x, speed: %x", ret, ret2);

  dc1394bool_t ison;
  if (dc1394_get_iso_status(itsHandle, itsCam.node, &ison) != DC1394_SUCCESS)
    LERROR("Cannot check whether ISO transmission on");
  else
    LDEBUG("ISO transmission on: %d", ison);

  LINFO("Initialization complete and ISO transmission under way...");
#endif // HAVE_IEEE1394
}

// ######################################################################
void IEEE1394grabber::stop2()
{
#ifndef HAVE_IEEE1394
  // don't LFATAL() in stop() since it may be called in a destructor chain
  LERROR("you must have ieee1394 (firewire) support and the libdc1394 "
         "library in order to use IEEE1394grabber");
#else
  if (itsCameraOk)
    {
      dc1394_dma_unlisten(itsHandle, &itsCam);
      dc1394_dma_release_camera(itsHandle, &itsCam);
      itsCameraOk = false;
    }
  if (itsHandle) { raw1394_destroy_handle(itsHandle); itsHandle = NULL; }
#endif // HAVE_IEEE1394
}

// ######################################################################
IEEE1394grabber::~IEEE1394grabber()
{  }

// ######################################################################
GenericFrameSpec IEEE1394grabber::peekFrameSpec()
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
SimTime IEEE1394grabber::getNaturalFrameTime() const
{
  return SimTime::HERTZ(itsFPS.getVal());
}

// ######################################################################
void IEEE1394grabber::grabPrealloc(Image< PixRGB<byte> >& image,
                                   pthread_mutex_t *lock, int *count)
{
#ifndef HAVE_IEEE1394
  LFATAL("you must have ieee1394 (firewire) support and the libdc1394 "
         "library in order to use IEEE1394grabber");
#else
  ASSERT(itsHandle); ASSERT(itsCameraOk);
  ASSERT(image.getWidth() == itsDims.getVal().w() &&
         image.getHeight() == itsDims.getVal().h());
  ASSERT(itsGrabMode.getVal() == VIDFMT_YUV444);

  // capture next frame:
  while (dc1394_dma_single_capture(&itsCam) != DC1394_SUCCESS)
    {
      LINFO("Grab not ready...");
      usleep(IEEE1394WAIT);
    }

  // Convert grabbed buffer to RGB (code same as in fromVideoYUV444()
  // of Image_ColorOps.H):
  byte *data = (byte *)itsCam.dma_ring_buffer + itsCam.dma_frame_size *
    itsCam.dma_last_buffer;
  Image<PixRGB<byte> >::iterator aptr = image.beginw();
  Image<PixRGB<byte> >::iterator stop = image.endw();

  if (lock) pthread_mutex_lock(lock);
  while(aptr != stop)
    {
      // data stored as: u0, y0, v0, u1, y1, v1
      (*aptr++) = PixRGB<byte>(PixVideoYUV<double>(data[1], data[0], data[2]));
      (*aptr++) = PixRGB<byte>(PixVideoYUV<double>(data[4], data[3], data[5]));
      data += 6;
    }
  if (count) *count += 1;
  if (lock) pthread_mutex_unlock(lock);

  // free that buffer:
  if (dc1394_dma_done_with_buffer(&itsCam) != DC1394_SUCCESS)
    LERROR("Error releasing dma frame buffer");
#endif // HAVE_IEEE1394
}

// ######################################################################
GenericFrame IEEE1394grabber::readFrame()
{
  return GenericFrame(this->grabRaw());
}

// ######################################################################
VideoFrame IEEE1394grabber::grabRaw()
{
#ifndef HAVE_IEEE1394
  LFATAL("you must have ieee1394 (firewire) support and the libdc1394 "
         "library in order to use IEEE1394grabber");
  /* can't happen */ return VideoFrame();
#else
  ASSERT(itsHandle); ASSERT(itsCameraOk);

  // capture next frame:
  while (dc1394_dma_single_capture(&itsCam) != DC1394_SUCCESS)
    {
      LINFO("Grab not ready...");
      usleep(IEEE1394WAIT);
    }

  // get the buffer:
  const byte* data = itsCam.dma_ring_buffer +
    itsCam.dma_frame_size * itsCam.dma_last_buffer;

  // free that buffer:
  if (dc1394_dma_done_with_buffer(&itsCam) != DC1394_SUCCESS)
    LERROR("Error releasing dma frame buffer");

  return VideoFrame(data, itsCam.dma_frame_size, itsDims.getVal(),
                    itsGrabMode.getVal(), itsByteSwap.getVal(), false);
#endif // HAVE_IEEE1394
}

// ######################################################################
void IEEE1394grabber::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{
#ifndef HAVE_IEEE1394
  LFATAL("you must have ieee1394 (firewire) support and the libdc1394 "
         "library in order to use IEEE1394grabber");
#else

  FrameIstream::paramChanged(param, valueChanged, status);

  // just handle online changes here (i.e. changes that come while we
  // are already started); if the change happens before start() then
  // we will pick up the proper dc1394 settings in start() so we don't
  // need to handle the change here

#define HANDLE_PARAM(MODELPARAM, DC1394NAME)                    \
      if (itsHandle && valueChanged && param == &MODELPARAM)    \
        {                                                       \
          LDEBUG("online change of " #DC1394NAME " from %s",    \
                 MODELPARAM.getName().c_str());                 \
          if (dc1394_set_ ## DC1394NAME(itsHandle, itsCam.node, \
                                        MODELPARAM.getVal())    \
              != DC1394_SUCCESS)                                \
            {                                                   \
              *status = ParamClient::CHANGE_REJECTED;           \
              LERROR("Unable to set " #DC1394NAME);             \
            }                                                   \
        }

  HANDLE_PARAM(itsBrightness, brightness);
  HANDLE_PARAM(itsExposure, exposure);
  HANDLE_PARAM(itsSharpness, sharpness);
  HANDLE_PARAM(itsHue, hue);
  HANDLE_PARAM(itsSaturation, saturation);
  HANDLE_PARAM(itsGamma, gamma);
  HANDLE_PARAM(itsShutter, shutter);
  HANDLE_PARAM(itsGain, gain);

#undef HANDLE_PARAM

  if (itsHandle && valueChanged
      && (param == &itsWhiteBalBU || param == &itsWhiteBalRV))
    {
      LDEBUG("online change of white balance from %s",
             param->getName().c_str());

      if (dc1394_set_white_balance(itsHandle, itsCam.node,
                                   itsWhiteBalBU.getVal(),
                                   itsWhiteBalRV.getVal())
          != DC1394_SUCCESS)
        {
          *status = ParamClient::CHANGE_REJECTED;
          LERROR("Unable to set white balance");
        }
    }

#endif // HAVE_IEEE1394
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
