/*!@file Devices/V4Lgrabber.C Interface with a video4linux frame grabber */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/V4Lgrabber.C $
// $Id: V4Lgrabber.C 10794 2009-02-08 06:21:09Z itti $
//

#ifdef HAVE_LINUX_VIDEODEV_H

#include "Devices/V4Lgrabber.H"

#include "Component/OptionManager.H" // for option alias requests
#include "Devices/DeviceOpts.H"
#include "Image/ColorOps.H"
#include "Raster/GenericFrame.H"
#include "Util/Assert.H"
#include "Util/log.H"
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
  //! A V4L palette entry
  struct V4LPalette
  {
    int pal;             //!< palette
    int depth;           //!< depth
    const char* name;    //!< name of palette e.g. "rgb24"
    VideoFormat vidformat; //!< Frame grabber mode
  };

  //! array of all known palettes
  //! high preference modes have lower indices
  struct V4LPalette palettes[12] =
    {
      { VIDEO_PALETTE_RGB24,         24, "bgr24", VIDFMT_RGB24 },
      { VIDEO_PALETTE_RGB24 | 0x80,  24, "rgb24", VIDFMT_RGB24 },
      { VIDEO_PALETTE_RGB32,         32, "bgr32", VIDFMT_RGB32 },
      { VIDEO_PALETTE_RGB32 | 0x80,  32, "rgb32", VIDFMT_RGB32 },
      { VIDEO_PALETTE_YUYV,          24, "yuyv 16bpp", VIDFMT_YUYV  },
      { VIDEO_PALETTE_YUV422,        24, "yuv422 16bpp", VIDFMT_YUV422 },
      { VIDEO_PALETTE_RGB565,        16, "rgb565", VIDFMT_RGB565 },
      { VIDEO_PALETTE_RGB555,        15, "rgb555", VIDFMT_RGB555 },
      { VIDEO_PALETTE_YUV420,        24, "yuv420 12bpp", VIDFMT_YUV420 },
      { VIDEO_PALETTE_YUV420P,       24, "yuv420 planar 12bpp", VIDFMT_YUV420P },
      { VIDEO_PALETTE_GREY,          8, "grayscale 8bpp", VIDFMT_GREY },
      { -1,                          -1, 0, VideoFormat(-1) }
    };

  int ioctl_nointr(int d, int req, void* mem)
  {
    int result = 0;
    do { result = ioctl(d, req, mem); }
    while ((result < 0) && (errno == EINTR));
    return result;
  }
}

// ######################################################################
V4Lgrabber::V4Lgrabber(OptionManager& mgr, const std::string& descrName,
                       const std::string& tagName,
                       const ParamFlag flags) :
  FrameIstream(mgr, descrName, tagName),
  // NOTE that contrary to the common case, we may give (by default
  // value of 'flags') USE_MY_VAL here when we construct the
  // OModelParam objects; that means that we push our values into the
  // ModelManager as the new default values, rather than having our
  // param take its value from the ModelManager's default
  itsDevName(&OPT_FrameGrabberDevice, this, "/dev/video0", flags), // V4l device
  itsChannel(&OPT_FrameGrabberChannel, this, 1, flags),  // composite input
  itsDims(&OPT_FrameGrabberDims, this, Dims(320, 240), flags),
  itsGrabMode(&OPT_FrameGrabberMode, this, VIDFMT_RGB24, flags), // 24-bit rgb grabbing
  itsByteSwap(&OPT_FrameGrabberByteSwap, this, true, flags), // use byte-swapping
  itsBrightness(&OPT_FrameGrabberBrightness, this, 32768, flags | ALLOW_ONLINE_CHANGES),
  itsHue(&OPT_FrameGrabberHue, this, 32768, flags | ALLOW_ONLINE_CHANGES),
  itsColour(&OPT_FrameGrabberColour, this, 32768, flags | ALLOW_ONLINE_CHANGES),
  itsContrast(&OPT_FrameGrabberContrast, this, 32768, flags | ALLOW_ONLINE_CHANGES),
  itsWhiteness(&OPT_FrameGrabberWhiteness, this, 32768, flags | ALLOW_ONLINE_CHANGES),
  itsStreamingMode(&OPT_FrameGrabberStreaming, this),
  itsFd(-1),
  itsMmapBuf(NULL),
  itsReadBuf(),
  itsTotalBufSize(0),
  itsNumBufFrames(0),
  itsCurrentFrame(0),
  itsGrabbing(NULL),
  itsFrameTime(SimTime::ZERO()),
  itsListener(),
  itsStreamStarted(false)
{
  // request a bunch of camera aliases which work with V4L:
  mgr.requestOptionAlias(&OPT_ALIAScamBttv);
}

// ######################################################################
void V4Lgrabber::start1()
{
  itsFd = open(itsDevName.getVal().c_str(), O_RDWR | O_NONBLOCK);
  if (itsFd == -1) PLFATAL("Cannot open V4L device %s",
                           itsDevName.getVal().c_str());

  // get frame grabber capabilities:
  struct video_capability vc;
  if (ioctl_nointr(itsFd, VIDIOCGCAP, &vc) < 0)
    IDPLFATAL("Cannot get V4L device capabilities");
  IDLINFO("FrameGrabber board name is: %s", vc.name);
  IDLINFO("maxwidth = %d, maxheight = %d", vc.maxwidth, vc.maxheight);
  if (itsDims.getVal().w() > vc.maxwidth ||
      itsDims.getVal().h() > vc.maxheight)
    IDLFATAL("Requested grab size %dx%d too large",
             itsDims.getVal().w(), itsDims.getVal().h());

  // select input channel & norm:
  struct video_channel vch;
  vch.channel = itsChannel.getVal();
  if (ioctl_nointr(itsFd, VIDIOCGCHAN, &vch) < 0)
    IDPLERROR("Cannot get V4L device channel information");
  vch.norm = VIDEO_MODE_NTSC;   // set NTSC norm
  vch.type = VIDEO_TYPE_CAMERA; // camera input
  IDLINFO("Channel %d is '%s' [norm %d]",
           vch.channel, vch.name, vch.norm);
  if (ioctl_nointr(itsFd, VIDIOCSCHAN, &vch) < 0)
    IDPLERROR("Cannot set V4L device channel information");

  switch (vch.norm)
    {
    case VIDEO_MODE_PAL:
      itsFrameTime = SimTime::HERTZ(25.0);
      break;

    case VIDEO_MODE_NTSC:
      itsFrameTime = SimTime::HERTZ(29.97);
      break;

    case VIDEO_MODE_SECAM:
      itsFrameTime = SimTime::HERTZ(25.0);
      break;

    default:
      itsFrameTime = SimTime::ZERO();
      break;
    }

  // get specs of video buffer:
  struct video_mbuf vmb;
  if (ioctl_nointr(itsFd, VIDIOCGMBUF, &vmb) < 0)
    IDPLFATAL("Cannot get V4L device buffer");

  IDLINFO("video Mbuf: 0x%x bytes, %d frames", vmb.size, vmb.frames);
  for (int i = 0; i < vmb.frames; ++i)
    IDLINFO("buffer offset[%d] = %d", i, vmb.offsets[i]);

  // get the picture properties
  struct video_picture vp;
  if (ioctl_nointr(itsFd, VIDIOCGPICT, &vp) != 0)
    IDPLFATAL("ioctl(VIDIOCSPICT) get picture properties failed");

  // get ready for capture, for all frames in buffer:
  switch (itsGrabMode.getVal()) {
  case VIDFMT_GREY: itsVmmInfo.format = VIDEO_PALETTE_GREY; break;
  case VIDFMT_RAW: itsVmmInfo.format = VIDEO_PALETTE_RAW; break;
  case VIDFMT_RGB555: itsVmmInfo.format = VIDEO_PALETTE_RGB555; break;
  case VIDFMT_RGB565: itsVmmInfo.format = VIDEO_PALETTE_RGB565; break;
  case VIDFMT_RGB24: itsVmmInfo.format = VIDEO_PALETTE_RGB24; break;
  case VIDFMT_RGB32: itsVmmInfo.format = VIDEO_PALETTE_RGB32; break;
  case VIDFMT_YUYV: itsVmmInfo.format = VIDEO_PALETTE_YUYV; break;
  case VIDFMT_UYVY: itsVmmInfo.format = VIDEO_PALETTE_UYVY; break;
  case VIDFMT_YUV422: itsVmmInfo.format = VIDEO_PALETTE_YUV422; break;
  case VIDFMT_YUV411: itsVmmInfo.format = VIDEO_PALETTE_YUV411; break;
  case VIDFMT_YUV420: itsVmmInfo.format = VIDEO_PALETTE_YUV420; break;
  case VIDFMT_YUV422P: itsVmmInfo.format = VIDEO_PALETTE_YUV422P; break;
  case VIDFMT_YUV411P: itsVmmInfo.format = VIDEO_PALETTE_YUV411P; break;
  case VIDFMT_YUV420P: itsVmmInfo.format = VIDEO_PALETTE_YUV420P; break;
  case VIDFMT_YUV410P: itsVmmInfo.format = VIDEO_PALETTE_YUV410P; break;
  case VIDFMT_AUTO:
    // Auto selection of grab mode:
    struct V4LPalette* pal;
    LINFO("Probing for supported palettes:");

#define CHECK_PALETTE(p)                        \
        {                                       \
          vp.palette = p;                       \
          vp.depth = 32;                        \
          ioctl_nointr(itsFd, VIDIOCSPICT, &vp);\
          ioctl_nointr(itsFd, VIDIOCGPICT, &vp);\
          if (vp.palette == p)                  \
            LINFO("  %-22s supported", #p);     \
          else                                  \
            LINFO("  %-22s NOT supported", #p); \
        }

    CHECK_PALETTE(VIDEO_PALETTE_GREY);
    CHECK_PALETTE(VIDEO_PALETTE_HI240);
    CHECK_PALETTE(VIDEO_PALETTE_RGB565);
    CHECK_PALETTE(VIDEO_PALETTE_RGB24);
    CHECK_PALETTE(VIDEO_PALETTE_RGB32);
    CHECK_PALETTE(VIDEO_PALETTE_RGB555);
    CHECK_PALETTE(VIDEO_PALETTE_YUV422);
    CHECK_PALETTE(VIDEO_PALETTE_YUYV);
    CHECK_PALETTE(VIDEO_PALETTE_UYVY);
    CHECK_PALETTE(VIDEO_PALETTE_YUV420);
    CHECK_PALETTE(VIDEO_PALETTE_YUV411);
    CHECK_PALETTE(VIDEO_PALETTE_RAW);
    CHECK_PALETTE(VIDEO_PALETTE_YUV422P);
    CHECK_PALETTE(VIDEO_PALETTE_YUV411P);
    CHECK_PALETTE(VIDEO_PALETTE_YUV420P);
    CHECK_PALETTE(VIDEO_PALETTE_YUV410P);

#undef CHECK_PALETTE

    // Brutal, loop through all available modes
    for (pal = &palettes[0]; pal->pal >= 0; ++pal)
      {
        vp.palette = pal->pal;
        vp.depth = pal->depth;
        ioctl_nointr(itsFd, VIDIOCSPICT, &vp);
        ioctl_nointr(itsFd, VIDIOCGPICT, &vp);
        if (vp.palette == pal->pal)
          {
            LINFO("  Using palette \"%s\" with depth %u",
                  pal->name, vp.depth);
            // hack
            itsGrabMode.setVal(pal->vidformat);
            itsVmmInfo.format = vp.palette;
            break;
          }
        else
          LINFO("  Palette \"%s\" not supported", pal->name);
      }

    if (pal->pal < 0)
      IDLFATAL("Auto palette selection failed - try setting manually.");
    break;
  default:
    LFATAL("Unsupported grab mode");
  }

  // get ready to grab frames, starting with buffer/frame 0:
  itsNumBufFrames = vmb.frames;
  itsTotalBufSize = vmb.size;
  itsCurrentFrame = 0;
  itsGrabbing = new bool[itsNumBufFrames];
  for (int i = 0; i < itsNumBufFrames; ++i) itsGrabbing[i] = false;
  itsVmmInfo.width = itsDims.getVal().w();
  itsVmmInfo.height = itsDims.getVal().h();
  itsVmmInfo.frame = 0;

  // decide on mmap'ed or read() access:
  if (ioctl_nointr(itsFd, VIDIOCGMBUF, &vmb) != -1)
    {
      IDLINFO("Using mmap'ed image capture");
      // setup mmap'ed access to the video buffer:
      itsMmapBuf =
        static_cast<byte*>(mmap((void*)0, vmb.size,
                                PROT_READ|PROT_WRITE,
                                MAP_SHARED, itsFd, 0));

      if (itsMmapBuf == MAP_FAILED)
        IDPLFATAL("mmap failed");

      itsReadBuf = Image<byte>();
    }
  else
    {
      IDLINFO("Using read() image capture");
      itsMmapBuf = NULL;

      itsReadBuf = Image<byte>(getFrameSize(itsGrabMode.getVal(),
                                            itsDims.getVal()),
                               1, NO_INIT);
    }

  // set picture properties
  vp.brightness = itsBrightness.getVal();
  vp.hue = itsHue.getVal();
  vp.colour = itsColour.getVal();
  vp.contrast = itsContrast.getVal();
  vp.whiteness = itsWhiteness.getVal();
  vp.palette = itsVmmInfo.format;
  LINFO("bright=%u hue=%u color=%u contrast=%u white=%u depth=%u palette=%u",
        vp.brightness, vp.hue, vp.colour, vp.contrast,
        vp.whiteness, vp.depth, vp.palette);
  if (ioctl_nointr(itsFd, VIDIOCSPICT, &vp) != 0)
    IDPLERROR("ioctl(VIDIOCSPICT) set picture properties failed");
}

// ######################################################################
void V4Lgrabber::stop2()
{
  if (itsMmapBuf) { munmap(itsMmapBuf, itsTotalBufSize); itsMmapBuf = NULL; }
  if (itsFd >= 0) { close(itsFd); itsFd = -1; }
  if (itsGrabbing) { delete [] itsGrabbing; itsGrabbing = NULL; }
  itsReadBuf = Image<byte>();
  itsStreamStarted = false;
}

// ######################################################################
V4Lgrabber::~V4Lgrabber()
{  }

// ######################################################################
void V4Lgrabber::setListener(rutz::shared_ptr<FrameListener> listener)
{
  itsListener = listener;
}

// ######################################################################
void V4Lgrabber::startStream()
{
  // reset itsStreamStarted so that we wait for any pending frames to
  // be grabbed, then start fresh grabbing requests:
  itsStreamStarted = false;

  this->restartStream();
}

// ######################################################################
SimTime V4Lgrabber::getNaturalFrameTime() const
{
  return itsFrameTime;
}

// ######################################################################
GenericFrameSpec V4Lgrabber::peekFrameSpec()
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
GenericFrame V4Lgrabber::readFrame()
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
VideoFrame V4Lgrabber::grabRaw()
{
  byte* result = 0;
  if (itsMmapBuf) // mmap interface
    {
      this->restartStream();

      result = itsMmapBuf + itsCurrentFrame * (itsTotalBufSize / itsNumBufFrames);

      // are we already grabbing buffer 'itsCurrentFrame'? otherwise,
      // start the grab now:
      if (itsGrabbing[itsCurrentFrame] == false)
        {
          itsVmmInfo.frame = itsCurrentFrame;
          itsGrabbing[itsCurrentFrame] = true;
          if (ioctl_nointr(itsFd, VIDIOCMCAPTURE, &itsVmmInfo) < 0)
            IDPLFATAL("VIDIOCMCAPTURE (frame %d)", itsCurrentFrame);
        }

      // wait until buffer 'itsCurrentFrame' has been fully captured:
      if (ioctl_nointr(itsFd, VIDIOCSYNC, &itsCurrentFrame) < 0)
        IDPLFATAL("VIDIOCSYNC (frame %d)", itsCurrentFrame);
      itsGrabbing[itsCurrentFrame] = false;

      // get ready for capture of that frame again (for later):
      itsVmmInfo.frame = itsCurrentFrame;
      itsGrabbing[itsCurrentFrame] = true;
      if (ioctl_nointr(itsFd, VIDIOCMCAPTURE, &itsVmmInfo) < 0)
        IDPLFATAL("VIDIOCMCAPTURE (frame %d)", itsCurrentFrame);
    }
  else // read() interface
    {
      ASSERT(itsReadBuf.initialized());
      const ssize_t nbytes =
        read(itsFd, itsReadBuf.getArrayPtr(), itsReadBuf.getSize());
      if (nbytes < 0)
        IDPLFATAL("read() failed");
      PLDEBUG("got %zd bytes", nbytes);
      result = itsReadBuf.getArrayPtr();
    }

  // switch to another frame:
  ++itsCurrentFrame;
  if (itsCurrentFrame >= itsNumBufFrames) itsCurrentFrame = 0;

  // return pointer to last-grabbed frame. You have a bit of time to
  // get a hold of the data but beware that an order to capture that
  // buffer has already been issued, so as soon as the grabber gets to
  // it, it will overwrite this buffer with a new frame:
  ASSERT(result != 0);
  VideoFrame frame(result, (itsTotalBufSize / itsNumBufFrames),
                   itsDims.getVal(),
                   itsGrabMode.getVal(), itsByteSwap.getVal(),
                   /* strictLength = */ false);

  return frame;
}

// ######################################################################
void V4Lgrabber::paramChanged(ModelParamBase* const param,
                                   const bool valueChanged,
                                   ParamClient::ChangeStatus* status)
{

  FrameIstream::paramChanged(param, valueChanged, status);

  // set picture properties
  if (valueChanged)
  {
    //FIXME
    /*struct video_picture vp;
    vp.brightness = itsBrightness.getVal();
    vp.hue = itsHue.getVal();
    vp.colour = itsColour.getVal();
    vp.contrast = itsContrast.getVal();
    vp.whiteness = itsWhiteness.getVal();
    vp.palette = itsVmmInfo.format;
    LINFO("bright=%u hue=%u color=%u contrast=%u white=%u depth=%u palette=%u",
        vp.brightness, vp.hue, vp.colour, vp.contrast,
        vp.whiteness, vp.depth, vp.palette);
    if (ioctl_nointr(itsFd, VIDIOCSPICT, &vp) != 0)
    {
      LERROR("ioctl(VIDIOCSPICT) set picture properties failed");
      *status = ParamClient::CHANGE_REJECTED;
    }*/
  }



}


// ######################################################################
VideoFrame V4Lgrabber::grabSingleRaw()
{
  byte* result = 0;
  if (itsMmapBuf) // mmap interface
    {
      itsCurrentFrame = 0;
      result = itsMmapBuf;

      // are we already grabbing buffer 'itsCurrentFrame'? otherwise,
      // start the grab now:
      if (itsGrabbing[itsCurrentFrame] == false)
        {
          itsVmmInfo.frame = itsCurrentFrame;
          itsGrabbing[itsCurrentFrame] = true;
          if (ioctl_nointr(itsFd, VIDIOCMCAPTURE, &itsVmmInfo) < 0)
            IDPLFATAL("VIDIOCMCAPTURE (frame %d)", itsCurrentFrame);
        }

      // wait until grab is complete:
      if (ioctl_nointr(itsFd, VIDIOCSYNC, &itsCurrentFrame) < 0)
        IDPLFATAL("VIDIOCSYNC (frame %d)", itsCurrentFrame);
      itsGrabbing[itsCurrentFrame] = false;
    }
  else // read() interface
    {
      ASSERT(itsReadBuf.initialized());
      const ssize_t nbytes =
        read(itsFd, itsReadBuf.getArrayPtr(), itsReadBuf.getSize());
      if (nbytes < 0)
        IDPLFATAL("read() failed (frame %d)", itsCurrentFrame);
      PLDEBUG("got %zd bytes", nbytes);
      result = itsReadBuf.getArrayPtr();
    }

  // return grabbed & converted frame:
  VideoFrame frame(result, (itsTotalBufSize / itsNumBufFrames),
                   itsDims.getVal(),
                   itsGrabMode.getVal(), itsByteSwap.getVal(),
                   /* strictLength = */ false);

  return frame;
}

// ######################################################################
void V4Lgrabber::restartStream()
{
  if (itsStreamingMode.getVal()
      && itsMmapBuf
      && !itsStreamStarted)
    {
      for (int i = 0; i < itsNumBufFrames; ++i)
        {
          // are we already grabbing buffer i?  if so, wait for grab
          // to finished if we've requested resync (if we don't do
          // this, then the first few frames after a restart will be
          // wrong -- they'll come too quickly and they'll be too old,
          // since the v4l driver doesn't grab new frames into buffers
          // until the old frames have been retrieved):
          if (itsGrabbing[i] == true)
            {
              if (ioctl_nointr(itsFd, VIDIOCSYNC, &i) < 0)
                IDPLFATAL("VIDIOCSYNC (frame %d)", i);
              LINFO("flushed buffer %d", i);
              itsGrabbing[i] = false;
            }
        }

      for (int i = 0; i < itsNumBufFrames; ++i)
        {
          // now start a fresh grab for buffer i:
          itsVmmInfo.frame = i; itsGrabbing[i] = true;
          if (ioctl_nointr(itsFd, VIDIOCMCAPTURE, &itsVmmInfo) < 0)
            IDPLFATAL("VIDIOCMCAPTURE (frame %d)", i);
        }

      itsCurrentFrame = 0;

      itsStreamStarted = true;
    }
}

#endif // HAVE_LINUX_VIDEODEV_H

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
