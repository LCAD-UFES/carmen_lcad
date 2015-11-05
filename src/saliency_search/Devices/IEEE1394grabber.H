/*!@file Devices/IEEE1394grabber.H Definition and access functions for
  FireWire grabber */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/IEEE1394grabber.H $
// $Id: IEEE1394grabber.H 10345 2008-10-15 17:27:10Z icore $
//

#ifndef IEEE1394GRABBER_H_DEFINED
#define IEEE1394GRABBER_H_DEFINED

#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Transport/FrameIstream.H"
#include "Util/Types.H"
#include "Video/VideoFormat.H"

#ifdef HAVE_IEEE1394
#include <libdc1394/dc1394_control.h>
#endif

class VideoFrame;
template <class T> class PixRGB;
template <class T> class Image;

//! Firewire bus speed to use
#define IEEE1394GRABSPEED SPEED_400

//! Definition and access functions for FireWire digital camera frame capture
/*! This class provides a trivial interface to FireWire digital
  cameras (i.e., web cams that send raw frame data over the firewire
  bus, not DV camcorders that send compressed video streams over the
  firewire bus).  All the low-level setup is done during
  construction. The user only needs to call readFrame() or readRGB()
  to capture an image.

  After each grab, the next grab is initiated, and will be ready 33ms
  later (or one frame later if not NTSC). If you call readFrame()
  again before 33ms have elapsed, it will block until the next frame
  is available. If you call it too late, you will have missed the
  latest frame, and readFrame() will block until the next frame is
  acquired.

  So a good strategy is to use readFrame() to not only grab but also
  to synchronize your code with the video rate (30 frames/s if NTSC).
  Typically, then, you would have a main loop that first grabs and
  then does various processing that is guaranteed to take less than
  33ms.  You do not need to insert any pause after that processing to
  obtain a stable framerate; just finish your main loop, and the next
  call to readFrame() (at the next iteration) will block until exactly
  one frame has passed since it was last called. See how this is done,
  for example, in pvisionTCP-master.C or test-grab.C

  Our Unibrain Fire-i rev 1.2 camera supports the following (use
  byteswap = false):

  160 x 120: YUV444 (30, 15, 7.5 fps)
  320 x 240: YUV422 (30, 15, 7.5, 3.75 fps)
  640 x 480: YUV411 (30, 15, 7.5, 3.75 fps)
  640 x 480: YUV422 (15, 7.5, 3.75 fps)
  640 x 480: RGB24 (15, 7.5, 3.75 fps)
  640 x 480: GREY (30, 15, 7.5, 3.75 fps)

  To use this, you must have libdc1394 0.9.0 or later and libraw1394
  0.9.0 or later installed (and your need the -devel RPMS installed as
  well to compile this code). Make sure you also have:
  mknod -m 777 /dev/raw1394 c 171 0
  mkdir /dev/video1394
  mknod -m 777 /dev/video1394/0 c 171 16
*/

class IEEE1394grabber : public FrameIstream
{
public:
  //! Constructor
  IEEE1394grabber(OptionManager& mgr,
                  const std::string& descrName="IEEE1394 Frame Grabber Driver",
                  const std::string& tagName = "IEEE1394FrameGrabber",
                  const ParamFlag flags = USE_MY_VAL);

  //! Destructor
  virtual ~IEEE1394grabber();

  //! Return the specifications of the next frame to be returned
  virtual GenericFrameSpec peekFrameSpec();

  //! Get the inter-frame time that matches our video mode
  virtual SimTime getNaturalFrameTime() const;

  //! Grab video into a pre-allocated buffer
  /*! This is faster than readFrame(), as here we will not allocate a
    new image for each grab, but rather we use an existing image. If
    not NULL, mutex 'lock' will be locked while the contents of image
    are being modified, and if not NULL count will be incremented by
    one. For IEEE1394, only YUV444 is supported. */
  void grabPrealloc(Image< PixRGB<byte> >& image,
                    pthread_mutex_t *lock = NULL, int *count = NULL);

  //! Get the next frame from the frame-grabber
  /*! Returns grabbed frame. This call will block until a frame is
      ready and has been grabbed.

      Beware that the integrity of the GenericFrame object may not
      last "very long"; basically, try to be finished using the
      GenericFrame object before you attempt to grab the next frame in
      the stream. If you need it for longer than that, then you should
       use GenericFrame::deepCopyOf() to make a copy of the frame that
     can be safely held indefinitely. */
  virtual GenericFrame readFrame();

protected:
  //! Grab raw data
  /*! Don't call this directly; use readFrame() instead. */
  VideoFrame grabRaw();

  //! get started
  virtual void start1();

  //! get stopped
  virtual void stop2();

private:
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! device name of the /dev/ entry for the grabber device
  OModelParam<std::string> itsDevName;

  //! input channel to use
  OModelParam<int> itsChannel;

  //! input subchannel to use
  OModelParam<int> itsSubChan;

  //! width of grabbed frames
  OModelParam<Dims> itsDims;

  //! grab mode that the hardware should use
  /*! Grabbed frames will internally be converted to Image<
    PixRGB<byte> > whatever that mode is, but playing with it may
    influence image quality, maximum achievable framerate, and amounts
    of CPU doing those conversions to RGB. */
  OModelParam<VideoFormat> itsGrabMode;

  //! determines whether byte swapping is done during conversion to RGB
  OModelParam<bool> itsByteSwap;

  //! frames per second
  OModelParam<float> itsFPS;

  //! number of frame buffers kept internally
  OModelParam<int> itsNbuf;

  //! brightness - highly dependent on your driver
  OModelParam<int> itsBrightness;

  //! hue - highly dependent on your driver
  OModelParam<int> itsHue;

  //! saturation - highly dependent on your driver
  OModelParam<int> itsSaturation;

  //! exposure - highly dependent on your driver
  OModelParam<int> itsExposure;

  //! sharpness - highly dependent on your driver
  OModelParam<int> itsSharpness;

  //! white balance (B/U value) - highly dependent on your driver
  OModelParam<int> itsWhiteBalBU;

  //! white balance (R/V value) - highly dependent on your driver
  OModelParam<int> itsWhiteBalRV;

  //! gamma  - highly dependent on your driver
  OModelParam<int> itsGamma;

  //! shutter - highly dependent on your driver
  OModelParam<int> itsShutter;

  //! gain - highly dependent on your driver
  OModelParam<int> itsGain;

// check whether the configure script found libraw1394; if not, then
// just stub out the core of the IEEE1394grabber class and issue
// LFATAL()s if somebody actually tries to use it
#ifdef HAVE_IEEE1394
  bool itsCameraOk;           // the camera is initialized and transmitting
  raw1394handle_t itsHandle;   // raw 1394 OHCI handle
  dc1394_cameracapture itsCam; // our camera
#endif // HAVE_IEEE1394

};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
