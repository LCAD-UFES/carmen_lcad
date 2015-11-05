/*!@file Devices/V4L2grabber.H Definition and access functions for
  video4linux v2 grabber */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/V4L2grabber.H $
// $Id: V4L2grabber.H 15162 2012-02-18 03:08:39Z kai $
//

#ifndef V4L2GRABBER_H_DEFINED
#define V4L2GRABBER_H_DEFINED

#ifdef HAVE_LINUX_VIDEODEV2_H

#include <linux/videodev2.h>

#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Transport/FrameIstream.H"
#include "Util/SimTime.H"
#include "Util/Types.H"
#include "Video/VideoFormat.H"

class VideoFrame;

//! Definition and access functions for video4linux2 frame grabber
/*! This class provides a trivial interface to Video4Linux2 frame
  grabbers.  All the low-level setup is done during construction. The
  user only needs to call readRGB() or readFrame() to capture an
  image. Shared memory and DMA directly from the grabber hardware into
  our memory is used if supported by the grabber and driver. This
  should work with any framegrabber that is supported by
  Video4Linux. We use Bt878-based boards.

  After each grab, the next grab is initiated, and will be ready 33ms
  later (or one frame later if not NTSC). If you call readFrame() again
  before 33ms have elapsed, it will block until the next frame is
  available. If you call it too late, you will have missed the latest
  frame, and readFrame() will block until the next frame is acquired.

  So a good strategy is to use readFrame() to not only grab but also
  to synchronize your code with the video rate (30 frames/s if NTSC).
  Typically, then, you would have a main loop that first grabs and
  then does various processing that is guaranteed to take less than
  33ms.  You do not need to insert any pause after that processing to
  obtain a stable framerate; just finish your main loop, and the next
  call to readFrame() (at the next iteration) will block until exactly
  one frame has passed since it was last called. See how this is done,
  for example, in pvisionTCP-master.C or test-grab.C  */

class V4L2grabber : public FrameIstream
{
public:
  //! Constructor
  V4L2grabber(OptionManager& mgr,
              const std::string& descrName = "V4L Frame Grabber Driver",
              const std::string& tagName = "V4LFrameGrabber",
              const ParamFlag flags = USE_MY_VAL);

  //! Destructor
  virtual ~V4L2grabber();

  /// Install a FrameListener
  /** We call the listener's onRawFrame() inside each readFrame(). */
  virtual void setListener(rutz::shared_ptr<FrameListener> listener);

  //! Get a streaming grab started
  /*! This will instruct the grabber to grab all of its possible
    buffers, one after the other. Typically you should call this just
    before you start reading frames. If the option
    --framegrabber-streaming=false is set, then this call is a (safe)
    no-op. */
  virtual void startStream();

  //! Return the specifications of the next frame to be returned
  virtual GenericFrameSpec peekFrameSpec();

  //! Get the inter-frame time that matches our video mode
  virtual SimTime getNaturalFrameTime() const;

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
  //! Grab a raw VideoFrame
  /*! Don't call this directly; use readFrame() instead. */
  VideoFrame grabRaw();

  //! Grab a single VideoFrame
  /*! Don't call this directly; use readFrame() instead.

      This function will work better with single-shot grabs (as
      opposed to video) and USB cameras. */
  VideoFrame grabSingleRaw();

  //! get started
  virtual void start1();

  //! get stopped
  virtual void stop2();


private:
  //! Set the camera parameters on the fly
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! Auxiliary helper for startStream()
  void restartStream();

  //! device name of the /dev/ entry for the grabber device
  OModelParam<std::string> itsDevName;

  //! input channel to use
  OModelParam<int> itsChannel;

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

  //! whether to operate in streaming or single-frame mode
  OModelParam<bool> itsStreamingMode;

  //! number of frame buffers kept internally
  OModelParam<int> itsNbuf;

  //! determines whether flip the image vertically
  OModelParam<bool> itsVflip;

  //! determines whether flip the image horizontally
  OModelParam<bool> itsHflip;

  int itsFd;                //!< file descriptor for video device
  byte** itsMmapBuf;        //!< mmap'ed buffer with the video frames
  int *itsMmapBufSize;      //!< size in bytes of each mmap'ed buffer
  Image<byte> itsReadBuf;   //!< raw buffer for use with read() interface
  int itsCurrentFrame;      //!< current frame
  bool* itsGrabbing;        //!< which bufs have we told the hardware to grab?

  SimTime itsFrameTime;     //!< inter-frame time for our video mode

  rutz::shared_ptr<FrameListener> itsListener;

  bool itsStreamStarted;    //!< whether startStream() has been called

  // management of V4L2 controls:
  class V4L2grabberControlBase {
  public:
    V4L2grabberControlBase(const int id, const v4l2_ctrl_type typ) : cid(id), ctype(typ) { }
    virtual ~V4L2grabberControlBase() { }

    uint cid; // use V4L2_CTRL_ID2CLASS(id) to get the control class
    v4l2_ctrl_type ctype;
  };

  template <class T> class V4L2grabberControl : public V4L2grabberControlBase {
  public:
    V4L2grabberControl(const ModelOptionDef *opt, ParamClient *client, const T& initval,
                       const ParamFlag flags, const int id, const v4l2_ctrl_type typ) :
      V4L2grabberControlBase(id, typ), param(opt, client, initval, flags | ALLOW_ONLINE_CHANGES) { }

    virtual ~V4L2grabberControl() { }

    OModelParam<T> param;
  };

  std::vector<rutz::shared_ptr<V4L2grabberControlBase> > itsControls;

  bool itsCanMMap; // the device has mmap capability
  bool itsCanRW;   // the device has RW capability


  void openDevice();
  void closeDevice();
  void addControl(const struct v4l2_queryctrl & crtl);

  ParamFlag itsOptionFlags;
};

#endif // HAVE_LINUX_VIDEODEV2_H

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
