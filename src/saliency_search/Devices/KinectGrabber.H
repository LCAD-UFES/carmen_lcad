/*!@file Devices/KinectGrabber.H Definition and access functions for Kinect grabber */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/KinectGrabber.H $
// $Id: KinectGrabber.H 14046 2010-09-25 05:58:24Z itti $
//

#ifndef KINECTGRABBER_H_DEFINED
#define KINECTGRABBER_H_DEFINED

#ifdef INVT_HAVE_LIBFREENECT

#include <libfreenect.hpp>

#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Image/Pixels.H"
#include "Transport/FrameIstream.H"
#include "Raster/GenericFrame.H"
#include "Util/SimTime.H"
#include "Util/Types.H"
#include "Video/VideoFormat.H"
#include "rutz/mutex.h"

#include <unistd.h> // for usleep()

//! Helper class that implements frame callbacks
class FreenectDev : public Freenect::FreenectDevice {
public:
  //! Constructor
  FreenectDev(freenect_context *_ctx, int _index) : FreenectDevice(_ctx, _index) {
    if (0 != pthread_mutex_init(&itsMutex, NULL)) LFATAL("pthread_mutex_init() failed");
  }

  //! Destructor
  virtual ~FreenectDev() {
    if (0 != pthread_mutex_destroy(&itsMutex)) LERROR("pthread_mutex_destroy() failed");
 }

  //! Called by the Kinect driver when an RBG image is received
  virtual void VideoCallback(void *rgb, uint32_t timestamp)
  {
    Image<PixRGB<byte> > img((PixRGB<byte>*)(rgb), FREENECT_FRAME_W, FREENECT_FRAME_H);
    {
      GVX_MUTEX_LOCK(&itsMutex);
      itsColorImage = img;
    }
  }

  //! Called by the Kinect driver when a depth image is received
  virtual void DepthCallback(void *depth, uint32_t timestamp)
  {
    Image<uint16> img(static_cast<uint16*>(depth), FREENECT_FRAME_W, FREENECT_FRAME_H);
    {
      GVX_MUTEX_LOCK(&itsMutex);
      itsDepthImage = img;
    }
  }

  //! Called by users to collect frames
  GenericFrame grab() {
    // wait until both the color and depth images are available:
    while(true) {
      {
        GVX_MUTEX_LOCK(&itsMutex);
        if (itsColorImage.initialized() && itsDepthImage.initialized()) break; // got both images
      }
      usleep(3000);
    }

    GVX_MUTEX_LOCK(&itsMutex);
    GenericFrame frame(itsColorImage, itsDepthImage);

    // invalidate our buffers:
    itsColorImage.freeMem(); itsDepthImage.freeMem();

    return frame;
  }

private:
  Image<PixRGB<byte> > itsColorImage;
  Image<uint16> itsDepthImage;
  mutable pthread_mutex_t itsMutex;
};

//! Definition and access functions for Kinect frame grabber
class KinectGrabber : public FrameIstream
{
public:
  //! Constructor
  KinectGrabber(OptionManager& mgr,
                const std::string& descrName = "Kinect Frame Grabber Driver",
                const std::string& tagName = "KinectFrameGrabber",
                const ParamFlag flags = USE_MY_VAL);

  //! Destructor
  virtual ~KinectGrabber();

  /// Install a FrameListener
  /** We call the listener's onRawFrame() inside each readFrame(). */
  virtual void setListener(rutz::shared_ptr<FrameListener> listener);

  //! Return the specifications of the next frame to be returned
  virtual GenericFrameSpec peekFrameSpec();

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
  //! get started
  virtual void start1();

  //! get stopped
  virtual void stop2();


private:
  OModelParam<Dims> itsDims; //!< Dims of grabbed frames
  rutz::shared_ptr<FrameListener> itsListener;

  rutz::shared_ptr<Freenect::Freenect<FreenectDev> > itsDev;
  FreenectDev *itsFdev;
};

#endif

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
