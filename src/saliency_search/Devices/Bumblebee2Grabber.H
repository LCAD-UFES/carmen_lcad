/*!@file Devices/Bumblebee2Grabber.H FireWire grabber class based on libdc1394 version 2.x */

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
// Primary maintainer for this file: Chin-Kai Chang <chinkaic@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/Bumblebee2Grabber.H $
// $Id: Bumblebee2Grabber.H 9255 2008-02-11 20:59:00Z kai$
//

#ifndef DEVICES_BUMBLEBEE2GRABBER_H_DEFINED
#define DEVICES_BUMBLEBEE2GRABBER_H_DEFINED

#undef HAVE_PCL// disable PCL code

#include "Component/ModelParam.H"
#include "Image/Dims.H"
#include "Transport/FrameIstream.H"
#include "Util/Types.H"
#include "Video/VideoFormat.H"
#include "Video/VideoFrame.H"
#include "Image/Image.H"


#ifdef HAVE_DC1394V2
#include <dc1394/control.h>
#include <dc1394/conversions.h>
#include <dc1394/utils.h>
#endif

#ifdef HAVE_BUMBLEBEE2
#include <pgr_registers.h>
#include <pgr_stereocam.h>
#endif

#ifdef HAVE_PCL
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <boost/thread/thread.hpp>
#include <pcl/common/common_headers.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/features/normal_3d.h>
#include <pcl/console/parse.h>

#endif
class VideoFrame;
template <class T> class PixRGB;
template <class T> class Image;

//! Firewire bus speed to use
#define IEEE1394GRABSPEED SPEED_400

//! FireWire grabber class based on libdc1394 version 2.x
class Bumblebee2Grabber : public FrameIstream
{
public:
  //! Constructor
  Bumblebee2Grabber(OptionManager& mgr,
                 const std::string& descrName="DC1394 Frame Grabber Driver v2",
                 const std::string& tagName = "Bumblebee2Grabber");

  //! Destructor
  virtual ~Bumblebee2Grabber();

  void run();

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
  //! Grab raw data
  /*! Don't call this directly; use readFrame() instead. */
  VideoFrame grabRaw();

  //! get started
  virtual void start1();

  //! get stopped
  virtual void stop2();

private:
  void makeRect();
  void makeStereo();
  virtual void paramChanged(ModelParamBase* const param,
                            const bool valueChanged,
                            ParamClient::ChangeStatus* status);

  //! whether to print detailed info about our camera
  OModelParam<bool> itsShowInputDetails;

  //! device name of the /dev/ entry for the grabber device
  OModelParam<std::string> itsDevName;

  //! input channel to use
  OModelParam<int> itsChannel;

  //! input subchannel to use
  OModelParam<int> itsSubChan;

  //! width of grabbed frames
  OModelParam<Dims> itsDims;
  //Dims itsDims;

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

  VideoFrame itsCurrentFrame;

  int itsFrameCount;
  unsigned int   itsBufferSize; // size of capture buffer
  // check whether the configure script found libdc1394 version 2.x;
  // if not, then just stub out the core of the Bumblebee2Grabber class
  // and LFATAL() if somebody actually tries to use it
#ifdef HAVE_DC1394V2
  dc1394_t* itsDC1394; // our link to libdc1394
  dc1394camera_t* itsCam; // our camera
  void cleanup_and_exit( dc1394camera_t* camera );
#endif

#ifdef HAVE_BUMBLEBEE2
  PGRStereoCamera_t itsStereoCamera;
  TriclopsInput  itsTriclopsInput; 
  TriclopsContext itsTriclops;
#endif

  void toPointCloud(Image<uint16> disparity);
  pthread_t itsRunThread;
  pthread_mutex_t itsLock;
  bool itsThreadRunning;
  Image<uint16> itsCurrentDisparityMap;
  Image<byte> itsCurrentRectMap;
  Image<byte> itsLeftImage;
  Image<byte> itsRightImage;
#ifdef HAVE_PCL
  boost::shared_ptr<pcl::visualization::PCLVisualizer> itsViewer;
  pcl::PointCloud<pcl::PointXYZ>::Ptr itsCloud;
#endif
  bool itsImageReady;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_BUMBLEBEE2GRABBER_H_DEFINED
