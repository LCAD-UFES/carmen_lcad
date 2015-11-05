/*!@file Devices/KinectGrabber.C Interface with a Kinect frame grabber */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/KinectGrabber.C $
// $Id: KinectGrabber.C 14130 2010-10-13 04:59:07Z itti $
//

#ifdef INVT_HAVE_LIBFREENECT

#include "Devices/KinectGrabber.H"

#include "Component/OptionManager.H" // for option alias requests
#include "Component/ModelOptionDef.H"
#include "Devices/DeviceOpts.H"
#include "Image/ColorOps.H"
#include "Raster/GenericFrame.H"
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

// ######################################################################
KinectGrabber::KinectGrabber(OptionManager& mgr, const std::string& descrName,
                         const std::string& tagName, const ParamFlag flags) :
  FrameIstream(mgr, descrName, tagName),
  // NOTE that contrary to the common case, we may give (by default
  // value of 'flags') USE_MY_VAL here when we construct the
  // OModelParam objects; that means that we push our values into the
  // ModelManager as the new default values, rather than having our
  // param take its value from the ModelManager's default
  itsDims(&OPT_FrameGrabberDims, this, Dims(FREENECT_FRAME_W, FREENECT_FRAME_H), flags),
  itsListener(),
  itsDev()
{

}

// ######################################################################
void KinectGrabber::start1()
{
  itsDev.reset(new Freenect::Freenect<FreenectDev>());
  LINFO("Creating device 0...");
  itsFdev = &(itsDev->createDevice(0));
  LINFO("Starting RGB streaming...");
  itsFdev->startVideo();
  LINFO("Starting Depth streaming...");
  itsFdev->startDepth();
  LINFO("Ready.");
}

// ######################################################################
void KinectGrabber::stop2()
{
  if (itsFdev) { itsFdev->stopVideo(); itsFdev->stopDepth(); }
  itsFdev = NULL;
  itsDev.reset();
}

// ######################################################################
KinectGrabber::~KinectGrabber()
{ }

// ######################################################################
void KinectGrabber::setListener(rutz::shared_ptr<FrameListener> listener)
{ itsListener = listener; }

// ######################################################################
GenericFrameSpec KinectGrabber::peekFrameSpec()
{
  GenericFrameSpec result;

  result.nativeType = GenericFrame::RGBD;
  result.videoFormat = VIDFMT_RGB24;
  result.videoByteSwap = false;
  result.dims = itsDims.getVal();
  result.floatFlags = 0;

  return result;
}

// ######################################################################
GenericFrame KinectGrabber::readFrame()
{
  if (itsFdev == NULL) LFATAL("Kinect device not started!");

  const GenericFrame frame = itsFdev->grab();
  if (itsListener.get() != 0) itsListener->onRawFrame(frame);

  return frame;
}

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
