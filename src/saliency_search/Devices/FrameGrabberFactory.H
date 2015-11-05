/*!@file Devices/FrameGrabberFactory.H Factory functions to build various FrameIstream subclasses */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/FrameGrabberFactory.H $
// $Id: FrameGrabberFactory.H 15444 2012-12-01 04:06:55Z kai $
//

#ifndef DEVICES_FRAMEGRABBERFACTORY_H_DEFINED
#define DEVICES_FRAMEGRABBERFACTORY_H_DEFINED

#include "Transport/FrameIstream.H"

//! Factory function to build a V4Lgrabber
/** NOTE: The reason for this function is so that we have only a
    single location where we need to concern ourselves with whether
    Video4Linux is actually supported on the build system. So, rather
    than having to do #ifdef HAVE_LINUX_VIDEODEV_H all over the place
    in client code, instead we just have the clients call
    makeV4Lgrabber(), and then within that function we'll just do an
    LFATAL() if Video4Linux isn't supported. */
nub::ref<FrameIstream>
makeV4Lgrabber(OptionManager& mgr,
               const std::string& descrName = "V4L Frame Grabber Driver",
               const std::string& tagName = "V4LFrameGrabber",
               const ParamFlag flags = USE_MY_VAL);

//! Factory function to build a V4L2grabber
/** NOTE: The reason for this function is so that we have only a
    single location where we need to concern ourselves with whether
    Video4Linux2 is actually supported on the build system. So, rather
    than having to do #ifdef HAVE_LINUX_VIDEODEV2_H all over the place
    in client code, instead we just have the clients call
    makeV4L2grabber(), and then within that function we'll just do an
    LFATAL() if Video4Linux2 isn't supported. */
nub::ref<FrameIstream>
makeV4L2grabber(OptionManager& mgr,
                const std::string& descrName = "V4L2 Frame Grabber Driver",
                const std::string& tagName = "V4L2FrameGrabber",
                const ParamFlag flags = USE_MY_VAL);

//! Factory function to build an IEEE1394grabber
nub::ref<FrameIstream>
makeIEEE1394grabber(OptionManager& mgr,
                    const std::string& descrName="IEEE1394 Frame Grabber Driver",
                    const std::string& tagName = "IEEE1394FrameGrabber",
                    const ParamFlag flags = USE_MY_VAL);

//! Factory function to build an XCgrabber(CameraLink framegrabber library from Epix Inc)
nub::ref<FrameIstream>
makeXCgrabber(OptionManager& mgr,
              const std::string& descrName="XC Frame Grabber Driver",
              const std::string& tagName = "XCFrameGrabber",
              const ParamFlag flags = USE_MY_VAL);

//! Factory function to build an XCgrabberFlex(CameraLink framegrabber library from Epix Inc with more functionality to change dims of acquired image)
nub::ref<FrameIstream>
makeXCgrabberFlex(OptionManager& mgr,
              const std::string& descrName="XC Frame Grabber Driver",
              const std::string& tagName = "XCFrameGrabberFlex",
              const ParamFlag flags = USE_MY_VAL);

//! Factory function to build a KinectGrabber
nub::ref<FrameIstream>
makeKinectgrabber(OptionManager& mgr,
                  const std::string& descrName = "Kinect Frame Grabber Driver",
                  const std::string& tagName = "KinectFrameGrabber",
                  const ParamFlag flags = USE_MY_VAL);

//! Factory function to build a Bumblebee2Grabber
nub::ref<FrameIstream>
makeBumblebee2grabber(OptionManager& mgr,
                  const std::string& descrName = "Bumblebee2 Frame Grabber Driver",
                  const std::string& tagName = "Bumblebee2 FrameGrabber",
                  const ParamFlag flags = USE_MY_VAL);

//! Factory function to build a OpenNIGrabber
nub::ref<FrameIstream>
makeOpenNIgrabber(OptionManager& mgr,
                  const std::string& descrName = "OpenNI Frame Grabber Driver",
                  const std::string& tagName = "OpenNIFrameGrabber",
                  const ParamFlag flags = USE_MY_VAL);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_FRAMEGRABBERFACTORY_H_DEFINED
