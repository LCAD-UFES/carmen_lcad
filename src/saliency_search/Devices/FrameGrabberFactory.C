/*!@file Devices/FrameGrabberFactory.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/FrameGrabberFactory.C $
// $Id: FrameGrabberFactory.C 15444 2012-12-01 04:06:55Z kai $
//

#ifndef DEVICES_FRAMEGRABBERFACTORY_C_DEFINED
#define DEVICES_FRAMEGRABBERFACTORY_C_DEFINED

#include "Devices/XCgrabber.H"
#include "Devices/XCgrabberFlex.H"
#include "Devices/FrameGrabberFactory.H"
#include "Devices/V4Lgrabber.H"
#include "Devices/V4L2grabber.H"
#include "Devices/IEEE1394grabber.H"
#include "Devices/KinectGrabber.H"
#include "Devices/OpenNIGrabber.H"
#include "Devices/Bumblebee2Grabber.H"
#include "Util/log.H"

// ######################################################################
nub::ref<FrameIstream>
makeV4Lgrabber(OptionManager& mgr,
               const std::string& descrName,
               const std::string& tagName,
               const ParamFlag flags)
{
#ifndef HAVE_LINUX_VIDEODEV_H
  LFATAL("<linux/videodev.h> must be installed to build a V4Lgrabber");
  return nub::ref<FrameIstream>((FrameIstream*)0); // can't happen, but placate compiler
#else
  return nub::ref<FrameIstream>
    (new V4Lgrabber(mgr, descrName, tagName, flags));
#endif
}

// ######################################################################
nub::ref<FrameIstream>
makeV4L2grabber(OptionManager& mgr,
                const std::string& descrName,
                const std::string& tagName,
                const ParamFlag flags)
{
#ifndef HAVE_LINUX_VIDEODEV2_H
  LFATAL("<linux/videodev2.h> must be installed to build a V4L2grabber");
  return nub::ref<FrameIstream>((FrameIstream*)0); // can't happen, but placate compiler
#else
  return nub::ref<FrameIstream>
    (new V4L2grabber(mgr, descrName, tagName, flags));
#endif
}

// ######################################################################
nub::ref<FrameIstream>
makeIEEE1394grabber(OptionManager& mgr,
                    const std::string& descrName,
                    const std::string& tagName,
                    const ParamFlag flags)
{
  return nub::ref<FrameIstream>
    (new IEEE1394grabber(mgr, descrName, tagName, flags));
}

// ######################################################################
nub::ref<FrameIstream>
makeXCgrabber(OptionManager& mgr,
              const std::string& descrName,
              const std::string& tagName,
              const ParamFlag flags)
{
  return nub::ref<FrameIstream>
    (new XCgrabber(mgr, descrName, tagName, flags));
}
// ######################################################################
nub::ref<FrameIstream>
makeXCgrabberFlex(OptionManager& mgr,
              const std::string& descrName,
              const std::string& tagName,
              const ParamFlag flags)
{
  return nub::ref<FrameIstream>
    (new XCgrabberFlex(mgr, descrName, tagName, flags));
}

// ######################################################################
nub::ref<FrameIstream>
makeKinectgrabber(OptionManager& mgr,
                  const std::string& descrName,
                  const std::string& tagName,
                  const ParamFlag flags)
{
#ifndef INVT_HAVE_LIBFREENECT
  LFATAL("<libfreenect/libfreenect.h> must be installed to build a KinectGrabber");
  return nub::ref<FrameIstream>((FrameIstream*)0); // can't happen, but placate compiler
#else
  return nub::ref<FrameIstream>(new KinectGrabber(mgr, descrName, tagName, flags));
#endif
}
// ######################################################################
nub::ref<FrameIstream>
makeBumblebee2grabber(OptionManager& mgr,
                  const std::string& descrName,
                  const std::string& tagName,
                  const ParamFlag flags)
{
#ifndef HAVE_DC1394V2
  LFATAL("you must have libdc1394 version 2.x in order to use Bumblebee2Grabber");
  return nub::ref<FrameIstream>((FrameIstream*)0); // can't happen, but placate compiler
#else
  return nub::ref<FrameIstream>(new Bumblebee2Grabber(mgr, descrName, tagName));
#endif
}
// ######################################################################
nub::ref<FrameIstream>
makeOpenNIgrabber(OptionManager& mgr,
                  const std::string& descrName,
                  const std::string& tagName,
                  const ParamFlag flags)
{
#ifndef INVT_HAVE_OPENNI
  LFATAL("OpenNI must be installed to build a OpenNIGrabber");
  return nub::ref<FrameIstream>((FrameIstream*)0); // can't happen, but placate compiler
#else
  
  return nub::ref<FrameIstream>(new OpenNIGrabber(mgr, descrName, tagName, flags));
#endif
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // DEVICES_FRAMEGRABBERFACTORY_C_DEFINED
