/*!@file Devices/FrameGrabberConfigurator.C Base class for a frame grabber device */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Devices/FrameGrabberConfigurator.C $
// $Id: FrameGrabberConfigurator.C 15444 2012-12-01 04:06:55Z kai $
//

#include "Devices/FrameGrabberConfigurator.H"

#include "Component/OptionManager.H"
#include "Devices/DeviceOpts.H"
#include "Devices/FrameGrabberFactory.H"
#include "Transport/BobDeinterlacer.H"

// ######################################################################
FrameGrabberConfigurator::
FrameGrabberConfigurator(OptionManager& mgr,
                         const std::string& descrName,
                         const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsGrabberType(&OPT_FrameGrabberType, this),
  itsDeinterlacerType(&OPT_DeinterlacerType, this),
  itsGrabber()
{  }

// ######################################################################
FrameGrabberConfigurator::~FrameGrabberConfigurator()
{  }

// ######################################################################
nub::soft_ref<FrameIstream> FrameGrabberConfigurator::getFrameGrabber() const
{
  if (itsDeinterlacer.isValid())
    {
      itsDeinterlacer->setDelegate(itsGrabber);
      return itsDeinterlacer;
    }

  // else... we have no deinterlacer, so just return the plain grabber
  return itsGrabber;
}

// ######################################################################
void FrameGrabberConfigurator::
paramChanged(ModelParamBase* const param,
             const bool valueChanged,
             ParamClient::ChangeStatus* status)
{
  ModelComponent::paramChanged(param, valueChanged, status);

  // was that a change of our baby's name?
  if (param == &itsGrabberType) {
    // if we had one, let's unregister it and destroy it; upon
    // destruction, it will unexport its command-line options:
    if (itsGrabber.isValid())
      { removeSubComponent(*itsGrabber); itsGrabber.reset(NULL); }

    // instantiate a grabber of the appropriate type:
    if (itsGrabberType.getVal().compare("None") == 0)
      itsGrabber.reset(NULL);
    else if (itsGrabberType.getVal().compare("V4L") == 0)
      itsGrabber = makeV4Lgrabber(getManager());
    else if (itsGrabberType.getVal().compare("V4L2") == 0)
      itsGrabber = makeV4L2grabber(getManager());
    else if (itsGrabberType.getVal().compare("1394") == 0)
      itsGrabber = makeIEEE1394grabber(getManager());
    else if (itsGrabberType.getVal().compare("XC") == 0)
      itsGrabber = makeXCgrabber(getManager());
    else if (itsGrabberType.getVal().compare("XCFLEX") == 0)
      itsGrabber = makeXCgrabberFlex(getManager());
    else if (itsGrabberType.getVal().compare("KINECT") == 0)
      itsGrabber = makeKinectgrabber(getManager());
    else if (itsGrabberType.getVal().compare("BB2") == 0)
      itsGrabber = makeBumblebee2grabber(getManager());
    else if (itsGrabberType.getVal().compare("OPENNI") == 0)
      itsGrabber = makeOpenNIgrabber(getManager());
    else
      LFATAL("Unknown FrameGrabber type %s", itsGrabberType.getVal().c_str());

    // add our baby as a subcomponent of us so that it will become
    // linked to the manager through us (hopefully we are registered
    // with the manager), which in turn will allow it to export its
    // command-line options and get configured:
    if (itsGrabber.isValid()) {
      addSubComponent(itsGrabber);

      // tell the controller to export its options:
      itsGrabber->exportOptions(MC_RECURSE);
    }

    // some info message:
    LINFO("Selected FrameGrabber of type %s", itsGrabberType.getVal().c_str());
  }

  else if (param == &itsDeinterlacerType) {
    if (itsDeinterlacer.isValid())
      {
        removeSubComponent(*itsDeinterlacer);
        itsDeinterlacer.reset(NULL);
      }

    if (itsDeinterlacerType.getVal().compare("None") == 0)
      itsDeinterlacer.reset(NULL);
    else if (itsDeinterlacerType.getVal().compare("Bob") == 0)
      itsDeinterlacer.reset(new BobDeinterlacer(getManager()));
    else
      LFATAL("Unknown Deinterlacer type %s",
             itsDeinterlacerType.getVal().c_str());

    if (itsDeinterlacer.isValid()) {
      addSubComponent(itsDeinterlacer);

      itsDeinterlacer->exportOptions(MC_RECURSE);
    }

    // some info message:
    LINFO("Selected Deinterlacer of type %s",
          itsDeinterlacerType.getVal().c_str());
  }
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
