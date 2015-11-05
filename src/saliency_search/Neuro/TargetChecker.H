/*!@file Neuro/TargetChecker.H check+count targets hit during a neural simulation */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/TargetChecker.H $
// $Id: TargetChecker.H 10744 2009-02-03 04:51:40Z itti $
//

#ifndef NEURO_TARGETCHECKER_H_DEFINED
#define NEURO_TARGETCHECKER_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Image/Image.H"
#include "Neuro/NeuroSimEvents.H"
#include "Simulation/SimModule.H"

class InputFrame;

//! Check and count targets hit during a neural simulation
/*! We look for SimEventRetinaImage as well as for SimEventWTAwinner,
  and post a SimEventBreak when all targets were attended to. */
class TargetChecker : public SimModule
{
public:
  //! Constructor
  TargetChecker(OptionManager& mgr,
                const std::string& descrName = "Target Checker",
                const std::string& tagName = "TargetChecker");

  //! Destructor
  virtual ~TargetChecker();

protected:
  //! Callback for when a new WTA winner is available
  SIMCALLBACK_DECLARE(TargetChecker, SimEventWTAwinner);

  //! Callback for when a new WTA winner is available
  SIMCALLBACK_DECLARE(TargetChecker, SimEventRetinaImage);

  //! Callback for when a new target mask should be used
  SIMCALLBACK_DECLARE(TargetChecker, SimEventTargetMask);

  //! Get started
  virtual void start1();

  //! Get reset
  virtual void reset1();

private:
  // FOA radius for our drawings
  OModelParam<int> itsFOAradius;

  // text log file name
  OModelParam<std::string> itsLogFile;

  // filename of a targetmask image to load; or empty string for no targetmask
  OModelParam<std::string> itsTargetMaskFname;

  // This is a visual memory of which locations in the visual field
  // have been covertly attended to; it starts as an empty image of
  // same dims as this->retinalImage, but FOAs get painted into it each time
  // a new shift of covert attention is made. NOTE: this concept works
  // well with a static input, but determining when the visual field
  // should be erased or shifted because the viewpoint changed should
  // come from the SaccadeController in a future implementation. The
  // same applies to SM and WTA, for that matter.
  Image<byte> itsVisualField;

  // stores the image that was passed in to setTargetMask()
  Image<byte> itsRawTargetMask;

  // binary mask indicating where targets are; if one was given to
  // Brain::input(), that's a copy of it with the targets found so far
  // painted in a different color, otherwise it's an initialized image
  Image<byte> itsTargetMask;

  // number of targets that still need to be found
  int itsNumTargetsRemaining;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_TARGETCHECKER_H_DEFINED
