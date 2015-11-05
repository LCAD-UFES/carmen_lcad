/*!@file Neuro/EyeHeadController.H Base class for eye/head controllers */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/EyeHeadController.H $
// $Id: EyeHeadController.H 10725 2009-02-02 01:12:31Z itti $
//

#ifndef NEURO_EYEHEADCONTROLLER_H_DEFINED
#define NEURO_EYEHEADCONTROLLER_H_DEFINED

#include "Simulation/SimModule.H"

//! This is the base class for an eye and head controller
/*! An EyeHeadController interacts with the brain. It is a holder and
  router class for SaccadeController objects for the eye and possibly
  the head. The EyeHeadController orchestrates the interactions
  between its components. The base class contains a number of default
  implementations that will need to be overloaded in derived classes
  to define this orchestration. See Neuro/EyeHeadControllers.H for
  derived classes which implement different control styles. */

class EyeHeadController : public SimModule {
public:
  //! Constructor
  EyeHeadController(OptionManager& mgr,
                    const std::string& descrName = "Eye/Head Controller",
                    const std::string& tagName = "EyeHeadController");

  //! Destructor
  virtual ~EyeHeadController();
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // NEURO_EYEHEADCONTROLLER_H_DEFINED
