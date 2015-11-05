/*!@file Neuro/SimulationViewerEyeSim.H simulate an eye-tracker recording */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeSim.H $
// $Id: SimulationViewerEyeSim.H 10821 2009-02-11 07:32:37Z itti $
//

#ifndef SIMULATIONVIEWEREYESIM_H_DEFINED
#define SIMULATIONVIEWEREYESIM_H_DEFINED

#include "Component/ModelParam.H"
#include "Neuro/SimulationViewerStd.H"

class EyeHeadController;

//! Simulate an eye-tracker recording
/*! This simulation viewer has for main purpose to output a text file
that is in the same format as what we would get from a human eye
tracker device (.eye file). Thus it allows us to generate and save eye
movement scanpaths that come from the model but are in the same format
as human recordings. In turn, these scanpaths can then be compared to
actual human scanpaths. SimulationViewerEyeSim derived from
SimulationViewerStd and hence inherits all its behavior, simply adding
provision for generating and saving the .eye scanpath files. */
class SimulationViewerEyeSim : public SimulationViewerStd {
public:
  // ######################################################################
  /*! @name Constructors and destructors */
  //@{

  //! Constructor. See ModelComponent.H.
  SimulationViewerEyeSim(OptionManager& mgr,
                         const std::string& descrName = "EyeSim Simulation Viewer",
                         const std::string& tagName = "SimulationViewerEyeSim");

  //! Destructor
  virtual ~SimulationViewerEyeSim();

  //@}

protected:
  //! Overload of callback defined in SimulationViewerStd
  virtual void
  onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e);

  //! Name of a calibrated eye movement file
  OModelParam<std::string> itsEyeFname;

  //! Period at which eye movement samples should be read in
  OModelParam<SimTime> itsEyePeriod;

  //! Number of initial eye movement samples before stimulus starts
  OModelParam<int> itsEyeTrash;

  //! Get started
  virtual void start1();

  //! get stopped
  virtual void stop1();

private:
  FILE *itsOutFile;
  Point2D<int> itsLastEyePos;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
