/*!@file Neuro/SimulationViewerEyeSim.C simulate an eye-tracker recording */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Neuro/SimulationViewerEyeSim.C $
// $Id: SimulationViewerEyeSim.C 13343 2010-04-30 22:37:42Z lior $
//

#include "Neuro/SimulationViewerEyeSim.H"
#include "Component/OptionManager.H"
#include "Neuro/Brain.H"
#include "Neuro/NeuroOpts.H"
#include "Neuro/NeuroSimEvents.H"
#include "Neuro/EyeHeadController.H"
#include "Neuro/SaccadeController.H"
#include "Simulation/SimEventQueue.H"
#include <stdio.h>

// ######################################################################
SimulationViewerEyeSim::
SimulationViewerEyeSim(OptionManager& mgr,
                       const std::string& descrName,
                       const std::string& tagName) :
  SimulationViewerStd(mgr, descrName, tagName),
  itsEyeFname(&OPT_SVeyeSimFname, this),
  itsEyePeriod(&OPT_SVeyeSimPeriod, this), // 240Hz
  itsEyeTrash(&OPT_SVeyeSimTrash, this),
  itsOutFile(NULL), itsLastEyePos(-1, -1)
{  }

// ######################################################################
SimulationViewerEyeSim::~SimulationViewerEyeSim()
{ }

// ######################################################################
void SimulationViewerEyeSim::start1()
{
  // get our base class started:
  SimulationViewerStd::start1();

  // abort if no output file:
  if (itsEyeFname.getVal().empty()) LFATAL("I need an output file!");

  // open output file:
  itsOutFile = fopen(itsEyeFname.getVal().c_str(), "w");
  if (itsOutFile == NULL)
    PLFATAL("Cannot write '%s'", itsEyeFname.getVal().c_str());

  // write our initial trash, here we just assume it's a long blink:
  for (int i = 0; i < itsEyeTrash.getVal(); i ++) fprintf(itsOutFile, "NaN NaN\n");
}

// ######################################################################
void SimulationViewerEyeSim::stop1()
{
  if (itsOutFile)
    {
      // write a few extra samples at end of simulation, duplicating whatever our latest eye position is:
      if (itsLastEyePos.isValid())
        for (int i = 0; i < 50; i ++) fprintf(itsOutFile, "%d.0 %d.0\n", itsLastEyePos.i, itsLastEyePos.j);
      else
        for (int i = 0; i < 50; i ++) fprintf(itsOutFile, "NaN NaN\n");

      // close down:
      fclose(itsOutFile);
      itsOutFile = NULL;
    }
  SimulationViewerStd::stop1();
}

// ######################################################################
void SimulationViewerEyeSim::
onSimEventWTAwinner(SimEventQueue& q, rutz::shared_ptr<SimEventWTAwinner>& e)
{
  Point2D<int> eye = e->winner().p;
  LINFO("Recording eyepos %d,%d", eye.i, eye.j);
  fprintf(itsOutFile, "%d.0 %d.0\n", eye.i, eye.j);
  itsLastEyePos = eye;

  SimulationViewerStd::onSimEventWTAwinner(q, e);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
