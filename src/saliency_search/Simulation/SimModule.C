/*!@file Simulation/SimModule.C A module in a simulation framework */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimModule.C $
// $Id: SimModule.C 10763 2009-02-04 23:40:19Z itti $
//

#include "Simulation/SimModule.H"
#include "Simulation/SimEventQueue.H"

// ######################################################################
SimModule::SimModule(OptionManager& mgr, const std::string& descrName,
                     const std::string& tagName)
  // ModelComponent::ModelComponent() auto called - virtual base
{
  ModelComponent::init(mgr, descrName, tagName);
}

// ######################################################################
SimModule::~SimModule()
{ }

// ######################################################################
void SimModule::start1()
{
  // any callbacks to register with a queue? (SimCallbackClient interface)
  if (numCallbacks() > 0)
    {
      // FIXME: how else could we find our queue?
      nub::ref<ModelComponent> c = getRootObject()->subComponent("SimEventQueue", MC_RECURSE);
      nub::ref<SimEventQueue> q = dyn_cast<SimEventQueue, ModelComponent>(c);
      q->registerSimCallbackClient(this);
    }

  // any request handlers to register with a queue? (SimReqHandlerClient interface)
  if (numReqHandlers() > 0)
    {
      // FIXME: how else could we find our queue?
      nub::ref<ModelComponent> c = getRootObject()->subComponent("SimEventQueue", MC_RECURSE);
      nub::ref<SimEventQueue> q = dyn_cast<SimEventQueue, ModelComponent>(c);
      q->registerSimReqHandlerClient(this);
    }

  ModelComponent::start1();
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */
