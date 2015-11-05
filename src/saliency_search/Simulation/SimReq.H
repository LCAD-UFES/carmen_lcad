/*!@file Simulation/SimReq.H A Request to be handled during simulation */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimReq.H $
// $Id: SimReq.H 13103 2010-03-31 02:24:47Z itti $
//

#ifndef SIMULATION_SIMREQ_H_DEFINED
#define SIMULATION_SIMREQ_H_DEFINED

#include "Simulation/SimModule.H"
#include <string>

//! Base class for requests that arise in simulation
/*! SimReq is similar to SimEvent but differs in two critical manners:
    (1) SimReq objects never get posted onto theSimEventQueue
    (blackboard). So they are private data exchanged between
    SimModules without becoming publicly available on the blackboard
    like SimEvent objects. (2) SimReq objects are typically used
    read/write as opposed to being read-only like SimEvents. */
class SimReq {
public:
  //! Constuctor
  /*! Normally a SimModule would pass "this" as src. */
  SimReq(SimModule* src);

  //! Destructor
  virtual ~SimReq();

  //! Get a description for printing out
  /*! The base class implementation only returns the demangled name of
    the class. Overload as necessary in derived classes if you want to
    also return additional information. */
  virtual std::string toString() const;

private:
  friend class SimEventQueue; // allow SimEventQueue to access our private data

  SimModule* const itsSource; // source of the event (a brain module)
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMREQ_H_DEFINED
