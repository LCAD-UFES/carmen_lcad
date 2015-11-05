/*!@file Simulation/SimEventBuffer.C */

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
// Primary maintainer for this file: Pezhman Firoozfam (pezhman.firoozfam@usc.edu)
// $HeadURL$ svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventBuffer.C
//

#include "Simulation/SimEventBuffer.H"

//Define the inst function name
SIMMODULEINSTFUNC(SimEventBuffer);

SimEventBuffer* SimEventBuffer::pThis = NULL;

SimEventBuffer::SimEventBuffer(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  SimModule(mgr, descrName, tagName),
  SIMCALLBACK_INIT(SimEventClockTick)
{
  if (pThis != NULL) LFATAL("Only one instance of SimEventBuffer can be created");

  pThis = this;

  pthread_mutex_t init = PTHREAD_MUTEX_INITIALIZER;
  buffer_access_mutex = init;
}

SimEventBuffer::~SimEventBuffer()
{
  pThis = NULL;
}

rutz::shared_ptr<SimEvent> SimEventBuffer::asyncRetrieve()
{
  GVX_MUTEX_LOCK(&buffer_access_mutex);
  if (buffer.empty()) return rutz::shared_ptr<SimEvent>();

  rutz::shared_ptr<SimEvent> event = buffer.front();
  buffer.pop();
  return event;
}

void SimEventBuffer::onSimEventClockTick(SimEventQueue& q,
                                         rutz::shared_ptr<SimEventClockTick>& e)
{
  for (rutz::shared_ptr<SimEvent> event = SimEventBuffer::asyncRetrieve();
       event.is_valid();
       event = SimEventBuffer::asyncRetrieve())
  {
    q.post(event);
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

