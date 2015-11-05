/*!@file Simulation/SimEventBuffer.H */

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
// $HeadURL$ svn://isvn.usc.edu/software/invt/trunk/saliency/src/Simulation/SimEventBuffer.H
//

#ifndef SIMULATION_SIMEVENTBUFFER_H_DEFINED
#define SIMULATION_SIMEVENTBUFFER_H_DEFINED

#include <queue>

#include "rutz/mutex.h"
#include "Simulation/SimEventQueue.H"
#include "Simulation/SimEvents.H"

class SimEventBuffer : public SimModule
{
public:
  //! Constructor
  SimEventBuffer(OptionManager& mgr,
    const std::string& descrName = "SimEventBuffer",
    const std::string& tagName = "SimEventBuffer");

  //! Destructor
  virtual ~SimEventBuffer();

  //! Post an event asynchronously
  /*! The event is queued in a buffer and asyncPost returns immediately.
      The buffered event is posted to SimEventQueue asynchronously on
      next clock tick.
  */
  template <class E> inline void asyncPost(rutz::shared_ptr<E> e);

  //! Retrieve the next buffered event.
  /*! Normally, this is called only when receving
      SimEventClockTick event. It returns invalid
      rutz::shared_ptr<SimEvent> when buffer is
      empty.
  */
  rutz::shared_ptr<SimEvent> asyncRetrieve();

  //! Return SimEventBuffer instance
  static SimEventBuffer& Instance()
  {
    if (pThis == NULL)
    {
      LFATAL("SimEventBuffer instance is not created yet or has been destroyed already");
    }
    return *pThis;
  }

protected:
  //! Callback for when a new clock tick is available
  SIMCALLBACK_DECLARE(SimEventBuffer, SimEventClockTick);

private:
  std::queue< rutz::shared_ptr<SimEvent> > buffer;
  pthread_mutex_t buffer_access_mutex;
  static SimEventBuffer* pThis;
};

template <class E> inline void SimEventBuffer::asyncPost(rutz::shared_ptr<E> e)
{
  // reject uninitialized shared_ptr's:
  if (e.is_valid() == false) LFATAL("You cannot post invalid events.");

  // enforce that E should derive from SimEvent, i.e., a pointer to an
  // object of type E should be dyn_cast'able to a pointer to type
  // SimEvent:
  rutz::shared_ptr<SimEvent> ebase; dyn_cast_to_from(ebase, e);
  if (ebase.is_valid() == false)
    LFATAL("Passed event of type '%s' does not derive from SimEvent", rutz::demangled_name(typeid(*e)));

  // plunge into the appropriate realm:
  // if (e->itsSource == 0) LFATAL("Received event [%s] with invalid source", e->toString().c_str());

  // lock the buffer and queue the event
  {
    GVX_MUTEX_LOCK(&buffer_access_mutex);
    buffer.push(ebase);
  }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // SIMULATION_SIMEVENTBUFFER_H_DEFINED
