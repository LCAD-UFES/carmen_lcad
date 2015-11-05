/*!@file Psycho/StimController.H an abstract class to support control
   of our psycho display system from an external device */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: David Berg <dberg@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/StimController.H $

#ifndef PSYCHO_STIMCONTROLLER_H_FEFINED
#define PSYCHO_STIMCONTROLLER_H_FEFINED

#include "Component/ModelComponent.H"
#include "Util/WorkThreadServer.H"
#include "Psycho/DisplayController.H"
#include "Psycho/StimListener.H"

// ######################################################################
//!a class to control psychodisplay from a host sending commands
// ######################################################################
/*! A ModelComponent that coordinates communication between the SDL
    display code and an external system. An object of this type
    initiates two threads, one to listen for commands from a host, and
    one to render them to the screen using SDL. The calling program can also
    pass in data which can be incorporated into the next display.*/
class StimController : public ModelComponent
{
public:
  //!constructor
  StimController(OptionManager& mgr,
                 const std::string& descrName = " StimController",
                 const std::string& tagName = "StimController");

  //!destructor
  virtual ~StimController();

  //!initialize our object with a stimulus controller and listener
  void initialize(const StimListener* const sl);

  //!set the event log for our components
  void setEventLog(const nub::soft_ref<EventLog>& elog);

  //!set the handle to a psycho display
  void setDisplay(const nub::soft_ref<PsychoDisplay>& display);

  //! return a reference to the current listener
  rutz::shared_ptr<StimListener> getStimListener();

  //! return a reference to the current Controller
  rutz::shared_ptr<DisplayController> getDispController();

  //! pause the display, we will still recieve commands, but stop
  //! processing them until unpaused.
  void pauseDisplay(const bool pausestate);

protected:
  //!start our module
  void start2();

  //stop and clean up
  void stop1();

private:
  //!prohibit assignment
  StimController& operator=(const StimController& d);

  //!prohibit copy constuctor
  StimController(const StimController& d);

  //!thread to listen for commands
  rutz::shared_ptr<StimListener> itsListener;

  //!thread to display commands
  rutz::shared_ptr<DisplayController> itsController;

  //!thread-safe shared buffer
  rutz::shared_ptr<SharedBuffer<StimData> > itsBuffer;

  //!A WorkThreadServer for our tasks.
  WorkThreadServer itsThreadServer;
};

// ######################################################################
//inline methods for StimController
// ######################################################################
StimController::StimController(OptionManager& mgr,
                               const std::string& descrName,
                               const std::string& tagName) :
  ModelComponent(mgr,descrName,tagName),
  itsBuffer(new SharedBuffer<StimData>()),
  itsListener(),
  itsController(new DisplayController()),
  itsThreadServer("Stimulus Controller",2)
{
}

// ######################################################################
StimController::~StimController()
{

}

// ######################################################################
void initialize(const StimListener* const sl)
{
  itsController = rutz::make_shared(sl);
}

// ######################################################################
void StimController::setEventLog(const nub::soft_ref<EventLog>& log)
{
  if (itsListener.is_valid() && itsController.is_valid())
    {
      itsListener->setEventLog(log);
      itsController->setEventLog(log);
    }
  else
    LFATAL("Must initialize with a DisplayController and StimListener");
}

// ######################################################################
void StimController::setDisplay(const nub::soft_ref<PsychoDisplay>& disp)
{
  if (itsListener.is_valid() && itsController.is_valid())
    itsController->setDisplay(disp);
  else
    LFATAL("Must initialize with a DisplayController and StimListener");

}

// ######################################################################
void StimController::start2()
{
  itsListener->setBuffer(itsBuffer);
  itsController->setBuffer(itsBuffer);
  itsListener->start();
  itsController->start();
  itsThreadServer.enqueueJob(itsListener);
  itsThreadServer.enqueueJob(itsController);
}

// ######################################################################
void StimController::stop1()
{
  itsListener->stop();
  itsBuffer->stopWaiting();
  itsController->stop();
}

// ######################################################################
rutz::shared_ptr<StimListener> StimController::getStimListener()
{
  return itsListener;
}

// ######################################################################
rutz::shared_ptr<DisplayController> StimController::getDispController()
{
  return itsController;
}

// ######################################################################
void StimController::pauseDisplay(const bool pausestate)
{
  itsController->pause(pausestate);
}


#endif
// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
