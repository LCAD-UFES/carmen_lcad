/*!@file Psycho/Staircase.H A staircase procedure for psychophysical thresholds */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/Staircase.H $
// $Id: Staircase.H 7272 2006-10-18 19:24:01Z rjpeters $
//

#ifndef STAIRCASE_H_DEFINED
#define STAIRCASE_H_DEFINED

#include "Component/ModelComponent.H"
#include "Component/ModelParam.H"
#include "Util/Timer.H"
#include "Util/Types.H"

#include <list>

class ModelManager;

//! A staircase procedure to measure psychophysical thresholds
/*! This class facilitates the evaluation of a threshold in a
  psychophysical task. */

class Staircase : public ModelComponent
{
public:
  //! Constructor
  Staircase(OptionManager& mgr,
            const std::string& descrName = "Staircase",
            const std::string& tagName = "Staircase");

  //! Destructor
  ~Staircase();

  //! Reset to initial conditions
  virtual void reset1();

  //! Get parameter values to use for next trial
  /*! This will give you two values, one to use for the first period
    of the following trial, and one for the second period. The
    Staircase class will keep in memory what the expected answer
    should be, which you will give using setResponse(). */
  void getValues(double& value1, double& value2);

  //! Set the response of the current trial
  /*! The response should be true is the user said
    nothing-then-something, false otherwise. */
  void setResponse(const bool response);

  //! Log an event
  /*! This is in the public section just in case someone wants to push
    external events, but should not be called by the user in normal
    situations, as it is called internally when getNextValue() is
    called. */
void pushEvent(const std::string& msg, const int trial = -1,
               const double val = 0.0, const bool response = false);

protected:
  NModelParam<double> itsInitial; //!< initial value
  NModelParam<double> itsDelta;   //!< delta in values
  NModelParam<double> itsMini;    //!< minimum possible value
  NModelParam<double> itsMaxi;    //!< maximum possible value
  NModelParam<int> itsNright;     //!< nb of right responses to subtract a delta
  NModelParam<int> itsNwrong;     //!< nb of wrong responses to add a delta
  NModelParam<std::string> itsFileName; //!< name of file to save the logs

  void start2(); //!< get started
  void stop1();  //!< get stopped

private:
  int itsTrial;
  double itsValue;
  Timer itsTimer;

  struct StaircaseEvent {
    uint64 tim;           // the time in microseconds since start()
    std::string descrip;  // description of the event
    int trial;            // trial number, or <0 to not display trial info
    double val;           // current value
    bool response;        // last response
  };

  std::list<StaircaseEvent> itsEvents;  // record a bunch of events
  int itsNr, itsNw;
  bool itsExpectedResponse;
};

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
