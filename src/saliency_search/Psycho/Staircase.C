/*!@file Psycho/Staircase.C A staircase procedure for psychophysical thresholds */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Psycho/Staircase.C $
// $Id: Staircase.C 8264 2007-04-17 21:43:10Z rjpeters $
//

#include "Psycho/Staircase.H"

#include "Component/OptionManager.H"
#include "Util/MathFunctions.H"
#include "Util/sformat.H"

#include <fstream>

// ######################################################################
Staircase::Staircase(OptionManager& mgr, const std::string& descrName,
                     const std::string& tagName) :
  ModelComponent(mgr, descrName, tagName),
  itsInitial("InitialValue", this, 1.0),
  itsDelta("DeltaValue", this, 0.1),
  itsMini("MinValue", this, 0.0),
  itsMaxi("MaxValue", this, 1.0),
  itsNright("Nright", this, 4),
  itsNwrong("Nwrong", this, 2),
  itsFileName("FileName", this, std::string("staircase-")+tagName+".txt"),
  itsTrial(0), itsValue(0.0),
  itsTimer(1000000),  // use microsecond resolution
  itsEvents(), itsNr(0), itsNw(0), itsExpectedResponse(false)
{ }

// ######################################################################
Staircase::~Staircase()
{ }

// ######################################################################
void Staircase::start2()
{
  initRandomNumbers();

  itsValue = itsInitial.getVal(); itsTimer.reset();
  time_t ti = time(NULL);
  pushEvent(sformat("##### START %s #####", ctime(&ti)));
}

// ######################################################################
void Staircase::reset1()
{
  itsTrial = 0; itsValue = itsInitial.getVal();
  itsTimer.reset(); itsNr = 0; itsNw = 0;
  time_t ti = time(NULL);
  pushEvent(sformat("##### RESET %s #####", ctime(&ti)));

  ModelComponent::reset1();
}

// ######################################################################
void Staircase::stop1()
{
  // let's save the events:
  if (itsEvents.empty() == false && itsFileName.getVal().length() > 1)
    {
      std::ofstream ofs(itsFileName.getVal().c_str());
      if (!ofs.is_open())
        {
          LERROR("Couldn't open file '%s' for writing.",
                 itsFileName.getVal().c_str());
          return;
        }

      std::list<StaircaseEvent>::const_iterator itr = itsEvents.begin();
      while (itr != itsEvents.end()) {
        int usec = int(itr->tim % 1000ULL);
        int msec = int((itr->tim / 1000ULL) % 1000ULL);
        int sec = int((itr->tim / 1000000ULL) % 60ULL);
        int minu = int((itr->tim / 60000000ULL) % 60ULL);
        int hour = int(itr->tim / 3600000000ULL);
        char tim[256]; sprintf(tim, "%03d:%02d:%02d.%03d.%03d",
                               hour, minu, sec, msec, usec);
        ofs << tim << " " << itr->descrip;
        if (itr->trial >= 0) {
          ofs << " Trial = " << itr->trial << " Value = " << itr->val;
          if (itr->descrip.compare("end  ") == 0)
            ofs << " Response = " << (itr->response ? "Correct" : "Incorrect");
        }
        ofs << std::endl;
        itr ++;
      }
      ofs.close();
      LINFO("Saved data to '%s'", itsFileName.getVal().c_str());
    }
}

// ######################################################################
void Staircase::getValues(double& value1, double& value2)
{
  // randomly pick a trial type
  if (randomDouble() < 0.5)
    {
      // nothing-then-something:
      value1 = 0.0; value2 = itsValue; itsExpectedResponse = true;
    }
  else
    {
      // something-then-nothing:
      value1 = itsValue; value2 = 0.0; itsExpectedResponse = false;
    }

  // log the setup:
  pushEvent("begin", itsTrial, itsValue, true);
}

// ######################################################################
void Staircase::setResponse(const bool response)
{
  bool correct = (response == itsExpectedResponse);

  // log the current value of the parameter and the corresponding
  // response obtained for that value:
  pushEvent("end  ", itsTrial, itsValue, correct);

  // compute the next value to use:
  if (correct) { itsNr ++; itsNw = 0; }  // correct answer
  else { itsNw ++; itsNr = 0; }  // incorrect answer

  if (itsNr >= itsNright.getVal())
    { itsValue -= itsDelta.getVal(); itsNr = 0; }
  if (itsNw >= itsNwrong.getVal())
    { itsValue += itsDelta.getVal(); itsNw = 0; }

  if (itsValue < itsMini.getVal()) itsValue = itsMini.getVal();
  if (itsValue > itsMaxi.getVal()) itsValue = itsMaxi.getVal();

  itsTrial ++;
}

// ######################################################################
void Staircase::pushEvent(const std::string& msg, const int trial,
                          const double val, const bool response)
{
  StaircaseEvent evt;
  evt.descrip = msg; evt.tim = itsTimer.get();
  evt.trial = trial; evt.val = val; evt.response = response;
  itsEvents.push_back(evt);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
