/*!@file Util/FpsTimer.H Utility class for monitoring framerates and cpu usage ratios */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/FpsTimer.H $
// $Id: FpsTimer.H 8668 2007-08-07 18:58:15Z rjpeters $
//

#ifndef UTIL_FPSTIMER_H_DEFINED
#define UTIL_FPSTIMER_H_DEFINED

#include "rutz/time.h"

#include <deque>

#include <sys/resource.h>
#include <sys/time.h>

/// Utility class for monitoring framerates and cpu usage ratios
class FpsTimer
{
public:
  FpsTimer();

  /// update frame count and times
  void nextFrame();

  rutz::time getElapsedTime() const;

  int frameNumber() const;
  double getRecentFps() const;
  double getRecentCpuUsage() const;
  double getTotalFps() const;
  double getTotalCpuUsage() const;

  struct State
  {
    rutz::time elapsed_time;
    unsigned int frame_number;
    double recent_fps;
    double recent_cpu_usage;
    double total_fps;
    double total_cpu_usage;
  };

  State getState() const;

private:
  unsigned int itsFrameNumber;

  rutz::time itsWallStart;
  rutz::time itsUserStart;
  rutz::time itsSysStart;

  std::deque<rutz::time> itsWallTimes;
  std::deque<rutz::time> itsUserTimes;
  std::deque<rutz::time> itsSysTimes;
};

// ######################################################################
inline FpsTimer::FpsTimer()
  :
  itsFrameNumber(0)
{
  itsWallTimes.push_back(rutz::time::wall_clock_now());

  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  itsUserTimes.push_back(rutz::time(ru.ru_utime));
  itsSysTimes.push_back(rutz::time(ru.ru_stime));

  itsWallStart = itsWallTimes.back();
  itsUserStart = itsUserTimes.back();
  itsSysStart = itsSysTimes.back();
}

// ######################################################################
void FpsTimer::nextFrame()
{
  ++itsFrameNumber;

  itsWallTimes.push_back(rutz::time::wall_clock_now());

  struct rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  itsUserTimes.push_back(rutz::time(ru.ru_utime));
  itsSysTimes.push_back(rutz::time(ru.ru_stime));

  if (itsWallTimes.size() > 31)
    {
      itsWallTimes.pop_front();
      itsUserTimes.pop_front();
      itsSysTimes.pop_front();
    }
}

// ######################################################################
rutz::time FpsTimer::getElapsedTime() const
{
  return itsWallTimes.back() - itsWallStart;
}

// ######################################################################
int FpsTimer::frameNumber() const
{ return itsFrameNumber; }

// ######################################################################
double FpsTimer::getRecentFps() const
{
  return (itsWallTimes.size() - 1)
    / (itsWallTimes.back() - itsWallTimes.front()).sec();
}

// ######################################################################
double FpsTimer::getRecentCpuUsage() const
{
  if (itsWallTimes.back() == itsWallTimes.front())
    return 0.0;

  return
    (itsUserTimes.back() + itsSysTimes.back()
     - itsUserTimes.front() - itsSysTimes.front()).sec()
    /
    (itsWallTimes.back() - itsWallTimes.front()).sec();
}

// ######################################################################
double FpsTimer::getTotalFps() const
{
  if (itsFrameNumber < 1)
    return 0.0;

  return itsFrameNumber
    / (itsWallTimes.back() - itsWallStart).sec();
}

// ######################################################################
double FpsTimer::getTotalCpuUsage() const
{
  if (itsWallTimes.back() == itsWallStart)
    return 0.0;

  return
    (itsUserTimes.back() + itsSysTimes.back()
     - itsUserStart - itsSysStart).sec()
    /
    (itsWallTimes.back() - itsWallStart).sec();
}

// ######################################################################
FpsTimer::State FpsTimer::getState() const
{
  State result;
  result.elapsed_time = this->getElapsedTime();
  result.frame_number = this->frameNumber();
  result.recent_fps = this->getRecentFps();
  result.recent_cpu_usage = this->getRecentCpuUsage();
  result.total_fps = this->getTotalFps();
  result.total_cpu_usage = this->getTotalCpuUsage();
  return result;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_FPSTIMER_H_DEFINED
