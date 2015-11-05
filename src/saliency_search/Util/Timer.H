/*!@file Util/Timer.H A simple class for precise time measurements */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/Timer.H $
// $Id: Timer.H 14168 2010-10-27 00:44:20Z farhan $
//

#ifndef TIMER_H_DEFINED
#define TIMER_H_DEFINED

#include "Util/SimTime.H"
#include "Util/Types.H"  // for uint64
#include "Util/log.H"
#include <sys/time.h>
#include <unistd.h>

//! This class implements a real-time timer
/*! Timing starts at construction or reset().

At construction, you can decide on a number of timer ticks per second;
use 1000 to count in milliseconds, 1000000 to count in microseconds,
etc.  The best resolution achievable is 1us, as this timer relies on
the gettimeofday() system call, which has microsecond resolution. Use
get() to measure the number of elapsed ticks since last reset(), or
getSecs() to get the fractional number of seconds.  Use ticksPerSec()
to retrieve this value, for example to convert yourself your measured
times back to seconds instead of using getSecs(). */
class Timer {
public:
  //! Constructor; immediately start timing
  /*! @param tickspersec number of timer ticks per second; use 1000 to
    count in milliseconds, 1000000 to count in microseconds,
    etc. Values larger than 1000000 will throw a fatal error. If
    1000000/tickspersec is not integer, a fatal error will also
    occur. */
  inline Timer(const int tickspersec = 1000);

  //! Reset the timer and immediately start timing again
  inline void reset();

  //! Return elapsed time
  /*! Return value is the number of ticks since last reset or since
    construction. */
  inline uint64 get() const;

  //! Return elapsed time since last reset, and reset
  inline uint64 getReset();

  //! Returns number of ticks per second, as specified at construction
  inline int ticksPerSec() const;

  //! Returns number of seconds since last reset
  inline double getSecs() const;

 //! Returns number of milliseconds since last reset
  inline double getMilliSecs() const;

  //! Returns the elapsed SimTime since last reset
  inline SimTime getSimTime() const;

  //! Pause the timer
  inline void pause();

  //! Resume the timer
  inline void resume();

protected:
  //! time unit multipliers
  int64 div, mul;
  //! timestamp of last reset()
  struct timeval tv;
        //! timer offset of pause()/resume()
        int64 offset;
        bool pauseFlag;
};

// ######################################################################
// ##### INLINE FUNCTIONS:
// ######################################################################

inline Timer::Timer(const int tickspersec) :
  div(1000000LL / int64(tickspersec)), mul(int64(tickspersec)),offset(0L),pauseFlag(false)
{
  if (tickspersec > 1000000LL) LFATAL("tickspersec too large");
  if (1000000LL % tickspersec) LFATAL("1000000/tickspersec not integer");
  reset();
}

// ######################################################################
inline void Timer::reset()
{ gettimeofday(&tv, NULL);
        pauseFlag = false;
        offset = 0L;
}

// ######################################################################
inline uint64 Timer::get() const
{
        if(pauseFlag)
                return offset;

  struct timeval tv2; gettimeofday(&tv2, NULL);
  return uint64(int64(tv2.tv_usec - tv.tv_usec) / div +
                int64(tv2.tv_sec  - tv.tv_sec ) * mul) + offset;
}

// ######################################################################
inline uint64 Timer::getReset()
{
  struct timeval tv2; gettimeofday(&tv2, NULL);
  uint64 ret = uint64(int64(tv2.tv_usec - tv.tv_usec) / div +
                      int64(tv2.tv_sec  - tv.tv_sec ) * mul);
  tv.tv_sec = tv2.tv_sec; tv.tv_usec = tv2.tv_usec;
  return ret;
}

// ######################################################################
inline int Timer::ticksPerSec() const
{ return int(mul); }

// ######################################################################
inline double Timer::getSecs() const
{
  struct timeval tv2; gettimeofday(&tv2, NULL);
  return double(tv2.tv_usec - tv.tv_usec) / 1000000.0 +
    double(tv2.tv_sec  - tv.tv_sec);
}

// ######################################################################
inline double Timer::getMilliSecs() const
{
  struct timeval tv2; gettimeofday(&tv2, NULL);
  return (double(tv2.tv_usec - tv.tv_usec) / 1000000.0 +
          double(tv2.tv_sec  - tv.tv_sec))*1000;
}

// ######################################################################
inline SimTime Timer::getSimTime() const
{
  return SimTime::SECS(this->getSecs());
}

// ######################################################################
inline void Timer::pause()
{
        if(!pauseFlag){
                offset = get();
                pauseFlag = true;
        }
}
// ######################################################################
inline void Timer::resume()
{
        if(pauseFlag){
                pauseFlag =  false;
                gettimeofday(&tv, NULL);
        }
}
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
