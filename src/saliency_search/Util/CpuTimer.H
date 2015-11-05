/*! @file Util/CpuTimer.H Track user/system CPU usage and wall-clock time */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; filed July 23, 2001, following provisional applications     //
// No. 60/274,674 filed March 8, 2001 and 60/288,724 filed May 4, 2001).//
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/CpuTimer.H $
// $Id: CpuTimer.H 5375 2005-08-23 15:19:13Z rjpeters $
//

#ifndef CPUTIMER_H_DEFINED
#define CPUTIMER_H_DEFINED

#include "Util/log.H"

#include <sys/resource.h>
#include <sys/time.h>

//! Utility class for monitoring user/system cpu usage plus wall-clock time.
class CpuTimer
{
public:
  //! Constructor; immediately start timing
  inline CpuTimer();

  //! Reset the timer and immediately start timing again
  inline void reset();

  //! Mark the current time for access via user_secs()/sys_secs()/real_secs()
  inline void mark();

  //! Get the user CPU seconds between reset() and mark()
  inline double user_secs() const;

  //! Get the system CPU seconds between reset() and mark()
  inline double sys_secs() const;

  //! Get the real (wall-clock) CPU seconds between reset() and mark()
  inline double real_secs() const;

  //! Print a report based on the most recent mark()
  inline void report(const char* where = "") const;

private:
  timeval t1, t2;
  rusage r1, r2;
};

// ######################################################################
// ##### INLINE FUNCTIONS:
// ######################################################################

inline CpuTimer::CpuTimer()
{ reset(); }

inline void CpuTimer::reset()
{
  gettimeofday(&t1, (struct timezone*)0);
  getrusage(RUSAGE_SELF, &r1);

  r2 = r1;
  t2 = t1;
}

inline void CpuTimer::mark()
{
  gettimeofday(&t2, (struct timezone*)0);
  getrusage(RUSAGE_SELF, &r2);
}

inline double CpuTimer::user_secs() const
{
  return
    double(r2.ru_utime.tv_sec - r1.ru_utime.tv_sec) +
    double(r2.ru_utime.tv_usec - r1.ru_utime.tv_usec) / 1000000.0;
}

inline double CpuTimer::sys_secs() const
{
  return
    double(r2.ru_stime.tv_sec - r1.ru_stime.tv_sec) +
    double(r2.ru_stime.tv_usec - r1.ru_stime.tv_usec) / 1000000.0;
}

inline double CpuTimer::real_secs() const
{
  return
    double(t2.tv_sec - t1.tv_sec) +
    double(t2.tv_usec - t1.tv_usec) / 1000000.0;
}

inline void CpuTimer::report(const char* where) const
{
  const double u = user_secs();
  const double s = sys_secs();
  const double r = real_secs();
  if (where != 0 && where[0] != '\0')
    LINFO("%s: %.3fu, %.3fs, %.3fr, %.1f%%",
          where, u, s, r, 100.0 * (u+s) / r);
  else
    LINFO("%.3fu, %.3fs, %.3fr, %.1f%%",
          u, s, r, 100.0 * (u+s) / r);
}

#endif // !CPUTIMER_H_DEFINED
