/** @file rutz/time.h user-friendly wrapper around timeval */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Nov  7 16:43:03 2002
// commit: $Id: time.h 8667 2007-08-07 18:57:58Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/time.h $
//
// --------------------------------------------------------------------
//
// This file is part of GroovX.
//   [http://ilab.usc.edu/rjpeters/groovx/]
//
// GroovX is free software; you can redistribute it and/or modify it
// under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// GroovX is distributed in the hope that it will be useful, but
// WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
// General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GroovX; if not, write to the Free Software Foundation,
// Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
//
///////////////////////////////////////////////////////////////////////

#ifndef GROOVX_RUTZ_TIME_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_TIME_H_UTC20050626084020_DEFINED

#include <sys/time.h>
// struct timeval {
//   unsigned long  tv_sec;   /* seconds since Jan. 1, 1970 */
//   long           tv_usec;  /* and microseconds */
// };

namespace rutz
{
  class time;
}

/// A simple wrapper around timeval.
/** All operations provide the no-throw guarantee. */
class rutz::time
{
public:
  /// Construct with time=0.
  time() throw() : m_timeval()
  {
    reset();
  }

  /// Construct from a timeval.
  time(const timeval& t) throw() : m_timeval(t)
  {
    normalize(this->m_timeval);
  }

  /// Construct with given seconds+microseconds values.
  time(unsigned long s, long us) throw() : m_timeval()
  {
    reset(s, us);
  }

  explicit time(double ss) throw() : m_timeval()
  {
    const unsigned long s = (unsigned long)(ss);
    const long us = long((ss - s) * 1000000);
    reset(s, us);
  }

  /// Copy construct.
  time(const time& x) throw() : m_timeval(x.m_timeval) {}

  /// Destruct.
  ~time() throw() {}

  /// Create a time representing current wall-clock time.
  static time wall_clock_now() throw();

  /// Create a time representing the current process's user cpu usage.
  static time user_rusage() throw();

  /// Create a time representing the current process's sys cpu usage.
  static time sys_rusage() throw();

  /// Create a time representing the current process's total user+sys cpu usage.
  static time rusage_now() throw()
  { return rutz::time::user_rusage() + rutz::time::sys_rusage(); }

  /// Reset to given seconds+microseconds values.
  void reset(unsigned long s = 0, long us = 0) throw()
  {
    m_timeval.tv_sec = s;
    m_timeval.tv_usec = us;
    normalize(this->m_timeval);
  }

  /// Assignment operator.
  time& operator=(const time& x) throw()
  {
    m_timeval = x.m_timeval;
    return *this;
  }

  /// Addition operator.
  time operator+(const time& t2) const throw()
  {
    timeval result;
    result.tv_sec  = this->m_timeval.tv_sec  + t2.m_timeval.tv_sec;
    result.tv_usec = this->m_timeval.tv_usec + t2.m_timeval.tv_usec;

    normalize(result);

    return time(result);
  }

  /// In-place addition operator.
  time& operator+=(const time& t2) throw()
  {
    m_timeval.tv_sec  += t2.m_timeval.tv_sec;
    m_timeval.tv_usec += t2.m_timeval.tv_usec;

    normalize(this->m_timeval);

    return *this;
  }

  /// Subtraction operator.
  time operator-(const time& t2) const throw()
  {
    timeval result;
    result.tv_sec  = this->m_timeval.tv_sec  - t2.m_timeval.tv_sec;
    result.tv_usec = this->m_timeval.tv_usec - t2.m_timeval.tv_usec;

    normalize(result);

    return time(result);
  }

  /// In-place subtraction operator.
  time& operator-=(const time& t2) throw()
  {
    m_timeval.tv_sec  -= t2.m_timeval.tv_sec;
    m_timeval.tv_usec -= t2.m_timeval.tv_usec;

    normalize(this->m_timeval);

    return *this;
  }

  /// Less-than comparison
  bool operator<(const time& t2) const throw()
  {
    return (m_timeval.tv_sec < t2.m_timeval.tv_sec)
      ||
      (m_timeval.tv_sec == t2.m_timeval.tv_sec
       && m_timeval.tv_usec < t2.m_timeval.tv_usec);
  }

  /// Less-than-or-equal comparison
  bool operator<=(const time& t2) const throw()
  {
    return (m_timeval.tv_sec <= t2.m_timeval.tv_sec)
      ||
      (m_timeval.tv_sec == t2.m_timeval.tv_sec
       && m_timeval.tv_usec <= t2.m_timeval.tv_usec);
  }

  /// Equality comparison
  bool operator==(const time& t2) const throw()
  {
    return (m_timeval.tv_sec == t2.m_timeval.tv_sec
            && m_timeval.tv_usec == t2.m_timeval.tv_usec);
  }

  /// Greater-than comparison
  bool operator>(const time& t2) const throw()
  {
    return (m_timeval.tv_sec > t2.m_timeval.tv_sec)
      ||
      (m_timeval.tv_sec == t2.m_timeval.tv_sec
       && m_timeval.tv_usec > t2.m_timeval.tv_usec);
  }

  /// Greater-than-or-equal comparison
  bool operator>=(const time& t2) const throw()
  {
    return (m_timeval.tv_sec >= t2.m_timeval.tv_sec)
      ||
      (m_timeval.tv_sec == t2.m_timeval.tv_sec
       && m_timeval.tv_usec >= t2.m_timeval.tv_usec);
  }

  /// Return time in floating-point seconds.
  double sec() const throw()
  {
    return double(m_timeval.tv_sec) + m_timeval.tv_usec / 1000000.0;
  }

  /// Return time in floating-point milliseconds.
  double msec() const throw()
  {
    return (m_timeval.tv_sec * 1000.0) + (m_timeval.tv_usec / 1000.0);
  }

  /// Return time in floating-point microseconds.
  double usec() const throw()
  {
    return (m_timeval.tv_sec * 1000000.0) + double(m_timeval.tv_usec);
  }

  /// Return internal timeval.
  const timeval& tval() const throw()
  {
    return m_timeval;
  }

  /// Conversion operator to timeval.
  operator const timeval&() const throw()
  {
    return m_timeval;
  }

private:
  /// Avoid overflow/underflow in the timeval's microseconds field.
  static void normalize(timeval& t) throw()
  {
    long s = t.tv_usec / 1000000;

    if (s != 0)
      {
        t.tv_sec += s;
        t.tv_usec -= (s * 1000000);
      }
  }

  timeval m_timeval;
};

static const char __attribute__((used)) vcid_groovx_rutz_time_h_utc20050626084020[] = "$Id: time.h 8667 2007-08-07 18:57:58Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/time.h $";
#endif // !GROOVX_RUTZ_TIME_H_UTC20050626084020_DEFINED
