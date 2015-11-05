/*!@file Util/SimTime.H */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/SimTime.H $
// $Id: SimTime.H 13293 2010-04-23 22:50:48Z dberg $
//

#ifndef UTIL_SIMTIME_H_DEFINED
#define UTIL_SIMTIME_H_DEFINED

#include "Util/Types.H" // for int64

#include <limits>
#include <string>

#ifdef MAX
#undef MAX
#endif

//! Encapsulated representation of simulation time
/*! Basic arithmetic operators are supported. The idea is to not rely
    on a particular representation, but currently we are using a
    64-bit signed integer to represent the number of microseconds or
    nanoseconds.

    With this representation, the possible range of values is
    -2^(-63)/10^9 seconds = -9.2e+9 seconds, to 2^(63)/10^9 seconds =
    9.2e+9 seconds. So, roughly: 33 bits are used to represent the
    integer portion, 30 bits to represent the fractional portion, and
    1 bit for the sign. That means that this representation has a
    somewhat larger range of values than does the standard 32+32 bit
    'struct timeval' from <sys/time.h>, which uses 32 bits for the
    integral portion, and 32 bits for the fractional portion (of which
    12 bits are wasted since the fractional portion represents
    microseconds and so only 20 bits are needed to get up to 10^6).

    In order to avoid losing precision, you should avoid calling
    secs() or msecs() etc. and doing math on the results; instead, do
    math using SimTime's builtin operators, and only convert to secs()
    or msecs() when absolutely necessary (e.g., to display the time to
    the user). */
class SimTime
{
public:
  //! Default constructor; initialize to time=0
  SimTime() : nseconds(0) {}

  //! Pseudo-constructor; returns time=0
  static SimTime ZERO() { return SimTime(0LL); }

  //! Pseudo-constructor; builds the largest representable time
  static SimTime MAX() { return SimTime(std::numeric_limits<int64>::max()); }

  //! Pseudo-constructor; returns time=s seconds
  static SimTime SECS(double s) { return SimTime(int64(1000000000LL*s)); }

  //! Pseudo-constructor; returns time=ms milliseconds
  static SimTime MSECS(double ms) { return SimTime(int64(1000000LL*ms)); }

  //! Pseudo-constructor; returns time=us microseconds
  static SimTime USECS(double us) { return SimTime(int64(1000LL*us)); }

  //! Pseudo-constructor; returns time=ns nanoseconds
  static SimTime NSECS(int64 ns) { return SimTime(ns); }

  //! Pseudo-constructor; returns time=1/hz seconds
  static SimTime HERTZ(double hz) { return SimTime::SECS(1.0/hz); }

  //! Return a delta-t close to timestep for traversing the given interval
  static SimTime computeDeltaT(const SimTime& interval,
                               const SimTime& timestep)
  {
    int64 nsteps = interval.nseconds / timestep.nseconds;
    if (nsteps <= 0) nsteps = 1;
    return SimTime(interval.nseconds / nsteps);
  }

  //! Convert to string
  /*! If with_suffix==true, then write a suffix after the number
      indicating the units (i.e., 's', 'ms', 'us'). If
      with_suffix==false, the no suffix is written and the number will
      represent the number of seconds. */
  std::string toString(bool with_suffix=false) const;

  //! Convert from string
  /*! If default_suffix=null, then a valid suffix must be present to
      indicate the units represented by the number. If
      default_suffix==non-null, then that suffix will be used if the
      string itself doesn't contain a units suffix.  */
  static SimTime fromString(const std::string& s,
                            const char* default_suffix = 0);

  //! Return the number of seconds represented
  double secs() const
  { return double(this->nseconds) / 1000000000LL; }

  //! Return the number of milliseconds represented
  double msecs() const
  { return double(this->nseconds) / 1000000LL; }

  //! Return the number of microseconds represented
  double usecs() const
  { return double(this->nseconds) / 1000LL; }

  //! Return the number of nanoseconds represented
  int64 nsecs() const
  { return this->nseconds; }

  //! Return the rate (inverse seconds) represented
  double hertz() const
  { 
    return (this->nseconds == 0LL) ? 
      0.0 : 
      1000000000LL / double(this->nseconds); 
  }
  
  //! Return true if the time represented is non-zero
  bool nonzero() const
  { return this->nseconds != 0; }

  //! Plus-equals
  SimTime& operator+=(const SimTime& step)
  { this->nseconds += step.nseconds; return *this; }

  //! result=time1+time2
  SimTime operator+(const SimTime& that) const
  { return SimTime(this->nseconds + that.nseconds); }

  //! result=time1-time2
  SimTime operator-(const SimTime& that) const
  { return SimTime(this->nseconds - that.nseconds); }


  //! result=time*factor
  SimTime operator*(const double x) const
  { return SimTime(int64(this->nseconds * x)); }

  //! result=time*factor
  SimTime operator*(const int x) const
  { return SimTime(this->nseconds * x); }

  //! result=time*factor
  SimTime operator*(const int64 x) const
  { return SimTime(this->nseconds * x); }


  //! compare time1<time2
  bool operator<(const SimTime& that) const
  { return this->nseconds < that.nseconds; }

  //! compare time1<=time2
  bool operator<=(const SimTime& that) const
  { return this->nseconds <= that.nseconds; }

  //! compare time1>time2
  bool operator>(const SimTime& that) const
  { return this->nseconds > that.nseconds; }

  //! compare time1>=time2
  bool operator>=(const SimTime& that) const
  { return this->nseconds >= that.nseconds; }

  //! compare time1==time2
  bool operator==(const SimTime& that) const
  { return this->nseconds == that.nseconds; }

  //! compare time1!=time2
  bool operator!=(const SimTime& that) const
  { return this->nseconds != that.nseconds; }

private:
  // this constructor is private since public users should call one of
  // pseudo-constructors SimTime::SECS() or SimTime::MSECS() so that
  // it is always clear at the point of the call which units are being
  // used (i.e. either seconds or milliseconds)
  explicit SimTime(int64 ns) : nseconds(ns) {}

  int64 nseconds;
};

//! SimTime overload: format is floating-point number of seconds
inline std::string convertToString(const SimTime& val)
{
  // pass false so that we don't get a units suffix
  return val.toString(false);
}

//! SimTime overload: format is floating-point number of seconds;
inline void convertFromString(const std::string& str, SimTime& val)
{
  // pass "s" to specify that the default units are seconds
  val = SimTime::fromString(str, "s");
}

// ######################################################################
// ######################################################################
// additional inline free functions for SimTime
// ######################################################################
// ######################################################################

// ######################################################################
inline SimTime operator*(const double x, const SimTime& t)
{ return t * x; }

// ######################################################################
inline SimTime operator*(const int x, const SimTime& t)
{ return t * x; }

// ######################################################################
inline SimTime operator*(const int64 x, const SimTime& t)
{ return t * x; }

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_SIMTIME_H_DEFINED
