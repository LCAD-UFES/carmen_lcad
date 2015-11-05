/** @file rutz/atomic_mutex.h rutz/atomic_mutex.h Heavyweight atomic integer implementation using mutexes */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Oct  5 10:55:50 2006
// commit: $Id: atomic_mutex.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/atomic_mutex.h $
//
// --------------------------------------------------------------------
//
// This file is part of GroovX.
//   [http://www.klab.caltech.edu/rjpeters/groovx/]
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

#ifndef GROOVX_RUTZ_ATOMIC_MUTEX_H_UTC20070412044614_DEFINED
#define GROOVX_RUTZ_ATOMIC_MUTEX_H_UTC20070412044614_DEFINED

#include <limits>
#include <pthread.h>

namespace rutz
{

/// Heavyweight atomic integer implementation using mutexes
class mutex_atomic_int
{
private:
  int x;
  pthread_mutex_t mut;

  mutex_atomic_int(const mutex_atomic_int&);
  mutex_atomic_int& operator=(const mutex_atomic_int&);

public:
  //! Construct with an initial value of 0.
  mutex_atomic_int() : x(0)
  { pthread_mutex_init(&mut, NULL); }

  //! Get the maximum representable value
  static int max_value() { return std::numeric_limits<int>::max(); }

  //! Get the current value.
  int atomic_get() const
  { return x; }

  //! Set value to the given value \a v.
  void atomic_set(int v)
  { pthread_mutex_lock(&mut); x = v; pthread_mutex_unlock(&mut); }

  //! Add \a v to the value.
  void atomic_add(int i)
  { pthread_mutex_lock(&mut); x += i; pthread_mutex_unlock(&mut); }

  //! Subtract \a v from the value.
  void atomic_sub(int i)
  { pthread_mutex_lock(&mut); x -= i; pthread_mutex_unlock(&mut); }

  //! Subtract \a v from the value; return true if the new value is zero.
  bool atomic_sub_test_zero(int i)
  {
    bool ret;
    pthread_mutex_lock(&mut);
    ret = bool((x -= i) == 0);
    pthread_mutex_unlock(&mut);
    return ret;
  }

  //! Increment the value by one.
  void atomic_incr()
  { pthread_mutex_lock(&mut); ++x; pthread_mutex_unlock(&mut); }

  //! Decrement the value by one.
  void atomic_decr()
  { pthread_mutex_lock(&mut); --x; pthread_mutex_unlock(&mut); }

  //! Decrement the value by one; return true if the new value is zero.
  bool atomic_decr_test_zero()
  {
    bool ret;
    pthread_mutex_lock(&mut);
    ret = bool(--x == 0);
    pthread_mutex_unlock(&mut);
    return ret;
  }

  //! Increment the value by one; return true if the new value is zero.
  bool atomic_incr_test_zero()
  {
    bool ret;
    pthread_mutex_lock(&mut);
    ret = bool(++x == 0);
    pthread_mutex_unlock(&mut);
    return ret;
  }

  //! Add \a v to the value and return the new value
  int atomic_add_return(int i)
  {
    int ret;
    pthread_mutex_lock(&mut);
    ret = (x += i);
    pthread_mutex_unlock(&mut);
    return ret;
  }

  //! Subtract \a v from the value and return the new value
  int atomic_sub_return(int i)
  {
    int ret;
    pthread_mutex_lock(&mut);
    ret = (x -= i);
    pthread_mutex_unlock(&mut);
    return ret;
  }

  //! Increment the value by one and return the new value.
  int atomic_incr_return()
  {
    int ret;
    pthread_mutex_lock(&mut);
    ret = ++x;
    pthread_mutex_unlock(&mut);
    return ret;
  }

  //! Decrement the value by one and return the new value.
  int atomic_decr_return()
  {
    int ret;
    pthread_mutex_lock(&mut);
    ret = --x;
    pthread_mutex_unlock(&mut);
    return ret;
  }
};

} // end namespace rutz

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

static const char __attribute__((used)) vcid_groovx_rutz_atomic_mutex_h_utc20070412044614[] = "$Id: atomic_mutex.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/atomic_mutex.h $";
#endif // !GROOVX_RUTZ_ATOMIC_MUTEX_H_UTC20070412044614DEFINED
