/** @file rutz/atomic_darwin.h rutz/atomic_darwin.H Atomic integer operations implemented using Apple Darwin's OSAtomicAdd32Barrier() */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Oct  5 10:55:50 2006
// commit: $Id: atomic_darwin.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/atomic_darwin.h $
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

#ifndef GROOVX_RUTZ_ATOMIC_DARWIN_H_UTC20070412044456_DEFINED
#define GROOVX_RUTZ_ATOMIC_DARWIN_H_UTC20070412044456_DEFINED

#include <libkern/OSAtomic.h>
#include <limits>

namespace rutz
{

///  Atomic integer operations implemented using Apple Darwin's OSAtomicAdd32Barrier().
class darwin_atomic_int
{
private:
  int32_t x;

  darwin_atomic_int(const darwin_atomic_int&);
  darwin_atomic_int& operator=(const darwin_atomic_int&);

public:
  //! Construct with an initial value of 0.
  darwin_atomic_int() : x(0) {}

  //! Get the maximum representable value
  static int max_value() { return std::numeric_limits<int32_t>::max(); }

  //! Get the current value.
  int atomic_get() const
  { return x; }

  //! Set value to the given value \a v.
  void atomic_set(int v)
  { x = v; }

  //! Add \a v to the value.
  void atomic_add(int i)
  { OSAtomicAdd32Barrier(i, &x); }

  //! Subtract \a v from the value.
  void atomic_sub(int i)
  { OSAtomicAdd32Barrier(-i, &x); }

  //! Subtract \a v from the value; return true if the new value is zero.
  bool atomic_sub_test_zero(int i)
  { return (OSAtomicAdd32Barrier(-i, &x) == 0); }

  //! Increment the value by one.
  void atomic_incr()
  { OSAtomicAdd32Barrier(1, &x); }

  //! Decrement the value by one.
  void atomic_decr()
  { OSAtomicAdd32Barrier(-1, &x); }

  //! Decrement the value by one; return true if the new value is zero.
  bool atomic_decr_test_zero()
  { return (OSAtomicAdd32Barrier(-1, &x) == 0); }

  //! Increment the value by one; return true if the new value is zero.
  bool atomic_incr_test_zero()
  { return (OSAtomicAdd32Barrier(1, &x) == 0); }

  //! Add \a v to the value and return the new value
  int atomic_add_return(int i)
  { return OSAtomicAdd32Barrier(i, &x); }

  //! Subtract \a v from the value and return the new value
  int atomic_sub_return(int i)
  { return OSAtomicAdd32Barrier(-i, &x); }

  //! Increment the value by one and return the new value.
  int atomic_incr_return()
  { return OSAtomicAdd32Barrier(1, &x); }

  //! Decrement the value by one and return the new value.
  int atomic_decr_return()
  { return OSAtomicAdd32Barrier(-1, &x); }
};

} // end namespace rutz

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

static const char __attribute__((used)) vcid_groovx_rutz_atomic_darwin_h_utc20070412044456[] = "$Id: atomic_darwin.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/atomic_darwin.h $";
#endif // !GROOVX_RUTZ_ATOMIC_DARWIN_H_UTC20070412044456DEFINED
