/** @file rutz/spin_lock_pthreads.h */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2007-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Sep  5 12:08:48 2007
// commit: $Id: spin_lock_pthreads.h 8741 2007-09-05 21:03:52Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/spin_lock_pthreads.h $
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

#ifndef GROOVX_RUTZ_SPIN_LOCK_PTHREADS_H_UTC20070905190848_DEFINED
#define GROOVX_RUTZ_SPIN_LOCK_PTHREADS_H_UTC20070905190848_DEFINED

#include "rutz/error.h"

namespace rutz
{

class spin_lock_pthreads
{
private:
  pthread_spinlock_t m_lock;

  spin_lock_pthreads(const spin_lock_pthreads&); // not implemented
  spin_lock_pthreads& operator=(const spin_lock_pthreads&); // not implemented

public:
  /// pthreads spin-lock constructor; throws an exception if pthread_spin_init() fails
  spin_lock_pthreads();

  ~spin_lock_pthreads();

  void lock()
  {
    if (0 != pthread_spin_lock(&m_lock))
      throw rutz::error("pthread_spin_lock() failed", SRC_POS);
  }

  bool try_lock()
  { return (0 == pthread_spin_trylock(&m_lock)); }

  void unlock()
  { pthread_spin_unlock(&m_lock); }
};

}

static const char vcid_groovx_rutz_spin_lock_pthreads_h_utc20070905190848[] = "$Id: spin_lock_pthreads.h 8741 2007-09-05 21:03:52Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/spin_lock_pthreads.h $";
#endif // !GROOVX_RUTZ_SPIN_LOCK_PTHREADS_H_UTC20070905190848DEFINED
