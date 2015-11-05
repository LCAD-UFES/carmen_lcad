/** @file rutz/spin_lock.h */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2007-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Tue Sep  4 14:12:19 2007
// commit: $Id: spin_lock.h 8741 2007-09-05 21:03:52Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/spin_lock.h $
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

#ifndef GROOVX_RUTZ_SPIN_LOCK_H_UTC20070904211219_DEFINED
#define GROOVX_RUTZ_SPIN_LOCK_H_UTC20070904211219_DEFINED

#if defined(HAVE_LIBKERN_OSATOMIC_H)
#  include "rutz/spin_lock_darwin.h"
namespace rutz { typedef spin_lock_darwin spin_lock_t; }

#elif defined(HAVE_PTHREAD_SPINLOCK_T)
#  include "rutz/spin_lock_pthreads.h"
namespace rutz { typedef spin_lock_pthreads spin_lock_t; }

#else
#  error no spinlock implementation available
#endif

namespace rutz
{

class spin_lock_locker
{
private:
  spin_lock_t* m_spin_lock; // if null, then we do no locking/unlocking

  spin_lock_locker(const spin_lock_locker&);
  spin_lock_locker& operator=(const spin_lock_locker&);

public:
  spin_lock_locker(spin_lock_t* lock) : m_spin_lock(lock)
  {
    if (m_spin_lock)
      m_spin_lock->lock();
  }

  ~spin_lock_locker()
  {
    if (m_spin_lock)
      m_spin_lock->unlock();
  }

  void unlock()
  {
    if (m_spin_lock)
      m_spin_lock->unlock();
    m_spin_lock = 0;
  }
};

}

#define GVX_SPIN_CONCAT2(x,y) x##y
#define GVX_SPIN_CONCAT(x,y) GVX_SPIN_CONCAT2(x,y)

#define GVX_SPIN_LOCK(x) \
  ::rutz::spin_lock_locker GVX_SPIN_CONCAT(anonymous_lock,__LINE__) (x)

#define GVX_SPIN_LOCK_IF(doit, x) \
  ::rutz::spin_lock_locker GVX_SPIN_CONCAT(anonymous_lock,__LINE__) ((doit) ? (x) : 0)

static const char vcid_groovx_rutz_spin_lock_h_utc20070904211219[] = "$Id: spin_lock.h 8741 2007-09-05 21:03:52Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/spin_lock.h $";
#endif // !GROOVX_RUTZ_SPIN_LOCK_H_UTC20070904211219DEFINED
