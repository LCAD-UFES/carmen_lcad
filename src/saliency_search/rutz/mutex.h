/** @file rutz/mutex.h */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Tue Sep 13 09:15:02 2005
// commit: $Id: mutex.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/mutex.h $
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

#ifndef GROOVX_RUTZ_MUTEX_H_UTC20050913161502_DEFINED
#define GROOVX_RUTZ_MUTEX_H_UTC20050913161502_DEFINED

#include <pthread.h>

namespace rutz
{
  class mutex_lock_class;
  class mutex_unlocker;
}

//! Quick mutex locking class
/** Use this when we don't want to use bare pthread_mutex_lock() and
    pthread_mutex_unlock() calls, because they aren't
    exception-safe. Use the GVX_MUTEX_LOCK() macro to create a local
    temporary lock object. */
class rutz::mutex_lock_class
{
public:
  /// Throws an exception if pthread_mutex_lock() fails
  mutex_lock_class(pthread_mutex_t* mut = 0);

  ~mutex_lock_class() throw()
  {
    this->unlock();
  }

  bool is_locked() const throw() { return m_mutex != 0; }

  void unlock() throw();

  void swap(mutex_lock_class& that) throw()
  {
    pthread_mutex_t* const this_mutex = this->m_mutex;
    this->m_mutex = that.m_mutex;
    that.m_mutex = this_mutex;
  }

private:
  pthread_mutex_t* m_mutex;

  mutex_lock_class(const mutex_lock_class&); // not implemented
  mutex_lock_class& operator=(const mutex_lock_class&); // not implemented
};

#define GVX_MUTEX_CONCAT2(x,y) x##y
#define GVX_MUTEX_CONCAT(x,y) GVX_MUTEX_CONCAT2(x,y)

#define GVX_MUTEX_LOCK(x) \
  ::rutz::mutex_lock_class GVX_MUTEX_CONCAT(anonymous_lock,__LINE__) (x)

static const char __attribute__((used)) vcid_groovx_rutz_mutex_h_utc20050913161502[] = "$Id: mutex.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/mutex.h $";
#endif // !GROOVX_RUTZ_MUTEX_H_UTC20050913161502DEFINED
