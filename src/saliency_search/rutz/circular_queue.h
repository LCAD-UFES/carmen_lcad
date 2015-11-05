/** @file rutz/circular_queue.h */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Oct  5 10:43:45 2006
// commit: $Id: circular_queue.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/circular_queue.h $
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

#ifndef GROOVX_RUTZ_CIRCULAR_QUEUE_H_UTC20061005174345_DEFINED
#define GROOVX_RUTZ_CIRCULAR_QUEUE_H_UTC20061005174345_DEFINED

#include "rutz/atomic.h"

#include <cstddef> // for size_t

namespace rutz
{
  /// Circular fixed-size queue; T must have a default constructor
  /** This class is designed to be used in a situation where one
      thread is filling the queue while another thread is emptying the
      queue; in that situation, no locks should be needed to access
      the queue safely. */
  template <class T>
  class circular_queue
  {
  public:
    circular_queue(const size_t n)
      :
      m_q(new entry[n == 0 ? 1 : n]),
      m_q_size(n == 0 ? 1 : n),
      m_front(0),
      m_back(0)
    {}

    ~circular_queue()
    {
      delete [] m_q;
    }

    /// Get the number of spaces in the queue (not all of which may be currently occupied)
    size_t size() const { return m_q_size; }

    /// Returns true of the pop succeeded
    bool pop_front(T& dest)
    {
      if (m_q[m_front].valid.atomic_get() == 0)
        // the queue is empty right now:
        return false;

      dest = m_q[m_front].value;

      m_q[m_front].value = T();
      m_q[m_front].valid.atomic_set(0);
      if (m_front+1 == m_q_size)
        m_front = 0;
      else
        ++m_front;

      return true;
    }

    /// Returns true if the push succeeded
    bool push_back(const T& val)
    {
      if (m_q[m_back].valid.atomic_get() != 0)
        // we don't have any space in the queue for another entry
        // right now:
        return false;

      m_q[m_back].value = val;
      m_q[m_back].valid.atomic_set(1);
      if (m_back+1 == m_q_size)
        m_back = 0;
      else
        ++m_back;

      return true;
    }

  private:
    circular_queue(const circular_queue&);
    circular_queue& operator=(const circular_queue&);

    struct entry
    {
      entry() : value(), valid() {}

      T value;
      rutz::atomic_int_t valid;
    };

    entry* const m_q;
    size_t m_q_size;
    size_t m_front;
    size_t m_back;
  };
}

static const char vcid_groovx_rutz_circular_queue_h_utc20061005174345[] = "$Id: circular_queue.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/circular_queue.h $";
#endif // !GROOVX_RUTZ_CIRCULAR_QUEUE_H_UTC20061005174345DEFINED
