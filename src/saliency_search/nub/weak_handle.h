/** @file nub/weak_handle.h weak reference-counted handle for nub::object */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2000-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Nov 16 08:50:18 2005
// commit: $Id: weak_handle.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/weak_handle.h $
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

#ifndef GROOVX_NUB_WEAK_HANDLE_H_UTC20051116165018_DEFINED
#define GROOVX_NUB_WEAK_HANDLE_H_UTC20051116165018_DEFINED

#include "nub/refcounted.h"
#include "nub/types.h"

#include "rutz/algo.h"         // for rutz::swap2()
#include "rutz/fileposition.h" // for SRC_POS macro

#include <typeinfo>

namespace nub
{
  namespace detail
  {
    template <class T> class weak_handle;

    void throw_soft_ref_invalid(const std::type_info& info, const rutz::file_pos& pos);
  }
}

/// Internal helper class for soft_ref.
/** weak_handle manages memory etc., and provides one important
    guarantee: it will never return an invalid pointer from get()
    (an exception will be raised if this would fail). */
template <class T>
class nub::detail::weak_handle
{
private:
  static nub::ref_counts* get_counts(T* master, ref_type tp) throw()
  {
    return (master && (tp == WEAK || master->is_not_shareable())) ?
      master->get_counts() : 0;
  }

public:
  weak_handle(T* master, ref_type tp) throw()
    :
    m_master(master),
    m_counts(get_counts(master, tp))
  { this->acquire(); }

  ~weak_handle() throw()
  { this->release(); }

  weak_handle(const weak_handle& other) throw()
    :
    m_master(other.m_master),
    m_counts(other.m_counts)
  { this->acquire(); }

  weak_handle& operator=(const weak_handle& other) throw()
  {
    weak_handle other_copy(other);
    this->swap(other_copy);
    return *this;
  }

  bool is_valid() const throw()
  {
    if (m_counts == 0) // implies we are using strong ref's
      {
        return (m_master != 0);
      }

    // else... (m_counts != 0) implies we are using weak ref's

    return m_counts->is_owner_alive();
  }

  T* get()      const         { this->ensure_valid(); return m_master; }
  T* get_weak() const throw() { return this->is_valid() ? m_master : 0; }

  ref_type get_ref_type() const throw()
  {
    return (m_master && !m_counts) ? STRONG : WEAK;
  }

private:
  void acquire() const throw()
  {
    if (m_master)
      {
        if (m_counts) m_counts->acquire_weak();
        else          m_master->incr_ref_count();
      }
  }

  void release() const throw()
  {
    if (m_master)
      {
        if (m_counts) m_counts->release_weak();
        else          m_master->decr_ref_count();
      }

    m_counts = 0; m_master = 0;
  }

  void ensure_valid() const
  {
    if (!is_valid())
      nub::detail::throw_soft_ref_invalid(typeid(T), SRC_POS);
  }

  void swap(weak_handle& other) throw()
  {
    rutz::swap2(m_master, other.m_master);
    rutz::swap2(m_counts, other.m_counts);
  }

  // In order to avoid storing a separate bool indicating whether we
  // are using strong or weak refs, we use the following convention:
  // if we are strong ref-counting, then m_counts stays null, but if
  // we are weak ref-counting, then m_counts will be non-null.

  mutable T* m_master;
  mutable nub::ref_counts* m_counts;
};

static const char __attribute__((used)) vcid_groovx_nub_weak_handle_h_utc20051116165018[] = "$Id: weak_handle.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/weak_handle.h $";
#endif // !GROOVX_NUB_WEAK_HANDLE_H_UTC20051116165018DEFINED
