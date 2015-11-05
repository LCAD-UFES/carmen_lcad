/** @file rutz/scopedptr.h smart pointer class for unshared objects
    (like std::auto_ptr but without transfer-of-ownership
    semantics) */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Oct 15 14:51:04 2004
// commit: $Id: scopedptr.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/scopedptr.h $
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

#ifndef GROOVX_RUTZ_SCOPEDPTR_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_SCOPEDPTR_H_UTC20050626084020_DEFINED

#include "rutz/algo.h"

namespace rutz
{
  //  #######################################################
  //  =======================================================

  /// A smart-pointer for unshared objects.

  /** \c scoped_ptr mimics a built-in pointer except that it guarantees
      deletion of the object pointed to, either on destruction of the
      scoped_ptr or via an explicit \c reset(). */

  template<class T>
  class scoped_ptr
  {
  public:
    /// Construct with pointer to given object (or null).
    explicit scoped_ptr( T* p=0 ) throw() :
      ptr(p)
    {}

    /// Construct with pointer to given object of related type.
    template <class TT>
    explicit scoped_ptr( TT* p ) throw() :
      ptr(p)
    {}

    /// Destructor.
    ~scoped_ptr()
    { delete ptr; }

    /// Reset with pointer to different object (or null).
    void reset( T* p=0 )
    {
      if ( ptr != p )
        { delete ptr; ptr = p; }
    }

    /// Dereference.
    T& operator*() const throw()
    { return *ptr; }

    /// Dereference for member access.
    T* operator->() const throw()
    { return ptr; }

    /// Get a pointer to the referred-to object.
    T* get() const throw()
    { return ptr; }

  private:
    scoped_ptr(const scoped_ptr& other);
    scoped_ptr& operator=(const scoped_ptr& other);

    T* ptr;
  };
}

static const char __attribute__((used)) vcid_groovx_rutz_scopedptr_h_utc20050626084020[] = "$Id: scopedptr.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/scopedptr.h $";
#endif // !GROOVX_RUTZ_SCOPEDPTR_H_UTC20050626084020_DEFINED
