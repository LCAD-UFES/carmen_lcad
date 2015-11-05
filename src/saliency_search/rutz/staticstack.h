/** @file rutz/staticstack.h STL-style class for push/pop stacks with
    a fixed maximum size */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Oct 13 16:34:11 2004
// commit: $Id: staticstack.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/staticstack.h $
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

#ifndef GROOVX_RUTZ_STATICSTACK_H_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_STATICSTACK_H_UTC20050626084019_DEFINED

#include "rutz/abort.h"

namespace rutz
{
  /// STL-style class for fixed-size stacks whose size is known at compile time.
  /** Because this class doesn't rely on dynamically-allocated memory,
      it can be both highly efficient and exception-safe. */
  template <typename T, unsigned int N>
  class static_stack
  {
  public:
    static_stack() throw() : vec(), sz(0) {}

    static_stack(const static_stack& other) throw() :
      vec(), sz(0)
    {
      *this = other;
    }

    static_stack& operator=(const static_stack& other) throw()
    {
      sz = other.sz;

      for (unsigned int i = 0; i < sz; ++i)
        {
          vec[i] = other.vec[i];
        }

      return *this;
    }

    unsigned int size() const throw() { return sz; }

    static unsigned int capacity() throw() { return N; }

    bool push(T p) throw()
    {
      if (sz >= N)
        return false;

      vec[sz++] = p;
      return true;
    }

    void pop() throw()
    {
      if (sz == 0)
        GVX_ABORT("underflow in static_stack::pop");

      --sz;
    }

    T top() const throw()
    {
      return (sz > 0) ? vec[sz-1] : 0;
    }

    T at(unsigned int i) const throw()
    {
      return (i < sz) ? vec[i] : 0;
    }

    T operator[](unsigned int i) const throw() { return at(i); }

    typedef       T*       iterator;
    typedef const T* const_iterator;

    iterator begin() throw() { return &vec[0]; }
    iterator end()   throw() { return &vec[0] + sz; }

    const_iterator begin() const throw() { return &vec[0]; }
    const_iterator end()   const throw() { return &vec[0] + sz; }

  private:
    T vec[N];
    unsigned int sz;
  };
}

static const char __attribute__((used)) vcid_groovx_rutz_staticstack_h_utc20050626084019[] = "$Id: staticstack.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/staticstack.h $";
#endif // !GROOVX_RUTZ_STATICSTACK_H_UTC20050626084019_DEFINED
