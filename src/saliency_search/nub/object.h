/** @file nub/object.h base class for objects to be exposed to a
    scripting language */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Tue Jun  5 10:23:15 2001
// commit: $Id: object.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/object.h $
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

#ifndef GROOVX_NUB_OBJECT_H_UTC20050626084018_DEFINED
#define GROOVX_NUB_OBJECT_H_UTC20050626084018_DEFINED

#include "nub/refcounted.h"
#include "nub/uid.h"

namespace rutz
{
  class fstring;
}

namespace nub
{
  class object;
}

///////////////////////////////////////////////////////////////////////
/**
 *
 * nub::object is a base class for objects that are to be exposed to a
 * scripting language. Associates with each object a unique integer
 * identifier. nub::object's are reference counted for automatic
 * memory management, and are generally passed around via nub::ref's
 * or nub::soft_ref's, which automatically manage the reference count.
 *
 **/
///////////////////////////////////////////////////////////////////////


class nub::object : public nub::ref_counted
{
protected:
  /// Default constructor.
  /** Can't have an empty throw() spec here, because ref_counted's
      constructor might throw (because it has to allocate memory for
      a nub::ref_counts object). */
  object();

  /// Virtual destructor.
  virtual ~object() GVX_DTOR_NOTHROW;

public:
  /** Returns the unique id for this object. The unique id will always
      be strictly positive; zero is always an invalid unique id. */
  nub::uid id() const throw();

  /// Returns the typename of the full object.
  /** The result is a demangled version of \c typeid(*this).name(), which
      should very closely resemble the way the object was declared in
      source code. */
  rutz::fstring real_typename() const;

  /// Returns the (apparent) typename of the full object.
  /** The default implementation just returns real_typename(). However,
      certain kinds of objects -- e.g., proxy objects -- might usefully
      choose to have obj_typename() return something besides the
      real_typename(), in order to masquerade as a different type of
      object.  */
  virtual rutz::fstring obj_typename() const;

  /// Returns a short string describing the object by its typename and id.
  rutz::fstring unique_name() const;

private:
  nub::uid m_uid;
};

static const char __attribute__((used)) vcid_groovx_nub_object_h_utc20050626084018[] = "$Id: object.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/object.h $";
#endif // !GROOVX_NUB_OBJECT_H_UTC20050626084018_DEFINED
