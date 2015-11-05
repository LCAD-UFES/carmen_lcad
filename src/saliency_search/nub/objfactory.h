/** @file nub/objfactory.h generic factory for creating objects given
    a type name */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Sat Jun 26 23:40:06 1999
// commit: $Id: objfactory.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objfactory.h $
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

#ifndef GROOVX_NUB_OBJFACTORY_H_UTC20050626084019_DEFINED
#define GROOVX_NUB_OBJFACTORY_H_UTC20050626084019_DEFINED

#include "nub/object.h"
#include "nub/ref.h"

#include "rutz/factory.h"

namespace nub
{
  class obj_factory;
}

/// Singleton wrapper for a nub::object factory.
class nub::obj_factory
  :
  public rutz::factory<nub::soft_ref<nub::object> >
{
protected:
  /// Default constructor.
  obj_factory();

  /// Virtual destructor.
  virtual ~obj_factory() throw();

public:
  /// Return the singleton instance.
  static obj_factory& instance();
};

static const char __attribute__((used)) vcid_groovx_nub_objfactory_h_utc20050626084019[] = "$Id: objfactory.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objfactory.h $";
#endif // !GROOVX_NUB_OBJFACTORY_H_UTC20050626084019_DEFINED
