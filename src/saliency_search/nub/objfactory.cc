/** @file nub/objfactory.cc generic factory for creating objects given
    a type name */
///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Jun 30 15:01:02 1999
// commit: $Id: objfactory.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objfactory.cc $
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

#ifndef GROOVX_NUB_OBJFACTORY_CC_UTC20050626084019_DEFINED
#define GROOVX_NUB_OBJFACTORY_CC_UTC20050626084019_DEFINED

#include "objfactory.h"

namespace
{
  nub::obj_factory* s_instance = 0;
}

nub::obj_factory::obj_factory() :
  rutz::factory<soft_ref<nub::object> >() {}

nub::obj_factory::~obj_factory() throw() {}

nub::obj_factory& nub::obj_factory::instance()
{
  if (s_instance == 0)
    {
      s_instance = new obj_factory;
    }
  return *s_instance;
}

static const char __attribute__((used)) vcid_groovx_nub_objfactory_cc_utc20050626084019[] = "$Id: objfactory.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/objfactory.cc $";
#endif // !GROOVX_NUB_OBJFACTORY_CC_UTC20050626084019_DEFINED
