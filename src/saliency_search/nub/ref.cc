/** @file nub/ref.cc smart pointers (both strong and weak) using
    intrusive ref-counting with nub::object and derivatives */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2000-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Nov 16 09:06:27 2005
// commit: $Id: ref.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/ref.cc $
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

#ifndef GROOVX_NUB_REF_CC_UTC20051116170627_DEFINED
#define GROOVX_NUB_REF_CC_UTC20051116170627_DEFINED

#include "nub/ref.h"

#include "nub/objdb.h"

#include "rutz/error.h"
#include "rutz/demangle.h"

#include "rutz/debug.h"
GVX_DBG_REGISTER

#ifndef GVX_DEFAULT_REFVIS
#  define GVX_DEFAULT_REFVIS PRIVATE
#endif

namespace
{
  nub::ref_vis default_vis = nub::GVX_DEFAULT_REFVIS;
}

nub::ref_vis nub::get_default_ref_vis()
{
  return default_vis;
}

void nub::set_default_ref_vis(nub::ref_vis vis)
{
  default_vis = vis;
}

bool nub::detail::is_valid_uid(nub::uid id) throw()
{
  return nub::objectdb::instance().is_valid_uid(id);
}

nub::object* nub::detail::get_checked_item(nub::uid id)
{
  return nub::objectdb::instance().get_checked_obj(id);
}

void nub::detail::insert_item(nub::object* obj, ref_vis vis)
{
  if (vis == DEFAULT)
    {
      vis = default_vis;
    }

  switch (vis)
    {
    case PUBLIC:    nub::objectdb::instance().insert_obj(obj); break;
    case PROTECTED: nub::objectdb::instance().insert_obj_weak(obj); break;
    case PRIVATE:   /* nothing */ break;
    default:
      GVX_PANIC("unknown ref_vis enum value");
    }
}

static const char __attribute__((used)) vcid_groovx_nub_ref_cc_utc20051116170627[] = "$Id: ref.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/ref.cc $";
#endif // !GROOVX_NUB_REF_CC_UTC20051116170627DEFINED
