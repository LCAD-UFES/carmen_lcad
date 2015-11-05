/** @file nub/weak_handle.cc weak reference-counted handle for nub::object */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2000-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Nov 16 08:59:50 2005
// commit: $Id: weak_handle.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/weak_handle.cc $
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

#ifndef GROOVX_NUB_WEAK_HANDLE_CC_UTC20051116165950_DEFINED
#define GROOVX_NUB_WEAK_HANDLE_CC_UTC20051116165950_DEFINED

#include "nub/weak_handle.h"

#include "rutz/error.h"
#include "rutz/demangle.h"
#include "rutz/sfmt.h"

void nub::detail::throw_soft_ref_invalid(const std::type_info& info,
                                      const rutz::file_pos& pos)
{
  throw rutz::error(rutz::sfmt("attempted to access invalid object "
                               "in soft_ref<%s>",
                               rutz::demangled_name(info)),
                    pos);
}

static const char __attribute__((used)) vcid_groovx_nub_weak_handle_cc_utc20051116165950[] = "$Id: weak_handle.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/weak_handle.cc $";
#endif // !GROOVX_NUB_WEAK_HANDLE_CC_UTC20051116165950DEFINED
