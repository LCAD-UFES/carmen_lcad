/** @file rutz/abort.cc low-level GVX_ABORT macro */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Thu Jun 30 15:26:51 2005
// commit: $Id: abort.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/abort.cc $
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

#ifndef GROOVX_RUTZ_ABORT_CC_UTC20050630222651_DEFINED
#define GROOVX_RUTZ_ABORT_CC_UTC20050630222651_DEFINED

#include "rutz/abort.h"

#include <cstdlib>
#include <cstdio>

void rutz::debug::abort_aux (const char* what, const char* where,
                             int line_no) throw()
{
  fprintf(stderr, "Abort (%s:%d):\n\tgot '%s'\n\n",
          where, line_no, what);
  abort();
}

static const char __attribute__((used)) vcid_groovx_rutz_abort_cc_utc20050630222651[] = "$Id: abort.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/abort.cc $";
#endif // !GROOVX_RUTZ_ABORT_CC_UTC20050630222651DEFINED
