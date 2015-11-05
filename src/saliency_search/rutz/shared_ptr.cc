/** @file rutz/shared_ptr.cc A thread-safe shared pointer class */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Aug 16 14:39:14 2006
// commit: $Id: shared_ptr.cc 8250 2007-04-12 07:34:07Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/shared_ptr.cc $
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

#ifndef GROOVX_RUTZ_SHARED_PTR_CC_UTC20070412044845_DEFINED
#define GROOVX_RUTZ_SHARED_PTR_CC_UTC20070412044845_DEFINED

#include "rutz/shared_ptr.h"

namespace
{
  rutz::shared_ptr_aux::ptr_check_function* g_function = 0;
}

rutz::shared_ptr_aux::ptr_check_function*
rutz::shared_ptr_aux::set_check_function(rutz::shared_ptr_aux::ptr_check_function* func)
{
  ptr_check_function* old = g_function;
  g_function = func;
  return old;
}

void rutz::shared_ptr_aux::check_ptr(const void* p)
{
  if (g_function != 0)
    (*g_function)(p);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

static const char __attribute__((used)) vcid_groovx_rutz_shared_ptr_cc_utc20070412044845[] = "$Id: shared_ptr.cc 8250 2007-04-12 07:34:07Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/shared_ptr.cc $";
#endif // !GROOVX_RUTZ_SHARED_PTR_CC_UTC20070412044845DEFINED
