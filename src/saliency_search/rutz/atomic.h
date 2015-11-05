/** @file rutz/atomic.h Atomic integers (e.g. for threadsafe reference counting) */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2006-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Aug 10 16:56:03 2006
// commit: $Id: atomic.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/atomic.h $
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

#ifndef GROOVX_RUTZ_ATOMIC_H_UTC20070412044324_DEFINED
#define GROOVX_RUTZ_ATOMIC_H_UTC20070412044324_DEFINED

// For now we only have inline assembly for atomic integer operations
// for ix86 cpus; would be nice to eventually have analogous
// operations for ppc.
#if defined(INVT_CPU_IX86)
#  include "rutz/atomic_ix86.h"
namespace rutz { typedef ix86_atomic_int atomic_int_t; }

#elif defined(HAVE_LIBKERN_OSATOMIC_H)
#  include "rutz/atomic_darwin.h"
namespace rutz { typedef darwin_atomic_int atomic_int_t; }

#else
#  include "rutz/atomic_mutex.h"
namespace rutz { typedef mutex_atomic_int atomic_int_t; }
#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

static const char __attribute__((used)) vcid_groovx_rutz_atomic_h_utc20070412044324[] = "$Id: atomic.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/atomic.h $";
#endif // !GROOVX_RUTZ_ATOMIC_H_UTC20070412044324DEFINED
