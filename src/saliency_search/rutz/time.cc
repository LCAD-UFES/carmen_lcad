/** @file rutz/time.cc user-friendly wrapper around timeval */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Nov  7 16:58:26 2002
// commit: $Id: time.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/time.cc $
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

#ifndef GROOVX_RUTZ_TIME_CC_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_TIME_CC_UTC20050626084019_DEFINED

#include "rutz/time.h"

#include <sys/resource.h>
#include <time.h>

rutz::time rutz::time::wall_clock_now() throw()
{
  rutz::time t;
  gettimeofday(&t.m_timeval, /* timezone */ 0);
  return t;
}

rutz::time rutz::time::user_rusage() throw()
{
  rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return rutz::time(ru.ru_utime);
}

rutz::time rutz::time::sys_rusage() throw()
{
  rusage ru;
  getrusage(RUSAGE_SELF, &ru);
  return rutz::time(ru.ru_stime);
}

static const char __attribute__((used)) vcid_groovx_rutz_time_cc_utc20050626084019[] = "$Id: time.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/time.cc $";
#endif // !GROOVX_RUTZ_TIME_CC_UTC20050626084019_DEFINED
