/** @file rutz/timeformat.cc c++-friendly wrapper around strftime() */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Thu Jun 30 15:18:13 2005
// commit: $Id: timeformat.cc 14376 2011-01-11 02:44:34Z pez $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/timeformat.cc $
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

#ifndef GROOVX_RUTZ_TIMEFORMAT_CC_UTC20050630221813_DEFINED
#define GROOVX_RUTZ_TIMEFORMAT_CC_UTC20050630221813_DEFINED

#include <time.h>

#include "rutz/timeformat.h"

#include "rutz/fstring.h"

rutz::fstring rutz::format_time(const timeval& tval,
                                const char* formatcode)
{
  const time_t t = time_t(tval.tv_sec);

  struct tm* tt = localtime(&t);

  char buf[512];

  std::size_t count = strftime(buf, 512, formatcode, tt);

  return rutz::fstring(rutz::char_range(&buf[0], count));
}

static const char __attribute__((used)) vcid_groovx_rutz_timeformat_cc_utc20050630221813[] = "$Id: timeformat.cc 14376 2011-01-11 02:44:34Z pez $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/timeformat.cc $";
#endif // !GROOVX_RUTZ_TIMEFORMAT_CC_UTC20050630221813DEFINED
