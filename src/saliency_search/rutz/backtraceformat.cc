/** @file rutz/backtraceformat.cc generate a human-readable string from
    a rutz::backtrace object */
///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Thu Jun 30 14:41:29 2005
// commit: $Id: backtraceformat.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/backtraceformat.cc $
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

#ifndef GROOVX_RUTZ_BACKTRACEFORMAT_CC_UTC20050630214129_DEFINED
#define GROOVX_RUTZ_BACKTRACEFORMAT_CC_UTC20050630214129_DEFINED

#include "rutz/backtraceformat.h"

#include "rutz/backtrace.h"
#include "rutz/compat_snprintf.h"
#include "rutz/fstring.h"
#include "rutz/trace.h" // for rutz::prof

#include <cstdio> // for snprintf()
#include <sstream>

rutz::fstring rutz::format(const rutz::backtrace& bt)
{
  const unsigned int size = bt.size();

  if (size == 0) return rutz::fstring();

  std::ostringstream result;

  const int LINELEN = 256;
  char line[LINELEN];

  unsigned int s = size;
  int width = 0;
  while (s != 0)
    { s /= 10; ++width; }

  for (unsigned int i = size; i > 0; --i)
    {
      snprintf(&line[0], LINELEN, "[%*d] %-35s (%s:%d)\n",
               width,
               size - i,
               bt[i-1]->context_name(),
               bt[i-1]->src_file_name(),
               bt[i-1]->src_line_no());

      result << &line[0];
    }

  return rutz::fstring(result.str().c_str());
}

static const char __attribute__((used)) vcid_groovx_rutz_backtraceformat_cc_utc20050630214129[] = "$Id: backtraceformat.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/backtraceformat.cc $";
#endif // !GROOVX_RUTZ_BACKTRACEFORMAT_CC_UTC20050630214129DEFINED
