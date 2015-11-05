/** @file rutz/trace.cc GVX_TRACE macro for user-controlled tracing and
    profiling */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Mon Jan  4 08:00:00 1999
// commit: $Id: trace.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/trace.cc $
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

#ifndef GROOVX_RUTZ_TRACE_CC_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_TRACE_CC_UTC20050626084020_DEFINED

#include "rutz/trace.h"

#include "rutz/backtrace.h"

#include <iostream>

///////////////////////////////////////////////////////////////////////
//
// File-scope helper functions and data
//
///////////////////////////////////////////////////////////////////////

namespace
{
  unsigned int             g_max_trace_level     = 20;
  bool                     g_do_global_trace     = false;

  void print_in(const char* context_name) throw()
  {
    const unsigned int n = rutz::backtrace::current().size();

    if (n < g_max_trace_level)
      {
        for (unsigned int i = 0; i < n; ++i)
          {
            std::cerr << "|   ";
          }
        // Test whether n has 1, 2, or >3 digits
        if (n < 10)
          std::cerr << n << "-->> ";
        else if (n < 100)
          std::cerr << n << "->> ";
        else
          std::cerr << n << ">> ";

        std::cerr << context_name << '\n';
      }
  }

  void print_out(const char* context_name) throw()
  {
    const unsigned int n = rutz::backtrace::current().size();

    if (n < g_max_trace_level)
      {
        for (unsigned int i = 0; i < n; ++i)
          {
            std::cerr << "|   ";
          }
        std::cerr << "| <<--";

        std::cerr << context_name << '\n';
      }

    if (n == 0)
      std::cerr << '\n';
  }
}

///////////////////////////////////////////////////////////////////////
//
// rutz::trace member definitions
//
///////////////////////////////////////////////////////////////////////

bool rutz::trace::get_global_trace() throw()
{
  return g_do_global_trace;
}

void rutz::trace::set_global_trace(bool on_off) throw()
{
  g_do_global_trace = on_off;
}

unsigned int rutz::trace::get_max_level() throw()
{
  return g_max_trace_level;
}

void rutz::trace::set_max_level(unsigned int lev) throw()
{
  g_max_trace_level = lev;
}

rutz::trace::trace(prof& p, bool use_msg) throw()
  :
  m_prof(p),
  m_start(),
  m_should_print_msg(g_do_global_trace || use_msg),
  m_should_pop(rutz::backtrace::current().push(&p)),
  m_timing_mode(rutz::prof::get_timing_mode())
{
  if (this->m_should_print_msg)
    print_in(this->m_prof.context_name());

  // We want this to be the last thing in the constructor, so that we
  // don't include the rest of the constructor runtime in our elapsed
  // time measurement:
  this->m_start = rutz::prof::get_now_time(this->m_timing_mode);
}

rutz::trace::~trace() throw()
{
  // We want this to be the first thing in the destructor, so that we
  // don't include the rest of the destructor runtime in our elapsed
  // time measurement:
  const rutz::time finish = rutz::prof::get_now_time(this->m_timing_mode);
  const rutz::time elapsed = finish - this->m_start;

  this->m_prof.add_time(elapsed);

  // Do a backtrace::pop() only if the corresponding backtrace::push()
  // succeeeded in the constructor (it could have failed due to memory
  // exhaustion, etc.):
  if (this->m_should_pop)
    {
      rutz::backtrace& c = rutz::backtrace::current();
      c.pop();
      rutz::prof* parent = c.top();
      if (parent != 0)
        parent->add_child_time(elapsed);
    }

  if (this->m_should_print_msg)
    print_out(this->m_prof.context_name());
}

static const char __attribute__((used)) vcid_groovx_rutz_trace_cc_utc20050626084020[] = "$Id: trace.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/trace.cc $";
#endif // !GROOVX_RUTZ_TRACE_CC_UTC20050626084020_DEFINED
