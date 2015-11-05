/** @file nub/log.cc functions for hierarchical logging */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2001-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Jun 20 17:49:28 2001
// commit: $Id: log.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/log.cc $
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

#ifndef GROOVX_NUB_LOG_CC_UTC20050626084019_DEFINED
#define GROOVX_NUB_LOG_CC_UTC20050626084019_DEFINED

#include "log.h"

#include "nub/object.h"

#include "rutz/fstring.h"
#include "rutz/sfmt.h"
#include "rutz/shared_ptr.h"
#include "rutz/stopwatch.h"
#include "rutz/time.h"
#include "rutz/timeformat.h"

#include <algorithm>
#include <fstream>
#include <functional>
#include <iomanip>
#include <iostream>
#include <vector>

#include "rutz/trace.h"

using rutz::fstring;

namespace
{
  struct scope_info
  {
    scope_info(const fstring& name) : m_name(name), m_timer() {}

    fstring          m_name;
    rutz::stopwatch  m_timer;

    void print(std::ostream& os, const rutz::time& now) const
    {
      GVX_TRACE("scope_info::print");
      os << m_name << " @ ";

      os.setf(std::ios::showpoint | std::ios::fixed);

      os << std::setprecision(3)
         << m_timer.elapsed(now).msec() << " | ";
    }
  };

  std::vector<scope_info> scopes;
  rutz::shared_ptr<std::ofstream> s_log_fstream;
  bool s_copy_to_stdout = true;

  template <class str>
  inline void log_impl(std::ostream& os, str msg)
  {
    const rutz::time now = rutz::time::wall_clock_now();

    for (unsigned int i = 0; i < scopes.size(); ++i)
      {
        scopes[i].print(os, now);
      }

    os << msg << std::endl;
  }
}

void nub::logging::reset()
{
GVX_TRACE("nub::logging::reset");
  scopes.clear();

  log(rutz::sfmt("log reset %s",
                 rutz::format_time(rutz::time::wall_clock_now()).c_str()));
}

void nub::logging::add_scope(const fstring& name)
{
GVX_TRACE("nub::logging::add_scope");
  scopes.push_back(scope_info(name));
}

void nub::logging::remove_scope(const fstring& name)
{
GVX_TRACE("nub::logging::remove_scope");
  for (int i = int(scopes.size()); i > 0; /* decr in loop body */)
    {
      --i;
      if (scopes.at(i).m_name == name)
        {
          scopes.erase(scopes.begin() + i);

          // Return immediately, since this function is intended to remove
          // at most one scope from the stack of scopes.
          return;
        }
    }
}

void nub::logging::add_obj_scope(const nub::object& obj)
{
GVX_TRACE("nub::logging::add_obj_scope");

  const fstring scopename(obj.unique_name());

  add_scope(scopename);

  log(rutz::sfmt("entering %s", scopename.c_str()));
}

void nub::logging::remove_obj_scope(const nub::object& obj)
{
GVX_TRACE("nub::logging::remove_obj_scope");

  const fstring scopename = obj.unique_name();

  log(rutz::sfmt("leaving %s", scopename.c_str()));

  remove_scope(scopename);
}

void nub::logging::set_log_filename(const fstring& filename)
{
GVX_TRACE("nub::logging::set_log_filename");

  rutz::shared_ptr<std::ofstream> newfile
    (new std::ofstream(filename.c_str(), std::ios::out | std::ios::app));

  if (newfile->is_open() && newfile->good())
    s_log_fstream.swap(newfile);
}

void nub::logging::copy_to_stdout(bool shouldcopy)
{
GVX_TRACE("nub::logging::copy_to_stdout");
  s_copy_to_stdout = shouldcopy;
}

void nub::log(const char* msg)
{
GVX_TRACE("nub::log");
  if (s_copy_to_stdout)
    log_impl(std::cout, msg);

  if (s_log_fstream.get() != 0)
    log_impl(*s_log_fstream, msg);
}

void nub::log(const fstring& msg)
{
GVX_TRACE("nub::log");
  if (s_copy_to_stdout)
    log_impl(std::cout, msg);

  if (s_log_fstream.get() != 0)
    log_impl(*s_log_fstream, msg);
}

static const char __attribute__((used)) vcid_groovx_nub_log_cc_utc20050626084019[] = "$Id: log.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/nub/log.cc $";
#endif // !GROOVX_NUB_LOG_CC_UTC20050626084019_DEFINED
