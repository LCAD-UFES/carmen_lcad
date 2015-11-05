/** @file rutz/prof.cc class for accumulating profiling information */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Thu Jun 30 14:47:13 2005
// commit: $Id: prof.cc 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/prof.cc $
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

#ifndef GROOVX_RUTZ_PROF_CC_UTC20050630214713_DEFINED
#define GROOVX_RUTZ_PROF_CC_UTC20050630214713_DEFINED

#include "rutz/prof.h"

#include "rutz/abort.h"
#include "rutz/mutex.h"
#include "rutz/staticstack.h"

#include <algorithm> // for std::stable_sort()
#include <cstdio>
#include <functional>
#include <iomanip>
#include <new> // for std::nothrow
#include <ostream>
#include <pthread.h>
#include <string>

namespace
{
  //
  // data and thread info for the profile output file
  //

  bool            g_pdata_print_at_exit = false;
  std::string     g_pdata_fname = "prof.out";
  FILE*           g_pdata_file = 0;
  pthread_once_t  g_pdata_file_once = PTHREAD_ONCE_INIT;
  pthread_mutex_t g_pdata_mutex = PTHREAD_MUTEX_INITIALIZER;

  void open_pdata_file()
  {
    if (g_pdata_fname.length() > 0)
      g_pdata_file = fopen(g_pdata_fname.c_str(), "w");

    if (g_pdata_file == 0)
      {
        fprintf(stderr,
                "couldn't open profile file '%s' for writing\n",
                g_pdata_fname.c_str());
      }
  }

  //
  // data and thread info for a global list of all rutz::prof objects
  //

  typedef rutz::static_stack<rutz::prof*, 2048> prof_list;

  prof_list*      g_prof_list = 0;
  pthread_once_t  g_prof_list_once = PTHREAD_ONCE_INIT;
  pthread_mutex_t g_prof_list_mutex = PTHREAD_MUTEX_INITIALIZER;

  void initialize_prof_list()
  {
    // Q: Why do a dynamic allocation here instead of just having a
    // static object?

    // A: With a static object, we could potentially run in to trouble
    // during program exit, if somebody's destructor called
    // backtrace::current() after the local static object's destructor
    // (i.e., ~backtrace()) had itself already been run. On the other
    // hand, with a dynamically-allocated object, we can just let the
    // memory dangle (it's not really a memory "leak" since the amount
    // of memory is finite and bounded), so the object will never
    // become invalid, even during program shutdown.

    g_prof_list = new (std::nothrow) prof_list;

    if (g_prof_list == 0)
      GVX_ABORT("memory allocation failed");
  }

  prof_list& all_profs() throw()
  {
    pthread_once(&g_prof_list_once, &initialize_prof_list);

    return *g_prof_list;
  }

  //
  // data and thread info for a global start time
  //

  rutz::time     g_start;
  pthread_once_t g_start_once = PTHREAD_ONCE_INIT;

  void initialize_start_time()
  {
    g_start = rutz::prof::get_now_time(rutz::prof::get_timing_mode());
  }
}

///////////////////////////////////////////////////////////////////////
//
// rutz::prof member definitions
//
///////////////////////////////////////////////////////////////////////

rutz::prof::timing_mode rutz::prof::s_timing_mode = rutz::prof::RUSAGE;

rutz::prof::prof(const char* s, const char* fname, int lineno)  throw():
  m_context_name(s),
  m_src_file_name(fname),
  m_src_line_no(lineno)
{
  reset();

  {
    GVX_MUTEX_LOCK(&g_prof_list_mutex);
    all_profs().push(this);
  }

  pthread_once(&g_start_once, &initialize_start_time);
}

rutz::prof::~prof() throw()
{
  if (g_pdata_print_at_exit)
    {
      pthread_once(&g_pdata_file_once, &open_pdata_file);

      GVX_MUTEX_LOCK(&g_pdata_mutex);

      if (g_pdata_file != 0)
        {
          print_prof_data(g_pdata_file);
        }
    }
}

void rutz::prof::reset() throw()
{
  m_call_count = 0;
  m_total_time.reset();
  m_children_time.reset();
}

unsigned int rutz::prof::count() const throw()
{
  return m_call_count;
}

void rutz::prof::add_time(const rutz::time& t) throw()
{
  m_total_time += t;
  ++m_call_count;
}

void rutz::prof::add_child_time(const rutz::time& t) throw()
{
  m_children_time += t;
}

const char* rutz::prof::context_name() const throw()
{
  return m_context_name;
}

const char* rutz::prof::src_file_name() const throw()
{
  return m_src_file_name;
}

int rutz::prof::src_line_no() const throw()
{
  return m_src_line_no;
}

double rutz::prof::total_time() const throw()
{
  return (m_call_count > 0)
    ? m_total_time.usec()
    : 0.0;
}

double rutz::prof::self_time() const throw()
{
  return (m_call_count > 0)
    ? m_total_time.usec() - m_children_time.usec()
    : 0.0;
}

double rutz::prof::avg_self_time() const throw()
{
  return m_call_count > 0 ? (total_time() / m_call_count) : 0.0;
}

void rutz::prof::print_prof_data(FILE* file) const throw()
{
  if (file == 0)
    GVX_ABORT("FILE* was null");

  const double total_elapsed_usec =
    (rutz::prof::get_now_time(s_timing_mode) - g_start).usec();

  // Don't try to convert the double values to long or int, because
  // we're likely to overflow and potentially cause a floating-point
  // exception.
  fprintf(file, "%10.0f %6u %10.0f %4.1f%% %10.0f %4.1f%% %s\n",
          avg_self_time(), count(),
          self_time(), (100.0 * self_time()) / total_elapsed_usec,
          total_time(), (100.0 * total_time()) / total_elapsed_usec,
          m_context_name);
}

void rutz::prof::print_prof_data(std::ostream& os) const throw()
{
  os.exceptions(std::ios::goodbit);

  const double total_elapsed_usec =
    (rutz::prof::get_now_time(s_timing_mode) - g_start).usec();

  // Don't try to convert the double values to long or int, because
  // we're likely to overflow and potentially cause a floating-point
  // exception.
  os << std::fixed
     << std::setw(10) << std::setprecision(0) << avg_self_time() << ' '
     << std::setw(6) << count() << ' '
     << std::setw(10) << std::setprecision(0) << self_time() << ' '
     << std::setw(4) << std::setprecision(1)
     << (100.0 * self_time()) / total_elapsed_usec << "% "
     << std::setw(10) << std::setprecision(0) << total_time() << ' '
     << std::setw(4) << std::setprecision(1)
     << (100.0 * total_time()) / total_elapsed_usec << "% "
     << m_context_name << '\n';
}

void rutz::prof::print_at_exit(bool yes_or_no) throw()
{
  g_pdata_print_at_exit = yes_or_no;
}

void rutz::prof::prof_summary_file_name(const char* fname)
{
  GVX_MUTEX_LOCK(&g_pdata_mutex);

  if (fname == 0)
    fname = "";

  g_pdata_fname = fname;
}

void rutz::prof::reset_all_prof_data() throw()
{
  GVX_MUTEX_LOCK(&g_prof_list_mutex);
  std::for_each(all_profs().begin(), all_profs().end(),
                std::mem_fun(&rutz::prof::reset));
}

namespace
{
  // comparison function for use with std::stable_sort()
  bool compare_total_time(rutz::prof* p1, rutz::prof* p2) throw()
  {
    return p1->total_time() < p2->total_time();
  }
}

void rutz::prof::print_all_prof_data(FILE* file) throw()
{
  GVX_MUTEX_LOCK(&g_prof_list_mutex);
  std::stable_sort(all_profs().begin(), all_profs().end(),
                   compare_total_time);

  for (unsigned int i = 0; i < all_profs().size(); ++i)
    {
      if (all_profs()[i]->count() > 0)
        all_profs()[i]->print_prof_data(file);
    }
}

void rutz::prof::print_all_prof_data(std::ostream& os) throw()
{
  GVX_MUTEX_LOCK(&g_prof_list_mutex);
  std::stable_sort(all_profs().begin(), all_profs().end(),
                   compare_total_time);

  for (unsigned int i = 0; i < all_profs().size(); ++i)
    {
      if (all_profs()[i]->count() > 0)
        all_profs()[i]->print_prof_data(os);
    }
}

static const char __attribute__((used)) vcid_groovx_rutz_prof_cc_utc20050630214713[] = "$Id: prof.cc 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/prof.cc $";
#endif // !GROOVX_RUTZ_PROF_CC_UTC20050630214713DEFINED
