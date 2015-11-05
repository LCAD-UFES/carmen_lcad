/** @file rutz/prof.h class for accumulating profiling information */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at klab dot caltech dot edu>
//
// created: Thu Jun 30 14:47:11 2005
// commit: $Id: prof.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/prof.h $
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

#ifndef GROOVX_RUTZ_PROF_H_UTC20050630214711_DEFINED
#define GROOVX_RUTZ_PROF_H_UTC20050630214711_DEFINED

#include "rutz/time.h"

#include <iosfwd>

namespace rutz
{
  class prof;
}

/// Accumulates profiling information for a given execution context.
/** rutz::prof, along with rutz::trace and associated macros
    (e.g. GVX_TRACE()) form a basic profiling facility.

    * GVX_TRACE() statements collect timing information at function
      entry and exit (or, more precisely, at construction and
      destruction of the local object that is introduced by the
      GVX_TRACE() macro call). The overhead for this profiling
      information is non-zero, but is completely neglible in
      proportion to the amount of execution time taken by any of the
      functions in FilterOps.

    * To turn on profiling in a given source file, just #include
      "rutz/trace.h" and insert a GVX_TRACE() wherever you like. To
      force profiling to be off, #define GVX_NO_PROF before you
      #include "rutz/trace.h".

    * In order to actually see the profiling information, you can
      either call rutz::prof::print_all_prof_data() directly, or you
      can call rutz::prof::print_at_exit(true). If you do the latter,
      then when the program exits, the profiling information will be
      dumped to a file named 'prof.out' in the current
      directory. Currently, the only program that generates a prof.out
      in this way is bin/invt (the tcl script interpreter).

    * Profiling information in prof.out looks like the following,
      where the first column is the average total usec per call, the
      second column is the number of calls, the third column is the
      total self usec (i.e. excluding time attributed to nested
      GVX_TRACE statements), the fourth column is the total
      self+children usec, and the remainder of the line is the
      function name or whatever string was passed to GVX_TRACE():

         4   4500      18999      18999 xFilterClean
         5   4500      23998      23998 yFilterClean
       305  11600      34998    3548467 lowPassX
         8   4900      42997      42997 lowPass3y
       297  11600      44994    3454495 lowPassY
        15   4900      77989      77989 lowPass3x
       508  14700    7458831    7477830 lowPass9x
       582  14700    8534732    8558730 lowPass9y
      8749   4000   25762090   34996666 orientedFilter
 */
class rutz::prof
{
public:
  prof(const char* s, const char* fname, int lineno) throw();
  ~prof() throw();

  /// Different types of timing.
  enum timing_mode
    {
      WALLCLOCK, ///< Track elapsed wall-clock time.
      RUSAGE     ///< Track elpased user+sys rusage.
    };

  /// Get the current timing_mode.
  static timing_mode get_timing_mode() throw()
  { return s_timing_mode; }

  /// Set the current timing_mode.
  static void set_timing_mode(timing_mode mode) throw()
  { s_timing_mode = mode; }

  /// Get the current time according to the given timing_mode.
  static rutz::time get_now_time(timing_mode mode) throw()
  {
    switch (mode)
      {
      case rutz::prof::RUSAGE:
        return (rutz::time::user_rusage() + rutz::time::sys_rusage());

      case rutz::prof::WALLCLOCK:
      default:
        return rutz::time::wall_clock_now();
      }
  }

  /// Reset the call count and elapsed time to zero.
  void reset() throw();

  /// Returns the number of calls since the last reset().
  unsigned int count() const throw();

  void add_time(const rutz::time& t) throw();

  void add_child_time(const rutz::time& t) throw();

  const char* context_name() const throw();

  const char* src_file_name() const throw();

  int src_line_no() const throw();

  /// Get the total elapsed time in microsecs since the last reset().
  double total_time() const throw();

  /// Get the total self time in microsecs since the last reset().
  double self_time() const throw();

  /// Get the per-call average self time in microsecs since the last reset().
  double avg_self_time() const throw();

  /// Print this object's info to the given file.
  void print_prof_data(FILE* f) const throw();

  /// Print this object's info to the given stream.
  void print_prof_data(std::ostream& os) const throw();

  /// Whether to write a profiling summary file when the program exits.
  static void print_at_exit(bool yes_or_no) throw();

  /// Specify the filename for the profiling summary (default is "prof.out").
  static void prof_summary_file_name(const char* fname);

  /// Reset all call counts and elapsed times to zero.
  static void reset_all_prof_data() throw();

  /// Print all profile data to the given file.
  static void print_all_prof_data(FILE* f) throw();

  /// Print all profile data to the given stream.
  static void print_all_prof_data(std::ostream& os) throw();

private:
  prof(const prof&) throw();
  prof& operator=(const prof&) throw();

  const char*  const m_context_name;
  const char*  const m_src_file_name;
  int          const m_src_line_no;
  unsigned int       m_call_count;
  rutz::time         m_total_time;
  rutz::time         m_children_time;

  static timing_mode s_timing_mode;
};

static const char __attribute__((used)) vcid_groovx_rutz_prof_h_utc20050630214711[] = "$Id: prof.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/prof.h $";
#endif // !GROOVX_RUTZ_PROF_H_UTC20050630214711DEFINED
