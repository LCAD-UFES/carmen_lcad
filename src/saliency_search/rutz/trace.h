/** @file rutz/trace.h GVX_TRACE macro for user-controlled tracing and
    profiling */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Mon Jan  4 08:00:00 1999
// commit: $Id: trace.h 9375 2008-03-04 17:04:59Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/trace.h $
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

#ifndef GROOVX_RUTZ_TRACE_H_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_TRACE_H_UTC20050626084019_DEFINED

// The basic idea is that for each function for which profiling is
// enabled, a static rutz::prof object is created. This object
// maintains the call count and total elapsed time for that
// function. The job of measuring and recording such information falls
// to the rutz::trace class. A local object of the rutz::trace class
// is constructed on entry to a function, and it is destructed just
// prior to function exit. If a GVX_TRACE_EXPR macro is defined, and
// that macro evaluates to true at the time the GVX_TRACE statement is
// reached, then the rutz::trace object will emit "entering" and
// "leaving" messages as it is constructed and destructed,
// respectively. (Thus, to simply turn on verbose tracing in a source
// file, just do "#define GVX_TRACE_EXPR true".) In any case, the
// rutz::trace object takes care of teling the static rutz::prof
// object to 1) increment its counter, and 2) record the elapsed time.
//
// The behavior of the control macros are as follows:
//
// 1) if GVX_NO_PROF is defined, no profiling/tracing will occur;
//    OTHERWISE: profiling always occurs, AND
// 2) if GVX_TRACE_EXPR is defined, that expression is used to control
//    verbose tracing, otherwise verbose tracing will be off

#include "rutz/prof.h"
#include "rutz/time.h"

#include <iosfwd>

namespace rutz
{
  class trace;
}

/// Times and traces execution in and out of a lexical scope.
/** This class cooperates with rutz::prof. rutz::prof objects are
    expected to have "long" lifetimes, and accumulate timing
    information over multiple passes. In each pass, a rutz::trace
    object should be constructed, which will pass its runtime info
    onto the rutz::prof, which then accumulates the timing info. */
class rutz::trace
{
public:
  /// Query whether we are unconditionally printing trace in/out messages.
  static bool get_global_trace() throw();
  /// Set whether to unconditionally print trace in/out messages.
  static void set_global_trace(bool on_off) throw();

  /// Get the max nesting level for printing trace in/out messages.
  static unsigned int get_max_level() throw();
  /// Set the max nesting level for printing trace in/out messages.
  static void         set_max_level(unsigned int lev) throw();

  /// Construct a rutz::trace object.
  /** Store the current time internally (either the wall clock time or
      current user+sys rusage, depending on the current
      timing_mode). If use_msg, then print a trace-begin message to
      stderr showing the name of the given rutz::prof. */
  trace(rutz::prof& p, bool use_msg) throw();

  /// Destruct the rutz::trace object, accumulating time information in the stored rutz::prof.
  /** Get the new time (either wall clock or user+sys rusage, matching
      whatever we did in the rutz::trace constructor. If we printed a
      trace-begin message, then also print a matching trace-end
      message. */
  ~trace() throw();

private:
  rutz::prof&  m_prof;
  rutz::time   m_start;
  const bool   m_should_print_msg;
  const bool   m_should_pop;
  rutz::prof::timing_mode  m_timing_mode; ///< Store this in case somebody changes the timing mode before we finish
};

#ifndef GVX_TRACE_EXPR
#  define GVX_TRACE_EXPR false
#endif

#define GVX_TRACE_CONCAT2(x,y) x##y
#define GVX_TRACE_CONCAT(x,y) GVX_TRACE_CONCAT2(x,y)

#ifndef GVX_NO_PROF
#  define GVX_TRACE(x) \
         static rutz::prof  GVX_TRACE_CONCAT(P_x_, __LINE__)  (x,   __FILE__, __LINE__); \
         rutz::trace        GVX_TRACE_CONCAT(T_x_, __LINE__)  (GVX_TRACE_CONCAT(P_x_, __LINE__), GVX_TRACE_EXPR)
#else
#  define GVX_TRACE(x) do {} while(0)
#endif

static const char __attribute__((used)) vcid_groovx_rutz_trace_h_utc20050626084019[] = "$Id: trace.h 9375 2008-03-04 17:04:59Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/trace.h $";
#endif // !GROOVX_RUTZ_TRACE_H_UTC20050626084019_DEFINED
