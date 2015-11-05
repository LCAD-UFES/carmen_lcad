/** @file rutz/debug.h debugging facilities, assertions,
    preconditions, postconditions, invariants */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Mon Jan  4 08:00:00 1999
// commit: $Id: debug.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/debug.h $
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

#ifndef GROOVX_RUTZ_DEBUG_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_DEBUG_H_UTC20050626084020_DEFINED

namespace rutz {namespace debug
{
  void eval (const char* what, int level, const char* where, int line_no, bool nl, bool expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, char expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, unsigned char expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, short expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, unsigned short expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, int expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, unsigned int expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, long expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, unsigned long expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, float expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, double expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, const char* expr) throw();
  void eval (const char* what, int level, const char* where, int line_no, bool nl, void* expr) throw();

  void dump (const char* what, int level, const char* where, int line_no) throw();

  void start_newline () throw();

  void panic_aux         (const char* what, const char* where, int line_no) throw();
  void assert_aux        (const char* what, const char* where, int line_no) throw();
  void precondition_aux  (const char* what, const char* where, int line_no) throw();
  void postcondition_aux (const char* what, const char* where, int line_no) throw();
  void invariant_aux     (const char* what, const char* where, int line_no) throw();

  /// Allocate a debug key for the given filename
  int create_key(const char* filename);

  /// Query whether the given value is a valid debug key
  bool is_valid_key(int key);

  /// Get the debug key associated with the given filename.
  /** Returns -1 if the filename is not registered. */
  int lookup_key(const char* filename);

  /// Get the current debug level associated with the given debug key
  int get_level_for_key(int key);

  /// Set the current debug level for the given debug key
  void set_level_for_key(int key, int level);

  /// Get the filename associated with the given debug key
  const char* get_filename_for_key(int key);

  void set_global_level(int lev);
}}

/// Print a message, followed by a rutz::backtrace (if available), then abort().
#define GVX_PANIC(message) rutz::debug::panic_aux(message, __FILE__, __LINE__)

/// Abort if the given expression evaluates to true.
/** This is like GVX_ASSERT(), except (1) it has the opposite sense
    (GVX_ASSERT() says some expression must be TRUE, GVX_ABORT_IF()
    says some expression must be false), and (2) it can't ever be
    turned off (whereas GVX_ASSERT() is under the control of the
    GVX_NO_DEBUG macro). */
#define GVX_ABORT_IF(expr) \
      do { if ( expr ) { rutz::debug::panic_aux(#expr, __FILE__, __LINE__); } } while (0)

#define GVX_CONCAT(x, y) x ## y

#define GVX_DO_DBG_REGISTER(ext)                                     \
static const int GVX_CONCAT(DEBUG_KEY, ext) =                        \
  rutz::debug::create_key(__FILE__);                                 \
                                                                     \
static inline int GVX_CONCAT(dbg_level_, ext) ()                     \
{                                                                    \
  return rutz::debug::get_level_for_key(GVX_CONCAT(DEBUG_KEY, ext)); \
}

#define GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(1)
#define GVX_DBG_LEVEL dbg_level_1

static const char __attribute__((used)) vcid_groovx_rutz_debug_h_utc20050626084020[] = "$Id: debug.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/debug.h $";

#else // DEBUG_H_DEFINED

//
// Everything here gets processed on the second and subsequent times that
// this file is #include'ed.
//

#undef GVX_DBG_REGISTER
#undef GVX_DBG_LEVEL

#if   !defined(GVX_DEBUG_H_2)
#      define  GVX_DEBUG_H_2
#      define  GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(2)
#      define  GVX_DBG_LEVEL dbg_level_2
#elif !defined(GVX_DEBUG_H_3)
#      define  GVX_DEBUG_H_3
#      define  GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(3)
#      define  GVX_DBG_LEVEL dbg_level_3
#elif !defined(GVX_DEBUG_H_4)
#      define  GVX_DEBUG_H_4
#      define  GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(4)
#      define  GVX_DBG_LEVEL dbg_level_4
#elif !defined(GVX_DEBUG_H_5)
#      define  GVX_DEBUG_H_5
#      define  GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(5)
#      define  GVX_DBG_LEVEL dbg_level_5
#elif !defined(GVX_DEBUG_H_6)
#      define  GVX_DEBUG_H_6
#      define  GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(6)
#      define  GVX_DBG_LEVEL dbg_level_6
#elif !defined(GVX_DEBUG_H_7)
#      define  GVX_DEBUG_H_7
#      define  GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(7)
#      define  GVX_DBG_LEVEL dbg_level_7
#elif !defined(GVX_DEBUG_H_8)
#      define  GVX_DEBUG_H_8
#      define  GVX_DBG_REGISTER GVX_DO_DBG_REGISTER(8)
#      define  GVX_DBG_LEVEL dbg_level_8
#else
#      error "debug.h included too many times!"
#endif

#undef dbg_eval
#undef dbg_eval_nl
#undef dbg_print
#undef dbg_print_nl

#undef GVX_ASSERT
#undef GVX_INVARIANT
#undef GVX_PRECONDITION
#undef GVX_POSTCONDITION

#endif // !GROOVX_RUTZ_DEBUG_H_UTC20050626084020_DEFINED

//
// Everything here gets re-processed every time that this file is #include'ed.
//

#if !defined(GVX_NO_DEBUG)
#  define dbg_eval(lev, x)     do { if (GVX_DBG_LEVEL() >= lev) rutz::debug::eval(#x, lev, __FILE__, __LINE__, false, x); } while (0)
#  define dbg_eval_nl(lev, x)  do { if (GVX_DBG_LEVEL() >= lev) rutz::debug::eval(#x, lev, __FILE__, __LINE__, true, x); } while (0)
#  define dbg_print(lev, x)    do { if (GVX_DBG_LEVEL() >= lev) rutz::debug::eval(0, lev, __FILE__, __LINE__, false, x); } while (0)
#  define dbg_print_nl(lev, x) do { if (GVX_DBG_LEVEL() >= lev) rutz::debug::eval(0, lev, __FILE__, __LINE__, true, x); } while (0)
#  define dbg_dump(lev, x)     do { if (GVX_DBG_LEVEL() >= lev) { rutz::debug::dump(#x, lev, __FILE__, __LINE__); (x).debug_dump(); } } while (0)
#  define dbg_nl(lev)          do { if (GVX_DBG_LEVEL() >= lev) rutz::debug::start_newline(); } while (0)

#  define GVX_ASSERT(expr)        do { if ( !(expr) ) rutz::debug::assert_aux(#expr, __FILE__, __LINE__); } while(0)
#  define GVX_INVARIANT(expr)     do { if ( !(expr) ) rutz::debug::invariant_aux(#expr, __FILE__, __LINE__); } while(0)
#  define GVX_PRECONDITION(expr)  do { if ( !(expr) ) rutz::debug::precondition_aux(#expr, __FILE__, __LINE__); } while(0)
#  define GVX_POSTCONDITION(expr) do { if ( !(expr) ) rutz::debug::postcondition_aux(#expr, __FILE__, __LINE__); } while(0)

#else // defined(GVX_NO_DEBUG)

#  define dbg_eval(lev, x)     do {} while (0)
#  define dbg_eval_nl(lev, x)  do {} while (0)
#  define dbg_print(lev, x)    do {} while (0)
#  define dbg_print_nl(lev, x) do {} while (0)
#  define dbg_dump(lev, expr)  do {} while (0)
#  define dbg_nl(lev)          do {} while (0)

#  define GVX_ASSERT(x)        do {} while (0)
#  define GVX_INVARIANT(x)     do {} while (0)
#  define GVX_PRECONDITION(x)  do {} while (0)
#  define GVX_POSTCONDITION(x) do {} while (0)

#endif
