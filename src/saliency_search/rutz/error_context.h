/** @file rutz/error_context.h */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2005-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Thu Nov 17 16:14:06 2005
// commit: $Id: error_context.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/error_context.h $
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

#ifndef GROOVX_RUTZ_ERROR_CONTEXT_H_UTC20051118001406_DEFINED
#define GROOVX_RUTZ_ERROR_CONTEXT_H_UTC20051118001406_DEFINED

#include "rutz/fstring.h"
#include "rutz/staticstack.h"

namespace rutz
{
  class error_context;
  class error_context_entry;
}

/// Don't use this class directly; use the GVX_ERR_CONTEXT() macro instead.
class rutz::error_context
{
public:
  /// Constructor (but use current() to get the current context)
  error_context();

  /// Destructor
  ~error_context();

  /// Get the current thread-local error context object
  static const rutz::error_context& current();

  /// Add an entry to the context stack
  bool add_entry(const error_context_entry* e);

  /// Remove an entry from the context stack
  void remove_entry(const error_context_entry* e);

  /// get the text of all context entries, separated by newlines
  rutz::fstring get_text() const;

  /// Prepend our message (if any) to the given string
  /** This function is a template that is designed to work with either
      std::string or rutz::fstring (but without naming std::string
      explicitly here, we can avoid needing to #include the 50K lines of
      code from <string>).

      Example usage:

      \code
      std::string mymsg = "something bad happened";
      rutz::error_context::current().prepend_to(mymsg);
      \endcode
  */
  template <class S>
  inline void prepend_to(S& str) const
  {
    const rutz::fstring ctx = this->get_text();

    if (ctx.length() > 0)
      {
        const S orig = str;
        str = "error context follows (innermost last):\n";
        str += ctx.c_str();
        str += "\n";
        str += orig;
      }
  }

private:
  error_context(const error_context&); // not implemented
  error_context& operator=(const error_context&); // not implemented

  rutz::static_stack<const error_context_entry*, 256> m_entries;
};

/// Don't use this class directly; use the GVX_ERR_CONTEXT() macro instead.
class rutz::error_context_entry
{
public:
  /// Construct an error context entry, adding it to the current error_context
  error_context_entry(const rutz::fstring& msg);

  /// Destruct the error context entry, removing it from its error_context
  ~error_context_entry();

  /// Get the message associated with this entry
  const rutz::fstring& text() const { return m_text; }

private:
  error_context_entry(const error_context_entry&);
  error_context_entry& operator=(const error_context_entry&);

  rutz::fstring m_text;

  // context from which to remove ourselves, or null
  rutz::error_context* m_context;
};

/// Give some contextual information that can be used if an error message is needed.
/** The GVX_ERR_CONTEXT() calls form a last-in/last-out stack that
    describes the call context at any moment, in particular at the
    moment that an error might occur. If an error does occur, we can
    capture the call context and report it to the user, (hopefully)
    making it easier to understand the error and debug the problem.
 */
#define GVX_ERR_CONTEXT(msg) \
  rutz::error_context_entry e_c_object_(msg)

static const char __attribute__((used)) vcid_groovx_rutz_error_context_h_utc20051118001406[] = "$Id: error_context.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/error_context.h $";
#endif // !GROOVX_RUTZ_ERROR_CONTEXT_H_UTC20051118001406DEFINED
