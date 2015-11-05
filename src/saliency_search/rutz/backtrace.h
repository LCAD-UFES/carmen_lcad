/** @file rutz/backtrace.h represents the state of the call stack as
    given by GVX_TRACE statements */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Oct 13 16:32:38 2004
// commit: $Id: backtrace.h 15310 2012-06-01 02:29:24Z itti $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/backtrace.h $
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

#ifndef GROOVX_RUTZ_BACKTRACE_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_BACKTRACE_H_UTC20050626084020_DEFINED

#include "rutz/staticstack.h"

#include <iosfwd>

namespace rutz
{
  class backtrace;
  class prof;
}

/// Represents an instantaneous state of the call stack.
class rutz::backtrace
{
public:
  /// Default construct an empty call stack.
  backtrace() throw();

  /// Copy constructor.
  backtrace(const rutz::backtrace& other) throw();

  /// Assignment operator.
  backtrace& operator=(const rutz::backtrace& other) throw();

  /// Destructor.
  ~backtrace() throw();

  /// Access the current call stack.
  static rutz::backtrace& current() throw();

  /// Push a new element onto the call stack. Returns true if successful.
  bool push(rutz::prof* p) throw();

  /// Pop the most recent element off of the call stack.
  void pop() throw();

  /// Get the number of elements in the call stack.
  unsigned int size() const throw();

  /// Get the top stack frame, or null if the backtrace is empty.
  rutz::prof* top() const throw();

  /// Will return a null pointer if i is out of range.
  rutz::prof* at(unsigned int i) const throw();

  /// Shorthand for at(i).
  rutz::prof* operator[](unsigned int i) const throw() { return at(i); }

  /// Print the call stack on stderr.
  void print() const throw();

  /// Print the call stack to the given stream.
  void print(std::ostream& os) const throw();

private:
  rutz::static_stack<rutz::prof*, 256> m_vec;
};

static const char __attribute__((used)) vcid_groovx_rutz_backtrace_h_utc20050626084020[] = "$Id: backtrace.h 15310 2012-06-01 02:29:24Z itti $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/backtrace.h $";
#endif // !GROOVX_RUTZ_BACKTRACE_H_UTC20050626084020_DEFINED
