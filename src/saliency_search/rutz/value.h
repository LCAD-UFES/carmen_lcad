/** @file rutz/value.h abstract interface for string-convertible types */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Tue Sep 28 11:19:17 1999
// commit: $Id: value.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/value.h $
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

#ifndef GROOVX_RUTZ_VALUE_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_VALUE_H_UTC20050626084020_DEFINED

#include <iosfwd>

namespace rutz
{
  class fstring;
  class value;
}

///////////////////////////////////////////////////////////////////////
/**
 *
 * \c rutz::value defines an interface for variant-type value objects
 * that can be converted to+from a string representation. Subclasses
 * may allow conversion among types so that a value of any basic type
 * can be retrieved, or they may allow only one basic type to be
 * retrieved.
 *
 **/
///////////////////////////////////////////////////////////////////////

class rutz::value
{
public:

  /// Default constructor.
  value();

  /// Virtual destructor ensures proper destruction of base classes.
  virtual ~value();

  /// Return a string giving the name of the native type.
  virtual rutz::fstring value_typename() const = 0;

  /// Write the value to a \c std::ostream.
  virtual void print_to(std::ostream& os) const = 0;

  /// Read the value from a \c std::istream.
  virtual void scan_from(std::istream& is) = 0;

  /// Get an \c rutz::fstring representation of the value.
  /** Builds a string stream, calls print_to(), and returns its
      value. */
  rutz::fstring get_string() const;

  /// Set the value from an rutz::fstring.
  /** Builds a string stream from \a val and calling scan_from() with
      it. */
  void set_string(rutz::fstring val);
};

///////////////////////////////////////////////////////////////////////
//
// Global stream insertion/extraction operators for rutz::value
//
///////////////////////////////////////////////////////////////////////

inline std::ostream& operator<<(std::ostream& os,
                                const rutz::value& val)
{
  val.print_to(os);
  return os;
}

inline std::istream& operator>>(std::istream& is,
                                rutz::value& val)
{
  val.scan_from(is);
  return is;
}

static const char __attribute__((used)) vcid_groovx_rutz_value_h_utc20050626084020[] = "$Id: value.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/value.h $";
#endif // !GROOVX_RUTZ_VALUE_H_UTC20050626084020_DEFINED
