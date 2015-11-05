/** @file rutz/demangle_gcc_v3.h demangle std::type_info::name()
    using the standardized cxxabi from g++ version >= 3 */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Tue Jun 19 17:00:38 2001 (as gcc_v3_demangle.h)
// commit: $Id: demangle_gcc_v3.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle_gcc_v3.h $
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

#ifndef GROOVX_RUTZ_DEMANGLE_GCC_V3_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_DEMANGLE_GCC_V3_H_UTC20050626084020_DEFINED

#include "rutz/error.h"

#include "rutz/demangle_cxxfilt.h"
#include "rutz/sfmt.h"

#include <cstdlib>
#include <cxxabi.h>
#include <string>

#include "rutz/debug.h"
#include "rutz/trace.h"

namespace
{
  const char* get_err_reason(int status)
  {
    switch (status)
      {
      case -1:
        return "memory allocation error";
        break;

      case -2:
        return "invalid mangled name";
        break;

      case -3:
        return "invalid arguments (e.g. buf non-NULL "
                   "and length NULL)";
        break;
      }

    // default:
    return "unknown error code";
  }

  std::string demangle_gcc_v3(const std::string& mangled)
  {
    GVX_TRACE("demangle_gcc_v3");

    int status = 0;

    static std::size_t length = 256;
    static char* demangled = static_cast<char*>(malloc(length));

    demangled = abi::__cxa_demangle(mangled.c_str(), demangled,
                                    &length, &status);

    if (status == 0)
      {
        GVX_ASSERT(demangled != 0);
        return std::string(demangled);
      }

    // ok, cxa_demangle failed, but before we report that error let's
    // try falling back to c++filt (but if any exception occurs there,
    // we'll ignore it and just report the original cxa_demangle error
    // instead):
    try
      {
        return demangle_cxxfilt(mangled);
      }
    catch (std::exception& e)
      {
        const rutz::fstring msg =
          rutz::sfmt("during cxa_demangle of '%s': %s\n"
                     "(c++filt also failed: %s)",
                     mangled.c_str(), get_err_reason(status),
                     e.what());

        throw rutz::error(msg, SRC_POS);
      }

    GVX_ASSERT(false);
    return "can't happen"; // can't happen, but placate compiler
  }
}

//
// demangling test code
//

//  #include <iostream>
//  #include <exception>
//  #include <typeinfo>

//  #include "src/util/demangle_gcc_v3.h"

//  void printboth(const char* n)
//  {
//    std::cout << n << '\t' << gcc_v3_demangle(n) << std::endl;
//  }

//  class Global { int x; };

//  namespace Nspace { struct Nested { struct Inner {int x; }; }; }

//  template <typename T>
//  class Tplate { T x; };

//  class i {};

//  template <class T1, class T2>
//  class Tplate2 { T1 x1; T2 x2; };

//  template <class T1>
//  class Tplate3 { struct Xt {}; };

//  int main()
//  {
//    printboth(typeid(int).name());
//    printboth(typeid(double).name());
//    printboth(typeid(i).name());
//    printboth(typeid(Global).name());
//    printboth(typeid(std::exception).name());
//    printboth(typeid(Nspace::Nested).name());
//    printboth(typeid(Nspace::Nested::Inner).name());
//    printboth(typeid(Tplate<int>).name());
//    printboth(typeid(Tplate<double>).name());
//    printboth(typeid(Tplate<Global>).name());
//    printboth(typeid(Tplate<std::exception>).name());
//    printboth(typeid(Tplate<Nspace::Nested::Inner>).name());
//    printboth(typeid(Tplate2<int, double>).name());
//    printboth(typeid(Tplate<int*>).name());
//    printboth(typeid(Tplate<Global*>).name());
//    printboth(typeid(Tplate<const int*>).name());
//    printboth(typeid(Tplate<const Global*>).name());
//    printboth(typeid(Tplate<int* const>).name());
//    printboth(typeid(Tplate<Global* const>).name());
//    printboth(typeid(Tplate<int const* const>).name());
//    printboth(typeid(Tplate<Global const* const>).name());
//    printboth(typeid(Tplate3<int>::Xt).name());
//  }

static const char __attribute__((used)) vcid_groovx_rutz_demangle_gcc_v3_h_utc20050626084020[] = "$Id: demangle_gcc_v3.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle_gcc_v3.h $";
#endif // !GROOVX_RUTZ_DEMANGLE_GCC_V3_H_UTC20050626084020_DEFINED
