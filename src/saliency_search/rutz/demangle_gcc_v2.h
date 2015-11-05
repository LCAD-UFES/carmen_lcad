/** @file rutz/demangle_gcc_v2.h hand-rolled demangling of
    std::type_info::name() from the g++ 2.x series */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2002-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Tue Jun 19 16:58:19 2001 (as gcc_v2_demangle.h)
// commit: $Id: demangle_gcc_v2.h 8249 2007-04-12 06:03:40Z rjpeters $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle_gcc_v2.h $
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

#ifndef GROOVX_RUTZ_DEMANGLE_GCC_V2_H_UTC20050626084020_DEFINED
#define GROOVX_RUTZ_DEMANGLE_GCC_V2_H_UTC20050626084020_DEFINED

#include <cctype>
#include <cstdlib>
#include <string>

#include "rutz/trace.h"
#include "rutz/debug.h"
GVX_DBG_REGISTER

namespace
{
  int munch_int(std::string& str, std::string::size_type pos)
  {
    int val = 0;
    while ( isdigit(str[pos]) )
      {
        val *= 10;
        val += (str[pos] - '0');
        str.erase(pos, 1);
      }
    return val;
  }

  void put_int(int val, std::string& str, std::string::size_type pos)
  {
    while (val != 0)
      {
        str.insert(pos, 1, '0' + (val%10));
        val /= 10;
      }
  }

  std::string read_modifiers(std::string& str, std::string::size_type& pos)
  {
    std::string modifiers("");
    std::string array_dims("");

    while ( isupper(str[pos]) )
      {
        switch (str[pos])
          {
          case 'A':
            str.erase(pos, 1); // we need to also erase the '_'
            array_dims.append("[");
            if ( isdigit(str[pos]) )
              {
                int dim_size = munch_int(str, pos);
                put_int(dim_size+1,array_dims,array_dims.length());
              }
            array_dims.append("]");
            break;
          case 'C':
            modifiers.insert(0, " const");
            break;
          case 'P':
            modifiers.insert(0, "*");
            break;
          case 'R':
            modifiers.insert(0, "&");
            break;
          case 'U':
            modifiers.insert(0, " unsigned");
            break;
          default:
            GVX_ASSERT(0);
          }
        str.erase(pos, 1);
      }
    dbg_eval_nl(3, str.c_str());

    modifiers.append(array_dims);

    return modifiers;
  }

  void insert_builtin_typename(std::string& str, std::string::size_type& pos)
  {
    char typecode = str[pos];
    str.erase(pos, 1);
    switch (typecode)
      {
      case 'b':
        str.insert(pos, "bool"); pos += 4;
        break;
      case 'c':
        str.insert(pos, "char"); pos += 4;
        break;
      case 'd':
        str.insert(pos, "double"); pos += 6;
        break;
      case 'f':
        str.insert(pos, "float"); pos += 5;
        break;
      case 'i':
        str.insert(pos, "int"); pos += 3;
        break;
      case 'l':
        str.insert(pos, "long"); pos += 4;
        break;
      case 's':
        str.insert(pos, "short"); pos += 5;
        break;
      case 'v':
        str.insert(pos, "void"); pos += 4;
        break;
      default:
        dbg_eval_nl(3, typecode);
        GVX_ASSERT(0);
        break;
      }
  }

  std::string demangle_gcc_v2(const std::string& in)
  {
    GVX_TRACE("demangle_impl");
    std::string out=in;

    dbg_eval_nl(3, out.c_str());

    bool isTemplate = false;

    if (out.length() == 0) return out;

    std::string::size_type pos = 0;

    // Check if we have namespace qualifiers...
    if (out[0] == 'Q')
      {
        out.erase(0, 1);
        int num_levels = 0;
        // get the number of nesting levels...
        if (out[0] == '_')
          {
            out.erase(0, 1);
            num_levels = munch_int(out, 0);
            dbg_eval_nl(3, num_levels);
            GVX_ASSERT(out[0] == '_');
            out.erase(0, 1);
          }
        else
          {
            num_levels = out[0] - '0';
            out.erase(0, 1);
          }
        while (num_levels > 1)
          {
            // Get the length of the current qualifier
            int length = munch_int(out, pos);
            // Skip over the qualifier itself
            pos += length;
            // Insert a scope resolution operator after the qualifier "::"
            out.insert(pos, "::");
            // Skip over the just-inserted "::"
            pos += 2;
            --num_levels;
          }
      }

    dbg_eval_nl(3, out.c_str());

    if (out[pos] == 't')
      {
        isTemplate = true;
        out.erase(pos, 1);
      }

    if ( !isTemplate )
      {
        while ( isdigit(out[pos]) )
          {
            out.erase(pos, 1);
          }
        dbg_eval_nl(3, out.c_str());
      }
    else
      {
        // Read length of template base name and skip over it
        int base_length = munch_int(out, pos);
        pos += base_length;

        // Insert left bracket
        out.insert(pos++, 1, '<');

        dbg_eval_nl(3, out.c_str());

        int num_parameters = munch_int(out, pos);;

        for (int n = 0; n < num_parameters; ++n)
          {
            // Template parameters must start with 'Z'
            GVX_ASSERT( out[pos] == 'Z' );
            out.erase(pos, 1);

            dbg_eval_nl(3, out.c_str());

            // Get the parameter name:
            std::string modifiers = read_modifiers(out, pos);

            if ( !isdigit(out[pos]) )
              {
                insert_builtin_typename(out, pos);
              }
            else
              {
                // Read length of template parameter name and skip over it
                int param_length = munch_int(out, pos);
                pos += param_length;
              }

            out.insert(pos, modifiers);
            pos += modifiers.length();

            // Insert a comma if this is not the last parameter
            if (n < (num_parameters-1))
              {
                out.insert(pos++, 1, ',');
                out.insert(pos++, 1, ' ');
              }

            dbg_eval_nl(3, out.c_str());
          }

        // Insert right bracket
        out.insert(pos++, 1, '>');
      }

    dbg_eval_nl(3, out.c_str());

    return out;
  }
}

static const char __attribute__((used)) vcid_groovx_rutz_demangle_gcc_v2_h_utc20050626084020[] = "$Id: demangle_gcc_v2.h 8249 2007-04-12 06:03:40Z rjpeters $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/demangle_gcc_v2.h $";
#endif // !GROOVX_RUTZ_DEMANGLE_GCC_V2_H_UTC20050626084020_DEFINED
