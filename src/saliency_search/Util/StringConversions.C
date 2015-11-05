/*!@file Util/StringConversions.C Convert to and from std::string */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
// by the University of Southern California (USC) and the iLab at USC.  //
// See http://iLab.usc.edu for information about this project.          //
// //////////////////////////////////////////////////////////////////// //
// Major portions of the iLab Neuromorphic Vision Toolkit are protected //
// under the U.S. patent ``Computation of Intrinsic Perceptual Saliency //
// in Visual Environments, and Applications'' by Christof Koch and      //
// Laurent Itti, California Institute of Technology, 2001 (patent       //
// pending; application number 09/912,225 filed July 23, 2001; see      //
// http://pair.uspto.gov/cgi-bin/final/home.pl for current status).     //
// //////////////////////////////////////////////////////////////////// //
// This file is part of the iLab Neuromorphic Vision C++ Toolkit.       //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is free software; you can   //
// redistribute it and/or modify it under the terms of the GNU General  //
// Public License as published by the Free Software Foundation; either  //
// version 2 of the License, or (at your option) any later version.     //
//                                                                      //
// The iLab Neuromorphic Vision C++ Toolkit is distributed in the hope  //
// that it will be useful, but WITHOUT ANY WARRANTY; without even the   //
// implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR      //
// PURPOSE.  See the GNU General Public License for more details.       //
//                                                                      //
// You should have received a copy of the GNU General Public License    //
// along with the iLab Neuromorphic Vision C++ Toolkit; if not, write   //
// to the Free Software Foundation, Inc., 59 Temple Place, Suite 330,   //
// Boston, MA 02111-1307 USA.                                           //
// //////////////////////////////////////////////////////////////////// //
//
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/StringConversions.C $
// $Id: StringConversions.C 11934 2009-11-06 03:03:46Z itti $
//

#include "Util/StringConversions.H"

#include "Util/StringConversions.H"
#include "Util/log.H"
#include "Util/sformat.H"
#include "rutz/demangle.h"
#include "rutz/error_context.h"

#include <iomanip>
#include <limits>
#include <netinet/in.h>          // for in_addr
#include <sstream>

namespace
{
  template <class T>
  std::string basictype_to_string(const T& val)
  {
    std::stringstream s;
    s << val;
    return s.str();
  }

  template <class T> inline
  void string_to_basictype(const std::string& str, T& val)
  {
    std::stringstream s;
    s << str;
    s >> val;
    if (s.fail())
      conversion_error::raise<T>(str);

    // now make sure there is no trailing junk in the string:
    s >> std::ws;
    std::string junk;
    s >> junk;
    if (!junk.empty())
      conversion_error::raise<T>
        (str, sformat("found extra character(s) '%s' "
                      "following a valid conversion",
                      junk.c_str()));
  }

  class real_conversion_error : public conversion_error
  {
  public:
    real_conversion_error(const std::type_info& type,
                          const std::string& str,
                          const std::string& extrainfo) throw()
      :
      m_msg(sformat("Bogus %s string: '%s'",
                    rutz::demangled_name(type), str.c_str()))
    {
      rutz::error_context::current().prepend_to(m_msg);

      if (extrainfo.length() > 0)
        m_msg += sformat(" (%s)", extrainfo.c_str());
    }

    virtual ~real_conversion_error() throw() {}

    virtual const char* what() const throw()
    {
      return m_msg.c_str();
    }

    std::string m_msg;
  };

}

// ######################################################################
// conversion_error
// ######################################################################

conversion_error::conversion_error() throw() {}

conversion_error::~conversion_error() throw() {}

void conversion_error::raise_impl(const std::type_info& type,
                                  const std::string& str,
                                  const std::string& extrainfo)
{
  throw real_conversion_error(type, str, extrainfo);
}

// ######################################################################
// converters for basic types
// ######################################################################

// int

std::string convertToString(const int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, int& val)
{ string_to_basictype(str, val); }

// long int

std::string convertToString(const long int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, long int& val)
{ string_to_basictype(str, val); }

// long long int

std::string convertToString(const long long int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, long long int& val)
{ string_to_basictype(str, val); }

// short int

std::string convertToString(const short int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, short int& val)
{ string_to_basictype(str, val); }

// unsigned int

std::string convertToString(const unsigned int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, unsigned int& val)
{ string_to_basictype(str, val); }

// unsigned long int

std::string convertToString(const unsigned long int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, unsigned long int& val)
{ string_to_basictype(str, val); }

// unsigned long long int

std::string convertToString(const unsigned long long int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, unsigned long long int& val)
{ string_to_basictype(str, val); }

// unsigned short int

std::string convertToString(const unsigned short int& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, unsigned short int& val)
{ string_to_basictype(str, val); }

// unsigned char

std::string convertToString(const unsigned char& val)
{
  const int intval = val;
  return basictype_to_string(intval);
}

void convertFromString(const std::string& str, unsigned char& val)
{
  int intval;
  string_to_basictype(str, intval);

  if (intval < 0)
    conversion_error::raise<unsigned char>
      (str, "value too small for range of type");
  else if ((unsigned int) intval > std::numeric_limits<unsigned char>::max())
    conversion_error::raise<unsigned char>
      (str, "value too large for range of type");

  val = (unsigned char) intval;
}

// float

std::string convertToString(const float& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, float& val)
{ string_to_basictype(str, val); }

// double

std::string convertToString(const double& val)
{ return basictype_to_string(val); }

void convertFromString(const std::string& str, double& val)
{ string_to_basictype(str, val); }


// ######################################################################
// no-op converters for std::string
// ######################################################################

std::string convertToString(const std::string& val)
{ return val; }

void convertFromString(const std::string& str, std::string& val)
{ val = str; }


// ######################################################################
// converters for bool
// ######################################################################

std::string convertToString(const bool& val)
{ return val ? "true" : "false"; }

void convertFromString(const std::string& str, bool& val)
{
  char c = str[0];
  if (c == '1' || c == 't' || c == 'T' || c == 'y' || c == 'Y')
    { val = true; }
  else if (c == '0' || c == 'f' || c == 'F' || c == 'n' || c == 'N')
    { val = false; }
  else
    conversion_error::raise<bool>(str);
}


// ######################################################################
// converters for in_addr
// ######################################################################

std::string convertToString(const in_addr& val)
{
  std::stringstream s;
  unsigned long int a = val.s_addr;
  s<<(a>>24)<<'.'<<((a>>16) & 0xFF)<<'.'<<((a>>8) & 0xFF)<<'.'<<(a & 0xFF);
  return s.str();
}

void convertFromString(const std::string& str, in_addr& val)
{
  std::stringstream s; unsigned long int n1 = 300, n2 = 300, n3 = 300, n4=300;
  char c; s<<str; s>>n1>>c>>n2>>c>>n3>>c>>n4;
  if (n1 == 300 || n2 == 300 || n3 == 300 || n4 == 300)
    conversion_error::raise<in_addr>(str);
  val.s_addr = (n1 << 24) | (n2 << 16) | (n3 << 8) | n4;
}

// ######################################################################
// converters for rutz::time
// ######################################################################

std::string convertToString(const rutz::time& val)
{
  const double s = val.sec();

  const int hpart = int(s/(60.0*60.0));
  const int mpart = int((s-hpart*60*60)/60.0);
  const double spart = s - hpart*60*60 - mpart*60;

  return sformat("%02d:%02d:%06.3f", hpart, mpart, spart);
}

void convertFromString(const std::string& str, rutz::time& val)
{
  std::stringstream ss; unsigned long int h = 100, m = 100; float s = 100.0F;
  char c; ss<<str; ss>>h>>c>>m>>c>>s;
  if (h == 100 || m == 100 || s == 100.0F) conversion_error::raise<rutz::time>(str);
  val.reset(h*3600 + m*60 + (unsigned long)s, (unsigned long)((s - int(s)) * 1.0e6F));
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
