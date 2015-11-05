/*!@file Component/ParamMap.C I/O operations for parameter files */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// Primary maintainer for this file: Rob Peters <rjpeters@klab.caltech.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Component/ParamMap.C $
// $Id: ParamMap.C 12962 2010-03-06 02:13:53Z irock $
//

#ifndef PARAMMAP_C_DEFINED
#define PARAMMAP_C_DEFINED

#include "Component/ParamMap.H"

#include "Util/Assert.H"
#include "Util/Types.H"
#include "Util/log.H"

#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

// ######################################################################
namespace dummy_namespace_to_avoid_gcc411_bug_ParamMap_C
{
  void escapeWrite(std::ostream& ostrm, const std::string& s)
  {
    for (uint i = 0; i < s.size(); ++i)
      {
        switch (s[i])
          {
          // Need to escape whitespace in order to allow interior
          // whitespace within attribute values:
          case ' '   : ostrm << '\\' << ' '; break;
          case '\t'  : ostrm << '\\' << '\t'; break;
          case '\n'  : ostrm << '\\' << '\n'; break;

          // Must escape a backslash itself in order to not confuse it with
          // "backslash-as-escape-character"
          case '\\'  : ostrm << '\\' << '\\'; break;

          // Anything else can be written literally
          default    : ostrm << s[i]; break;
          }
      }
  }

  std::string escapeRead(std::istream& istrm)
  {
    // Read into a std::vector<char> rather than a std::string since it has
    // amortized-constant push_back(), which is not necessarily the case
    // for std::string.
    std::vector<char> buf;

    bool finished = false;

    // Skip over any unescaped whitespace that precedes the text of interest:
    istrm >> std::ws;

    while (!finished)
      {
        const int c = char(istrm.get());

        switch(c)
          {
          // (1) In case of unescaped whitespace or EOF, we're done:
          case EOF:
          case ' ':
          case '\t':
          case '\n':
            finished = true;
            break;

          // (2) In case of a backslash, take the subsequent character literally:
          case '\\':
            if (istrm.peek() == EOF)
              {
                // We'd better not find EOF following a backslash
                LFATAL("Incomplete escape sequence before EOF!");
              }
            buf.push_back(char(istrm.get()));
            break;

          // (3) Otherwise, take the current character literally:
          default:
            buf.push_back(char(c));
            break;
          }
      }

    // Must add a null-terminator
    buf.push_back('\0');

    // Gobble up any trailing un-escaped whitespace from the input stream
    while( istrm.peek() == '\t' || istrm.peek() == ' ' )
      istrm.get();

    return std::string(&buf[0]);
  }

  struct Param
  {
    Param() : str(), pmap() {}

    Param(double v) : str(), pmap()
    {
      std::ostringstream oss;
      oss << std::setprecision(25) << v;
      str = oss.str();
    }

    Param(int v) : str(), pmap()
    {
      std::ostringstream oss; oss << v; str = oss.str();
    }

    Param(const std::string& s) : str(s), pmap() {}

    Param(const rutz::shared_ptr<ParamMap>& p) : str(), pmap(p) {}

    void put(std::ostream& ostrm, int indentlev = 0) const
    {
      if (pmap.get() != 0)
        {
          ostrm << "{\n";
          pmap->format(ostrm, indentlev+1);
          for (int i = 0; i < indentlev; ++i) ostrm << '\t';
          ostrm << '}';
        }
      else
        {
          escapeWrite(ostrm, str);
        }
    }

    double getDouble() const
    {
      std::istringstream iss(str);
      double val;
      iss >> val;
      return val;
    }

    int getInt() const
    {
      std::istringstream iss(str);
      int val;
      iss >> val;
      return val;
    }

    rutz::shared_ptr<ParamMap> getMap() const
    {
      if (pmap.get() == 0)
        {
          LFATAL("No such parameter submap.");
        }
      return pmap;
    }

    bool isLeaf() const
    {
      if (pmap.get() == 0)
        return true;
      else
        return false;
    }

   std::string str;
    mutable rutz::shared_ptr<ParamMap> pmap;
  };
}

using namespace dummy_namespace_to_avoid_gcc411_bug_ParamMap_C;

// ######################################################################
struct ParamMap::Impl
{
  typedef std::map<std::string, Param> MapType;

  MapType itsParams;
};

// ######################################################################
struct ParamMap::key_iterator::IterRep
{
  ParamMap::Impl::MapType::const_iterator iter;
};

// ######################################################################
ParamMap::key_iterator::key_iterator() :
  rep(new IterRep)
{}

// ######################################################################
ParamMap::key_iterator::~key_iterator()
{
  delete rep;
}

// ######################################################################
ParamMap::key_iterator::key_iterator(const key_iterator& other) :
  rep(new IterRep)
{
  rep->iter = other.rep->iter;
}

// ######################################################################
ParamMap::key_iterator&
ParamMap::key_iterator::operator=(const key_iterator& other)
{
  rep->iter = other.rep->iter; return *this;
}

// ######################################################################
const std::string& ParamMap::key_iterator::operator*() const
{
  return (*(rep->iter)).first;
}

// ######################################################################
ParamMap::key_iterator&
ParamMap::key_iterator::operator++()
{
  ++(rep->iter); return *this;
}

// ######################################################################
bool ParamMap::key_iterator::operator==(const key_iterator& other) const
{
  return rep->iter == other.rep->iter;
}

// ######################################################################
bool ParamMap::key_iterator::operator!=(const key_iterator& other) const
{
  return rep->iter != other.rep->iter;
}

// ######################################################################
rutz::shared_ptr<ParamMap> ParamMap::loadPmapFile(const std::string& fname)
{
  rutz::shared_ptr<ParamMap> result(new ParamMap);
  result->load(fname);
  return result;
}

// ######################################################################
rutz::shared_ptr<ParamMap> ParamMap::loadConfFile(const std::string& fname)
{
  rutz::shared_ptr<ParamMap> pmap(new ParamMap);

  std::ifstream inFile(fname.c_str());

  int counter = 0;
  bool inComment = false;
  std::string instring;
  std::string paramName;
  while (inFile >> instring)
    {
      if (instring == "#")
        {
          // the comment code "#" toggles our in-comment state
          inComment = !inComment;
        }
      else if (!inComment) //real line found
        {
          if (paramName.length() == 0)
            {
              paramName.swap(instring);
            }
          else
            {
              pmap->putStringParam(paramName, instring);
              LINFO("%s[%d]: %s = %s",
                    fname.c_str(), counter,
                    paramName.c_str(), instring.c_str());
              paramName.clear();
              ++counter;
            }
        }
    }

  return pmap;
}

/*
// ######################################################################
template <class T>
rutz::shared_ptr<ParamMap> ParamMap::loadC_Map(const std::map <std::string, T> c_map)
{
  typedef std::map<std::string, T> TMap;
  rutz::shared_ptr<ParamMap> pmap(new ParamMap);

  int counter = 0;
  T val;
  std::string str, paramName;
  TMap::iterator i;
  for(i = c_map.begin(); i != c_map.end(); ++i)
    {
      paramName = i->first;
      val = i->second;
      std::ostringstream oss; oss << val; str = oss.str();
      pmap->putStringParam(paramName, str);
      LINFO("cmap:[%d]: %s = %s",
            counter, paramName.c_str(), str.c_str());
      paramName.clear();
      ++counter;
    }
  return pmap;
  }
*/
// ######################################################################
ParamMap::ParamMap() :
  rep(new Impl)
{}

// ######################################################################
ParamMap::ParamMap(const std::string& fname) :
  rep(new Impl)
{
  load(fname);
}

// ######################################################################
ParamMap::~ParamMap()
{
  delete rep;
}

// ######################################################################
ParamMap::key_iterator
ParamMap::keys_begin() const
{
  key_iterator result;
  result.rep->iter = rep->itsParams.begin();
  return result;
}

// ######################################################################
ParamMap::key_iterator
ParamMap::keys_end() const
{
  key_iterator result;
  result.rep->iter = rep->itsParams.end();
  return result;
}

// ######################################################################
void ParamMap::load(std::istream& istrm)
{
  while (istrm.peek() != EOF)
    {
      if (istrm.peek() == '#')
        {
          istrm.ignore(std::numeric_limits<int>::max(), '\n');
          istrm >> std::ws;
          continue;
        }

      const std::string attribname = escapeRead(istrm);

      if (istrm.peek() != '{')
        {
          std::string attribval;
          while( istrm.peek() == '\t' || istrm.peek() == ' ' )
              istrm.get();
          if ( istrm.peek() == '\n' )
              attribval = "";
          else
            attribval = escapeRead(istrm);

          rep->itsParams.insert(Impl::MapType::value_type(attribname,
                                                          attribval));
        }
      else
        {
          std::string attribval;

          int bracelevel = 0;
          // First time through loop we pick up the opening brace '{'
          do
            {
              int c = istrm.get();

              if (c == EOF)
                {
                  LFATAL("Unexpected EOF before closing brace '}'.");
                }
              switch (c)
                {
                case '{':
                  // Don't add the opening brace to the value string
                  if (bracelevel++ > 0)
                    attribval.push_back(char(c));
                  break;
                case '}':
                  // Don't add the closing brace to the value string
                  if (--bracelevel > 0)
                    attribval.push_back(char(c));
                  break;
                default:
                  attribval.push_back(char(c));
                  break;
                }
            }
          while (bracelevel > 0);

          rutz::shared_ptr<ParamMap> submap(new ParamMap);

          std::istringstream iss(attribval);
          submap->load(iss);

          rep->itsParams.insert(Impl::MapType::value_type(attribname, submap));
        }

      istrm >> std::ws;
    }
}

// ######################################################################
void ParamMap::load(const std::string& fname)
{
  std::ifstream ifs(fname.c_str());

  if (!ifs.is_open())
    {
      LFATAL("Couldn't open file '%s' for reading.", fname.c_str());
    }

  load(ifs);

  ifs.close();
}

// ######################################################################
void ParamMap::format(std::ostream& ostrm, int indentlev) const
{
  for (Impl::MapType::const_iterator
         itr = rep->itsParams.begin(), stop = rep->itsParams.end();
       itr != stop;
       ++itr)
    {
      for (int i = 0; i < indentlev; ++i) ostrm << '\t';
      escapeWrite(ostrm, (*itr).first);
      ostrm << "  ";
      (*itr).second.put(ostrm, indentlev);
      ostrm << '\n';
    }
}

// ######################################################################
void ParamMap::format(const std::string& fname) const
{
  std::ofstream ofs(fname.c_str());

  if (!ofs.is_open())
    {
      LFATAL("Couldn't open file '%s' for writing.", fname.c_str());
    }

  format(ofs);

  ofs.close();
}

// ######################################################################
bool ParamMap::hasParam(const std::string& paramname) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  return (itr != rep->itsParams.end());
}

// ######################################################################
bool ParamMap::isLeaf(const std::string& paramname) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr->second.isLeaf())
    return true;
  else
    return false;
}

// ######################################################################
uint ParamMap::getsize() const
{
  return rep->itsParams.size();
}

// ######################################################################
rutz::shared_ptr<ParamMap> ParamMap::getSubpmap(const std::string& paramname) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr == rep->itsParams.end())
    {
      LFATAL("No parameter named '%s'.", paramname.c_str());
    }
  return (*itr).second.getMap();
}

// ######################################################################
std::string ParamMap::getStringParam(const std::string& paramname) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr == rep->itsParams.end())
    {
      LFATAL("No parameter named '%s'.", paramname.c_str());
    }
  return (*itr).second.str;
}

// ######################################################################
double ParamMap::getDoubleParam(const std::string& paramname) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr == rep->itsParams.end())
    {
      LFATAL("No parameter named '%s'.", paramname.c_str());
    }
  return (*itr).second.getDouble();
}

// ######################################################################
int ParamMap::getIntParam(const std::string& paramname) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr == rep->itsParams.end())
    {
      LFATAL("No parameter named '%s'.", paramname.c_str());
    }
  return (*itr).second.getInt();
}

// ######################################################################
std::string ParamMap::getStringParam(const std::string& paramname,
                                     const std::string& defval) const
{
  std::string result = defval;
  queryStringParam(paramname, result);
  return result;
}

// ######################################################################
double ParamMap::getDoubleParam(const std::string& paramname,
                                const double defval) const
{
  double result = defval;
  queryDoubleParam(paramname, result);
  return result;
}

// ######################################################################
int ParamMap::getIntParam(const std::string& paramname,
                          const int defval) const
{
  int result = defval;
  queryIntParam(paramname, result);
  return result;
}

// ######################################################################
rutz::shared_ptr<ParamMap> ParamMap::lookupSubpmap(const std::string& paramname)
{
  if (hasParam(paramname))
    return getSubpmap(paramname);

  // else...
  rutz::shared_ptr<ParamMap> submap(new ParamMap);
  putSubpmap(paramname, submap);
  return submap;
}

// ######################################################################
ParamMap::ReturnCode
ParamMap::queryStringParam(const std::string& paramname, std::string& result) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr != rep->itsParams.end())
    {
      const std::string new_val = (*itr).second.str;
      if (result.compare(new_val) != 0)
        {
          result = new_val;
          return CHANGED;
        }
      else
        return UNCHANGED;
    }

  // else...
  LINFO("Parameter '%s' not found; using default value '%s'.",
        paramname.c_str(), result.c_str());
  return MISSING;
}

// ######################################################################
ParamMap::ReturnCode
ParamMap::queryDoubleParam(const std::string& paramname, double& result) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr != rep->itsParams.end())
    {
      const double new_val = (*itr).second.getDouble();
      if (new_val != result)
        {
          result = new_val;
          return CHANGED;
        }
      else
        return UNCHANGED;
    }

  // else...
  LINFO("Parameter '%s' not found; using default value '%f'.",
        paramname.c_str(), result);
  return MISSING;
}

// ######################################################################
ParamMap::ReturnCode
ParamMap::queryIntParam(const std::string& paramname, int& result) const
{
  Impl::MapType::const_iterator itr = rep->itsParams.find(paramname);
  if (itr != rep->itsParams.end())
    {
      const int new_val = (*itr).second.getInt();
      if (new_val != result)
        {
          result = new_val;
          return CHANGED;
        }
      else
        return UNCHANGED;
    }

  // else...
  LINFO("Parameter '%s' not found; using default value '%d'.",
        paramname.c_str(), result);
  return MISSING;
}

// ######################################################################
void ParamMap::putSubpmap(const std::string& paramname, const rutz::shared_ptr<ParamMap>& val)
{
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::putStringParam(const std::string& paramname, const std::string& val)
{
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::putDoubleParam(const std::string& paramname, double val)
{
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::putIntParam(const std::string& paramname, int val)
{
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::replaceSubpmap(const std::string& paramname,
                              const rutz::shared_ptr<ParamMap>& val)
{
  rep->itsParams.erase(rep->itsParams.find(paramname));
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::replaceStringParam(const std::string& paramname, const std::string& val)
{
  rep->itsParams.erase(rep->itsParams.find(paramname));
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::replaceDoubleParam(const std::string& paramname, double val)
{
  rep->itsParams.erase(rep->itsParams.find(paramname));
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::replaceIntParam(const std::string& paramname, int val)
{
  rep->itsParams.erase(rep->itsParams.find(paramname));
  rep->itsParams.insert(Impl::MapType::value_type(paramname, val));
}

// ######################################################################
void ParamMap::clear()
{
   rep->itsParams.clear();
}

// ######################################################################
void ParamMap::erase(const std::string& paramname)
{
  // should have error handling
  rep->itsParams.erase(rep->itsParams.find(paramname));
}

// ######################################################################
void ParamMap::print(const std::string& name) const
{
  std::cout << "--- begin " << name << " ---\n";
  for (Impl::MapType::const_iterator
         itr = rep->itsParams.begin(), stop = rep->itsParams.end();
       itr != stop;
       ++itr)
    {
      std::cout << "name: " << (*itr).first <<"****" <<std::endl;
      std::cout << "val: " << (*itr).second.str <<"****"<< std::endl;
    }
  std::cout << "--- end " << name << " ---\n";
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // !PARAMMAP_C_DEFINED
