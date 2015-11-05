/*!@file Util/StringUtil.H */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2005   //
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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/StringUtil.H $
// $Id: StringUtil.H 14609 2011-03-15 17:48:32Z rand $
//

#ifndef UTIL_STRINGUTIL_H_DEFINED
#define UTIL_STRINGUTIL_H_DEFINED

#include <cctype> // for tolower(), toupper()
#include <set>
#include <string>

/// Split a string into tokens separated by the given delimiters.
/** Example:

    \code

    string s = "/:/foo::bar/:baz///";
    vector<string> tokens;
    split(s, ":/", std::back_inserter(tokens));

    // tokens now contains { "foo", "bar", "baz" };

    \endcode

 */
template <class Itr> inline
void split(const std::string& in, const char* delim, Itr out)
{
  // Find start of the first token:
  std::string::size_type start = in.find_first_not_of(delim, 0);

  // Find end of the first token:
  std::string::size_type end   = in.find_first_of(delim, start);

  while (end != std::string::npos || start != std::string::npos)
    {
      // Extract the current token and save it:
      *out++ = in.substr(start, end - start);

      // Skip delimiters.  Note the "not_of"
      start = in.find_first_not_of(delim, end);

      // Find end of the next token:
      end = in.find_first_of(delim, start);
    }
}

/// Join a sequence of tokens with the given delimiter
/** Example:

    \code

    const char* toks[] = { "foo", "bar", "baz" };
    string s = join(&toks[0], &toks[0] + 3, "||");

    // s == "foo||bar||baz";

    \endcode
*/
template <class Itr> inline
std::string join(Itr it, Itr stop, const char* delim)
{
  std::string result;
  bool first = true;
  while (it != stop)
    {
      if (!first) { result += delim; }
      result += (*it++);
      first = false;
    }
  return result;
}

/// Wrap a sequence of words into a multi-line string, with optional prefix and suffix
/** Use stdLineWrap() if you have a single string that should be first
    split into words on standard whitespace and then line-wrapped. */
template <class Itr> inline
std::string lineWrap(Itr itr, Itr stop,
                     size_t linelength,
                     const std::string& pfx = std::string(),
                     const std::string& sfx = std::string())
{
  const size_t corelen =
    pfx.length() + sfx.length() < linelength
    ? (linelength - pfx.length() - sfx.length())
    : 1;

  std::string out;
  std::string line;

  for ( ; itr != stop; ++itr)
    {
      if ((*itr).length() + line.length() + 1 <= corelen)
        {
          if (line.length() > 0)
            line += ' ';
          line += *itr;
        }
      else
        {
          out += pfx;
          out += line;
          out += sfx;
          out += '\n';
          line = *itr;
        }
    }

  out += pfx;
  out += line;
  out += sfx;

  return out;
}

/// Split a string on whitespace, then wrap into a multi-line string with optional prefix and suffix
std::string stdLineWrap(const std::string& in, size_t linelength,
                        const std::string& pfx = std::string(),
                        const std::string& sfx = std::string());

/// Make a lowercase string out of the input string
inline std::string toLowerCase(const std::string& in)
{
  std::string out = in;
  for (std::string::size_type i = 0; i < out.size(); ++i)
    out[i] = tolower(out[i]);
  return out;
}

/// Make an uppercase string out of the input string
inline std::string toUpperCase(const std::string& in)
{
  std::string out = in;
  for (std::string::size_type i = 0; i < out.size(); ++i)
    out[i] = toupper(out[i]);
  return out;
}

/// Compute the Levenshtein distance between two strings
/** http://en.wikipedia.org/wiki/Levenshtein_distance */
unsigned int levenshteinDistance(const std::string& s,
                                 const std::string& t);

/// Compute the Damerau-Levenshtein distance between two strings
/** http://en.wikipedia.org/wiki/Damerau-Levenshtein_distance */
unsigned int damerauLevenshteinDistance(const std::string& s,
                                        const std::string& t);

/// Convert a CamelCase string to a string of space-separated words
/** @param acronyms If non-null, this set of uppercase acronyms will
    be used as a heuristic to find proper word boundaries. */
std::string camelCaseToSpaces(const std::string& s,
                              const std::set<std::string>* acronyms);

/// Remove whitespace from the beginning and end of the input string
std::string trim(std::string const& str);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_STRINGUTIL_H_DEFINED
