/*!@file Util/StringUtil.C */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/StringUtil.C $
// $Id: StringUtil.C 14610 2011-03-15 17:53:47Z rand $
//

#ifndef UTIL_STRINGUTIL_C_DEFINED
#define UTIL_STRINGUTIL_C_DEFINED

#include "Util/StringUtil.H"

#include "Util/Assert.H"

#include <algorithm>
#include <iterator>
#include <vector>

// ######################################################################
std::string stdLineWrap(const std::string& in, size_t linelength,
                        const std::string& pfx, const std::string& sfx)
{
  std::vector<std::string> words;

  split(in, " \t\n", std::back_inserter(words));

  return lineWrap(words.begin(), words.end(), linelength, pfx, sfx);
}

// ######################################################################
unsigned int levenshteinDistance(const std::string& s,
                                 const std::string& t)
{
  const size_t m = s.length();
  const size_t n = t.length();

  std::vector<int> d((m+1) * (n+1)); // m+1 columns, n+1 rows

#define D_INDEX(i, j) ((i) + ((j)*(m+1)))

  for (size_t i = 0; i <= m; ++i)
    d[D_INDEX(i,0)] = i;

  for (size_t j = 0; j <= n; ++j)
    d[D_INDEX(0,j)] = j;

  for (size_t i = 1; i <= m; ++i)
    for (size_t j = 1; j <= n; ++j)
      {
        const int cost = s[i-1] == t[j-1] ? 0 : 1;

        d[D_INDEX(i,j)] =
          std::min(std::min(d[D_INDEX(i-1,j)] + 1 /* deletion */,
                            d[D_INDEX(i,j-1)] + 1 /* insertion */),
                   d[D_INDEX(i-1,j-1)] + cost /* substitution */);
      }

  ASSERT(d[D_INDEX(m,n)] >= 0);

  return (unsigned int)(d[D_INDEX(m,n)]);

#undef D_INDEX
}

// ######################################################################
unsigned int damerauLevenshteinDistance(const std::string& s,
                                        const std::string& t)
{
  const size_t m = s.length();
  const size_t n = t.length();

  std::vector<int> d((m+1) * (n+1)); // m+1 columns, n+1 rows

#define D_INDEX(i, j) ((i) + ((j)*(m+1)))

  for (size_t i = 0; i <= m; ++i)
    d[D_INDEX(i,0)] = i;

  for (size_t j = 0; j <= n; ++j)
    d[D_INDEX(0,j)] = j;

  for (size_t i = 1; i <= m; ++i)
    for (size_t j = 1; j <= n; ++j)
      {
        const int cost = s[i-1] == t[j-1] ? 0 : 1;

        d[D_INDEX(i,j)] =
          std::min(std::min(d[D_INDEX(i-1,j)] + 1 /* deletion */,
                            d[D_INDEX(i,j-1)] + 1 /* insertion */),
                   d[D_INDEX(i-1,j-1)] + cost /* substitution */);

        if (i > 1 && j > 1 && s[i-1] == t[j-2] && s[i-2] == t[j-1])
          d[D_INDEX(i,j)] =
            std::min(d[D_INDEX(i,j)],
                     d[D_INDEX(i-2,j-2)] + cost /* transposition */);
      }

  ASSERT(d[D_INDEX(m,n)] >= 0);

  return (unsigned int)(d[D_INDEX(m,n)]);

#undef D_INDEX
}

// ######################################################################
std::string camelCaseToSpaces(const std::string& s,
                              const std::set<std::string>* acronyms)
{
  std::string r;

  std::string upword;

  for (size_t i = 0; i < s.length(); ++i)
    {
      const bool is_last_char = (i + 1 == s.length());

      bool wasupper = false;
      if (isupper(s[i])
          ||
          // digits stick to preceding uppercase strings:
          (upword.length() > 0 && isdigit(s[i])))
        {
          upword += s[i];
          wasupper = true;
        }

      // do we need to inject our collected uppercase word into the
      // result?
      if (!wasupper || is_last_char)
        {
          if (upword.length() > 0)
            {
              // Is our full uppercase word a known acronym?
              if (acronyms != 0
                  && acronyms->find(upword) != acronyms->end())
                {
                  if (r.length() > 0)
                    r += ' ';

                  for (size_t j = 0; j < upword.size(); ++j)
                    r += tolower(upword[j]);

                  if (!is_last_char)
                    r += ' ';
                }
              // Otherwise are the first n-1 uppercase chars a known
              // acronym (in which case the last char would be the
              // first char of the next word)?
              else if (upword.length() >= 2
                       && acronyms != 0
                       &&
                       (acronyms->find(upword.substr(0, upword.length()-1))
                        != acronyms->end()))
                {
                  if (r.length() > 0)
                    r += ' ';

                  for (size_t j = 0; j < upword.size() - 1; ++j)
                    r += tolower(upword[j]);

                  r += ' ';
                  r += tolower(upword[upword.size() - 1]);
                }
              // Otherwise we treat each uppercase char as its own
              // word:
              else
                {
                  for (size_t j = 0; j < upword.size(); ++j)
                    {
                      if (r.length() > 0)
                        r += ' ';
                      r += tolower(upword[j]);
                    }
                }

              upword = "";
            }
        }

      if (!wasupper)
        {
          r += s[i];
        }
    }

  return r;
}


// ######################################################################
std::string trim(std::string const& str)
{
	size_t f = str.find_first_not_of(" ");
	size_t l = str.find_last_not_of(" ");
	return str.substr(f, l-f+1);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_STRINGUTIL_C_DEFINED
