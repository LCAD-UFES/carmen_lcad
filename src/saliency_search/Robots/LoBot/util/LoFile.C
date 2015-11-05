/**
   \file  Robots/LoBot/util/LoFile.C
   \brief Definitions of functions defined in LoFile.H
*/

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
// Primary maintainer for this file: mviswana usc edu
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoFile.C $
// $Id: LoFile.C 14083 2010-09-30 13:59:37Z mviswana $
//

//-------------------------- BOOST-BASED API ----------------------------

#ifdef INVT_HAVE_BOOST_FILESYSTEM

// lobot headers
#include "Robots/LoBot/util/LoFile.H"

// Boost headers
#include <boost/filesystem/operations.hpp>

// Standard C++ headers
#include <algorithm>
#include <deque>
#include <functional>
#include <iterator>

namespace lobot {

// File/path name manipulations
std::string dirname(const std::string& path)
{
   namespace fs = boost::filesystem ;
   return fs::path(path).branch_path().string() ;
}

std::string basename(const std::string& path)
{
   namespace fs = boost::filesystem ;
   return fs::path(path).leaf() ;
}

// File/path name tests
bool exists(const std::string& path)
{
   namespace fs = boost::filesystem ;
   return fs::exists(fs::path(path));
}

bool is_dir(const std::string& path)
{
   namespace fs = boost::filesystem ;
   fs::path p(path) ;
   return fs::exists(p) && fs::is_directory(p) ;
}

bool is_file(const std::string& path)
{
   return !is_dir(path) ;
}

// Directory operations
std::vector<std::string> list_dir(const std::string& path)
{
   namespace fs = boost::filesystem ;
   fs::path p(path) ;
   std::vector<std::string> ls ;
   std::transform(fs::directory_iterator(p), fs::directory_iterator(),
                  std::back_inserter(ls), std::mem_fun_ref(&fs::path::string));
   return ls ;
}

} // end of namespace encapsulating above definitions

#else // Boost.Filesystem not available

//-------------------------- UNIX-BASED API -----------------------------

// lobot headers
#include "Robots/LoBot/util/LoFile.H"
#include "Robots/LoBot/util/LoSysConf.H"

// Standard C++ headers
#include <algorithm>
#include <deque>
#include <iterator>

// Standard C headers
#include <stdlib.h>
#include <stddef.h>

// Unix headers
#include <libgen.h>
#include <dirent.h>
#include <sys/stat.h>
#include <sys/types.h>

// Helper function to copy an std::string to a dynamically allocated char
// array. The client must delete the char array returned by this
// function.
static char* dup_string(const std::string& s)
{
   const int N = s.size() ;
   char* tmp   = new char[N + 1] ;
   std::copy(s.begin(), s.end(), tmp) ;
   tmp[N] = '\0' ;
   return tmp ;
}

namespace lobot {

// File/path name manipulations
std::string dirname(const std::string& path)
{
   char* tmp = dup_string(path) ;
   std::string dname = ::dirname(tmp) ;
   delete[] tmp ;
   return dname ;
}

std::string basename(const std::string& path)
{
   char* tmp = dup_string(path) ;
   std::string bname = ::basename(tmp) ;
   delete[] tmp ;
   return bname ;
}

// File/path name tests
bool exists(const std::string& path)
{
   struct stat buf ;
   return stat(path.c_str(), &buf) == 0 ;
}

bool is_dir(const std::string& path)
{
   struct stat buf ;
   return stat(path.c_str(), &buf) == 0 && S_ISDIR(buf.st_mode) ;
}

bool is_file(const std::string& path)
{
   struct stat buf ;
   return stat(path.c_str(), &buf) == 0 && S_ISREG(buf.st_mode) ;
}

// Directory operations
//
// DEVNOTE: This code is adapted from Marc J. Rochkind's "Advanced Unix
// Programming." See chapter 3 (Advanced File I/O), section 3.6.1:
// Reading Directories.
std::vector<std::string> list_dir(const std::string& path)
{
   typedef std::vector<std::string> list ;

   const int N = maxlen_filename(path.c_str()) ;
   struct dirent* input = reinterpret_cast<struct dirent*>(
                             malloc(offsetof(struct dirent, d_name) + N + 1)) ;
   if (input == NULL) // gave it our best shot, but now system is out of memory
      return list() ;

   DIR* dir = opendir(path.c_str()) ;
   if (dir == NULL) { // couldn't open directory for reading
      free(input) ;
      return list() ;
   }

   list ls ;
   for(;;)
   {
      struct dirent* output = NULL ;
      readdir_r(dir, input, &output) ;
      if (output == NULL) // error or end of directory entries
         break ;

      std::string file_name = output->d_name ;
      if (file_name == "." || file_name == "..")
         continue ;
      ls.push_back(path + "/" + file_name) ;
   }

   closedir(dir) ;
   free(input) ;
   return ls ;
}

} // end of namespace encapsulating above functions

#endif // INVT_HAVE_BOOST_FILESYSTEM

//-------------------------- BOOST-BASED API ----------------------------

#ifdef INVT_HAVE_BOOST_REGEX

#include <boost/regex.hpp>

// Helper function to perform an exact string match between name and target
static bool match_exact(const std::string& name, const std::string& target)
{
   return name == target ;
}

namespace lobot {

// Helper function object to perform regular expression matching between
// name and target.
//
// NOTE: The function call operator will be passed two parameters, viz.,
// name and target (just like match_exact). However, we do not use the
// second parameter, viz., target, because it is expected to be a regular
// expression, which we would have compiled during instantiation.
// Therefore, the function call operator compares against the precompiled
// regex rather than the supplied target parameter.
//
// DEVNOTE: Even though the function call operator ignores its second
// parameter, it must nonetheless be accepted because this function
// object is used in conjunction with find_helper(), which expects its
// matching predicate to take two parameters.
class match_regex {
   boost::regex m_regex ;
public:
   match_regex(const std::string& pattern) ;
   bool operator()(const std::string& name, const std::string&) const {
      return boost::regex_search(name, m_regex) ;
   }
} ;

match_regex::match_regex(const std::string& pattern)
   : m_regex(pattern)
{}

// Return list of all files or directories under given directory using
// the given predicate to match file names against the supplied target
// pattern.
template<typename pred>
static std::vector<std::string>
find_helper(const std::string& dir, const std::string& target,
            pred match, bool (*is_type)(const std::string&))
{
   typedef std::vector<std::string> list ;
   if (! is_dir(dir))
      return list() ;

   list matches ;
   std::deque <std::string> fringe ; // queue of unexplored items
   fringe.push_back(dir) ;

   while (! fringe.empty())
   {
      std::string item = fringe.front() ;
      fringe.pop_front() ;

      if (is_dir(item)) {
         list ls = list_dir(item) ;
         std::copy(ls.begin(), ls.end(), std::back_inserter(fringe)) ;
      }

      if (is_type(item) && match(item, target))
         matches.push_back(item) ;
   }
   std::sort(matches.begin(), matches.end()) ;
   return matches ;
}

// Return list of all files under given directory matching specified name
std::vector<std::string>
find_file(const std::string& dir, const std::string& target)
{
   try
   {
      return find_helper(dir, target, match_regex(target), is_file) ;
   }
   catch (boost::regex_error&){}
   return find_helper(dir, target, match_exact, is_file) ;
}

// Return list of all subdirectories of given directory matching specified name
std::vector<std::string>
find_dir(const std::string& dir, const std::string& target)
{
   try
   {
      return find_helper(dir, target, match_regex(target), is_dir) ;
   }
   catch (boost::regex_error&){}
   return find_helper(dir, target, match_exact, is_dir) ;
}

} // end of namespace lobot

#else // Boost.Regex not available

//-------------------------- UNIX-BASED API -----------------------------

#include <regex.h>

// Helper function to perform an exact string match between name and target
static bool
match_exact(const std::string& name, const std::string& target, const regex_t*)
{
   return name == target ;
}

// Helper function to match given name against a regular expression
static bool
match_regex(const std::string& name, const std::string&, const regex_t* regex)
{
   return regexec(regex, name.c_str(), 0, 0, 0) == 0 ;
}

namespace lobot {

// Return list of all files or directories under given directory matching
// specified name.
static std::vector<std::string>
find_helper(const std::string& dir, const std::string& target,
            bool (*is_type)(const std::string&))
{
   typedef std::vector<std::string> list ;
   if (! is_dir(dir))
      return list() ;

   typedef bool (*MatchFunc)(const std::string&,
                             const std::string&, const regex_t*) ;
   MatchFunc match = match_exact ;

   regex_t regex ;
   if (regcomp(&regex, target.c_str(),
               REG_EXTENDED | REG_NOSUB | REG_NEWLINE) == 0)
      match = match_regex ;

   list matches ;
   std::deque<std::string> fringe ; // queue of unexplored items
   fringe.push_back(dir) ;

   while (! fringe.empty())
   {
      std::string item = fringe.front() ;
      fringe.pop_front() ;

      if (is_dir(item)) {
         list ls = list_dir(item) ;
         std::copy(ls.begin(), ls.end(), std::back_inserter(fringe)) ;
      }

      if (is_type(item) && match(item, target, &regex))
         matches.push_back(item) ;
   }

   if (match == match_regex)
      regfree(&regex) ;

   std::sort(matches.begin(), matches.end()) ;
   return matches ;
}

// Return list of files under given directory matching specified name
std::vector<std::string>
find_file(const std::string& dir, const std::string& target)
{
   return find_helper(dir, target, is_file) ;
}

// Return list of subdirectories under given directory matching specified name
std::vector<std::string>
find_dir(const std::string& dir, const std::string& target)
{
   return find_helper(dir, target, is_dir) ;
}

} // end of namespace lobot

#endif // INVT_HAVE_BOOST_REGEX

//---------------------- COMMON IMPLEMENTATIONS -------------------------

// These functions are identical regardless of whether we use Boost or
// the Unix API.
namespace lobot {

// Return the file name's extension
std::string extension(const std::string& path)
{
   std::string base_name = basename(path) ;
   return base_name.substr(base_name.find('.') + 1) ;
}

} // end of namespace lobot

//-----------------------------------------------------------------------

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
