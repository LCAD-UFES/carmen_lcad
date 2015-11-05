/**
   \file  Robots/LoBot/util/LoFile.H
   \brief File system related functions.

   Various high-level functions encapsulating the underlying filesystem
   API. These functions either wrap around Boost.Filesystem or fallback
   to using the Unix API.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/util/LoFile.H $
// $Id: LoFile.H 14083 2010-09-30 13:59:37Z mviswana $
//

#ifndef LOBOT_FILESYSTEM_UTILITIES_DOT_H
#define LOBOT_FILESYSTEM_UTILITIES_DOT_H

//------------------------------ HEADERS --------------------------------

// Standard C++ headers
#include <string>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//---------------------------- FILE NAMES -------------------------------

/// Given a file's path name, this function returns everything up to the
/// final path separator, i.e., the absolute or relative directory
/// hierarchy component of the file name.
std::string dirname(const std::string& path) ;

/// This function returns just the file name component of an absolute or
/// relative path specification, i.e., everything after the last
/// directory separator.
std::string basename(const std::string& path) ;

/// This function returns just the extension component of the file name,
/// i.e., the part of the basename that comes after the first period.
std::string extension(const std::string& path) ;

//---------------------------- PATH TESTS -------------------------------

/// This function returns true if the given path name refers to an extant
/// file or directory.
bool exists(const std::string& path) ;

/// This function returns true if the given path name refers to an extant
/// directory, i.e., the specified filesystem object must exist and must
/// be a directory.
bool is_dir(const std::string& path) ;

/// This function returns true if the given path name refers to an extant
/// file, i.e., the specified filesystem object must exist and must be a
/// file.
bool is_file(const std::string& path) ;

//----------------------- DIRECTORY OPERATIONS --------------------------

/// This function returns the contents of the specified directory. It
/// performs a shallow listing, not a deep one. That is, it is equivalent
/// to a bare/unadorned "ls" command as opposed to an "ls -R".
///
/// The list returned by this function is an std::vector of std::string
/// objects. All the path names in this list are full (not necessarily
/// absolute) w.r.t. the given path. Furthermore, this function does not
/// return the "." and ".." entries in a directory.
///
/// For example, calling this function with the argument "/home/foo" will
/// result in a one-level listing of "/home/foo" with all the entries in
/// the list beginning with "/home/foo/". On the other hand, a relative
/// path specification such "foo" will return a one-level listing of
/// ./foo (where the "." is implicit) with all entries beginning with
/// "foo/".
///
/// The directory listing returned by this function is not sorted. And,
/// as mentioned above, does not contain "." and "..".
std::vector<std::string> list_dir(const std::string& path) ;

//------------------- FINDING FILES AND DIRECTORIES ---------------------

/// This function returns a sorted list of file names under the specified
/// directory that match the given file name or pattern. The entire
/// directory tree rooted at the point specified by the first parameter
/// is searched, i.e., this function implements a "deep" search and not
/// just a one-level search.
///
/// This function treats the second parameter as a regular expression and
/// returns a list of all the files whose names match the given regex.
/// Please note that regex matching is not the same as shell glob
/// patterns and wildcards.
///
/// The list returned by this function is an std::vector of std::string
/// objects containing the full (not necessarily absolute) path names of
/// all the files under the given directory that match the given target
/// pattern.
///
/// For example, given "/home/foo" as the first argument and "bar" as the
/// second argument, this function will return a list of all files under
/// /home/foo whose names contain "bar". The path names will all begin
/// with "/home/foo/". If the first argument were simply "foo", then all
/// returned path names will begin with "foo/".
///
/// The first parameter must be the name of an extant directory.
/// Otherwise, an empty list will be returned.
///
/// NOTE: This function does not throw any exceptions. Thus, the caller
/// will not be able to tell whether an empty list indicates no matches
/// or a failure of some sort. Sorry.
std::vector<std::string>
find_file(const std::string& dir, const std::string& file_name) ;

/// This function returns a sorted list of directory names under the
/// specified directory that match the given name or pattern. The entire
/// directory tree rooted at the point specified by the first parameter
/// is searched, i.e., this function implements a "deep" search and not
/// just a one-level search.
///
/// This function treats the second parameter as a regular expression and
/// returns a list of all the directories whose names match the given
/// regex. Please note that regex matching is not the same as shell glob
/// patterns and wildcards.
///
/// The list returned by this function is an std::vector of std::string
/// objects containing the full (not necessarily absolute) path names of
/// all the subdirectories under the given directory that match the given
/// target pattern.
///
/// For example, given "/home/foo" as the first argument and "bar" as the
/// second argument, this function will return a list of all
/// subdirectories under /home/foo whose names contain "bar". The path
/// names will all begin with "/home/foo/". If the first argument were
/// simply "foo", then all returned path names will begin with "foo/".
///
/// The first parameter must be the name of an extant directory.
/// Otherwise, an empty list will be returned.
///
/// NOTE: This function does not throw any exceptions. Thus, the caller
/// will not be able to tell whether an empty list indicates no matches
/// or a failure of some sort. Sorry.
std::vector<std::string>
find_dir(const std::string& dir, const std::string& subdir_name) ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
