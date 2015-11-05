/*!@file Util/FileUtil.H Utility routines for working with filenames and files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/FileUtil.H $
// $Id: FileUtil.H 8790 2007-09-28 22:24:10Z rjpeters $
//

#ifndef UTIL_FILEUTIL_H_DEFINED
#define UTIL_FILEUTIL_H_DEFINED

#include <cerrno> // for errno
#include <cstdio> // for perror()
#include <dirent.h>
#include <fcntl.h> // for open()
#include <iosfwd>
#include <stdio.h>
#include <string>
#include <sys/file.h>
#include <sys/types.h>

namespace rutz { template <class T> class shared_ptr; }

/// Like fopen(fname, "w"), with some special cases
/** @param fname if fname is "" (the empty string), "-", "stdout", or
    "STDOUT", then the returned FILE* will be stdout and *owned will
    be set to false; if fname is "stderr" or "STDERR", then the
    returned FILE* will be stderr and *owned will be set to false;
    otherwise, the returned FILE* will be fopen(fname,"w") and *owned
    will be set to true (if fopen() fails then LFATAL() will be
    called)

    @param owned must be non-null; on return, *owned will indicates
    whether the returned FILE* was newly fopen()-ed; if *owned is
    true, then the caller is responsible for eventually fclose()-ing
    the returned FILE*

    @return guaranteed to be a non-null FILE* (if the internal fopen()
    fails, LFATAL() will be called)
*/
FILE* stdOutputFileOpen(const std::string& fname, bool* owned);

/// case-INsensitive check of whether str ends with ext
bool hasExtension(const std::string& str, const std::string& ext);

/// get the filename extension, INCLUDING the dot (e.g. ".jpg")
/** returns an empty string if there is no extension on the filename */
std::string dotExtension(const std::string& in);

/// get the filename extension, EXCLUDING the dot (e.g., "jpg")
/** returns an empty string if there is no extension on the filename

    @param base If non-null, the base portion of the filename, but NOT
    INCLUDING the dot, will be returned here. So, the original
    filename could be reconstructed as "BASE.EXT"
*/
std::string nodotExtension(const std::string& in,
                           std::string* base = 0);

/// open a (possibly-compressed) file for reading
/** If the filename ends in ".gz", then the file is assumed to be
    gzip-compressed (likewise, ".bz2" implies bzip2-compressed), and
    the returned istream object will transparently
    decompressed. Otherwise, the file will be assumed to be
    uncompressed.

    If the file cannot be opened, an exception will be thrown.
*/
rutz::shared_ptr<std::istream>
openMaybeCompressedFile(const std::string& fname);

/// split a path into a directory prefix (if any) and a filename
/** the directory prefix will be empty if there are no path separators
    ('/') in the path

    e.g. splitting "foo/bar.mpg" would give "foo/" and "bar.mpg"
*/
void splitPath(const std::string& fname,
               std::string& path, std::string& file);

/// mkdir the given directory
/** Not an error if the directory already exists, but it is an error
    if a file (not a directory) exists with the given name. */
void makeDirectory(const std::string& dirname);

/// Returns true if fname refers to a directory.
/** Returns false in any other case (e.g., fname refers to a regular
    file, or refers to no file at all, etc.) */
bool isDirectory(const char* fname);

/// Returns true if dirp refers to a directory.
/** If dirent::d_type is available, use that; otherwise fall back to
    using a stat() call. */
bool isDirectory(const struct dirent* dirp);

//! Use fcntl to set and unset advisory locks on a file
/*! You must include stdio.h, fcntl.h, cerrno and cstdio
    fl is of the type struct flock.
    This function is provided since fcntl is kind of weird to set
    and unset. Also, fcntl locks are different from flock locks
    for instance linux apache will only check fcntl based locks
*/
ushort lockFile(const int fd, struct flock &fl);
//! Use fcntl to set and unset advisory locks on a file
ushort unlockFile(const int fd, struct flock &fl);
//! lock a file by name
ushort lockFile(const std::string fileName, int &fd, struct flock &fl);
//! unlock a file and close
ushort unlockFile(const std::string fileName, const int fd, struct flock &fl);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_FILEUTIL_H_DEFINED
