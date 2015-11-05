/*!@file Util/FileUtil.C Utility routines for working with filenames and files */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/FileUtil.C $
// $Id: FileUtil.C 8895 2007-10-24 22:14:24Z rjpeters $
//

#ifndef UTIL_FILEUTIL_C_DEFINED
#define UTIL_FILEUTIL_C_DEFINED

#include "Util/FileUtil.H"

#include "Util/log.H"
#include "rutz/bzip2stream.h"
#include "rutz/gzstreambuf.h"
#include "rutz/shared_ptr.h"

#include <cstring>
#include <fstream>
#include <sys/stat.h> // for mkdir(), stat()
#include <sys/types.h> // for mkdir(), stat()
#include <unistd.h> // for stat()

// ######################################################################
FILE* stdOutputFileOpen(const std::string& fname, bool* owned)
{
  if (fname.compare("") == 0
      || fname.compare("-") == 0
      || fname.compare("stdout") == 0
      || fname.compare("STDOUT") == 0)
    {
      *owned = false;
      return stdout;
    }
  else if (fname.compare("stderr") == 0
           || fname.compare("STDERR") == 0)
    {
      *owned = false;
      return stderr;
    }

  // else ...
  FILE* const result = fopen(fname.c_str(), "w");

  if (result == 0)
    LFATAL("couldn't open file '%s' for writing",
           fname.c_str());

  *owned = true;
  return result;
}

// ######################################################################
bool hasExtension(const std::string& str, const std::string& ext)
{
  if (ext.length() > str.length())
    return false;

  return (strcasecmp(str.c_str() + (str.length() - ext.length()),
                     ext.c_str())
          == 0);
}

// ######################################################################
std::string dotExtension(const std::string& in)
{
  std::string::size_type slash = in.rfind('/');
  std::string::size_type dot = in.rfind('.');

  if (dot != in.npos && (slash == std::string::npos || dot > slash))
    return in.substr(dot);
  // else ...
  return std::string();
}

// ######################################################################
std::string nodotExtension(const std::string& in,
                           std::string* base_out)
{
  std::string::size_type slash = in.rfind('/');
  std::string::size_type dot = in.rfind('.');

  const bool valid =
    dot != std::string::npos
    && (slash == std::string::npos || dot > slash)
    && dot+1 < in.size();

  const std::string ext =
    valid
    ? in.substr(dot+1)
    : std::string();

  const std::string base =
    valid
    ? in.substr(0, dot)
    : in;

  // don't modify *base_out until we're after done fiddling with in,
  // because the caller may have passed the same string object for
  // both args, as in nodotExtension(foo, &foo)
  if (base_out != 0)
    *base_out = base;

  return ext;
}

// ######################################################################
rutz::shared_ptr<std::istream>
openMaybeCompressedFile(const std::string& fname)
{
  if (hasExtension(fname, ".gz"))
    {
      return rutz::igzopen(fname.c_str());
    }
  else if (hasExtension(fname, ".bz2"))
    {
      return rutz::ibzip2open(fname.c_str());
    }
  // else...
  rutz::shared_ptr<std::ifstream> f =
    rutz::make_shared(new std::ifstream(fname.c_str()));

  if (!f->is_open())
    LFATAL("Couldn't open file '%s' for reading.", fname.c_str());

  return f;
}

// ######################################################################
void splitPath(const std::string& fname,
               std::string& path, std::string& file)
{
  std::string::size_type idx = fname.rfind('/');
  if (idx == fname.npos)
    {
      path = "";
      file = fname;
    }
  else
    {
      path = fname.substr(0, idx) + '/';
      file = fname.substr(idx+1, fname.npos);
    }
}

// ######################################################################
void makeDirectory(const std::string& dirname)
{
  struct stat info;
  // see if the named file/directory exists
  if (stat(dirname.c_str(), &info) == 0)
    {
      if (S_ISDIR(info.st_mode))
        return;
      else
        LFATAL("'%s' already exists but is not a directory",
               dirname.c_str());
    }

  if (mkdir(dirname.c_str(), S_IRWXU | S_IRWXG | S_IRWXO) != 0)
    LFATAL("couldn't mkdir '%s'", dirname.c_str());
}

// ######################################################################
bool isDirectory(const char* fname)
{
  struct stat statbuf;
  errno = 0;
  if (stat(fname, &statbuf) != 0)
    {
      // OK, the stat itself failed, so fname can't be a directory:
      return false;
    }

  return S_ISDIR(statbuf.st_mode);
}

// ######################################################################
bool isDirectory(const struct dirent* dirp)
{
  if (dirp == 0)
    return false;

#ifdef HAVE_STRUCT_DIRENT_D_TYPE
  return dirp->d_type == DT_DIR;
#else
  return isDirectory(dirp->d_name);
#endif
}

// ######################################################################
ushort lockFile(const int fd, struct flock &fl)
{
  if (fd < 0)
  {
    LFATAL("Attempt to lock an unopened file!");
  }
  else
  {
    struct flock fl;
    fl.l_type   = F_WRLCK;
    fl.l_whence = SEEK_SET;
    fl.l_start  = 0;
    fl.l_len    = 0;
    if (fcntl(fd, F_SETLK, &fl) == -1)
    {
      return errno;
    }
  }
  return 0;
}

// ######################################################################
ushort unlockFile(const int fd, struct flock &fl)
{
  fl.l_type   = F_UNLCK;
  fl.l_whence = SEEK_SET;
  fl.l_start  = 0;
  fl.l_len    = 0;
  if (fcntl(fd, F_SETLK, &fl) == -1)
  {
    return 1;
  }

  return 0;
}

// ######################################################################
ushort lockFile(const std::string fileName, int &fd, struct flock &fl)
{
  fd = open(fileName.c_str(), O_RDWR);
  return lockFile(fd,fl);
}

// ######################################################################
ushort unlockFile(const std::string fileName, const int fd, struct flock &fl)
{
  const ushort err = unlockFile(fd,fl);
  close(fd);
  return err;
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_FILEUTIL_C_DEFINED
