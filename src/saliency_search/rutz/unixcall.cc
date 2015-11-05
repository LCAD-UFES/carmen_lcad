/** @file rutz/unixcall.cc thin wrappers around unix system calls */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 1999-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Wed Nov 17 15:05:41 1999
// commit: $Id: unixcall.cc 14376 2011-01-11 02:44:34Z pez $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/unixcall.cc $
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

#ifndef GROOVX_RUTZ_UNIXCALL_CC_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_UNIXCALL_CC_UTC20050626084019_DEFINED

#include <cctype>
#include "unixcall.h"

#include "rutz/arrays.h"
#include "rutz/error.h"
#include "rutz/fstring.h"
#include "rutz/sfmt.h"

#include <cerrno> // for ::errno
#include <cstdio> // for ::rename(), ::remove()
#include <cstdlib> // for ::atoi()
#include <cstring> // for ::strerror()
#include <dirent.h> // for ::opendir(), ::readdir()
#include <sys/stat.h> // for ::chmod(), ::stat()
#include <unistd.h> // for ::getcwd() (POSIX)

#include "rutz/trace.h"
#include "rutz/debug.h"
GVX_DBG_REGISTER

namespace
{
  void throwErrno(const char* where, const rutz::file_pos& pos)
  {
    throw rutz::error(rutz::sfmt("in \"%s\": %s", where,
                                 ::strerror(errno)), pos);
  }

  class ErrnoSaver
  {
  public:
    ErrnoSaver() : saved(errno)
    {
      errno = 0;
    }

    ~ErrnoSaver()
    {
      errno = saved;
    }

    const int saved;
  };
}

void rutz::unixcall::chmod(const char* path, mode_t mode)
{
GVX_TRACE("rutz::unixcall::chmod");

  ErrnoSaver saver;

  if ( ::chmod(path, mode) != 0 )
    throwErrno("chmod", SRC_POS);
}

void rutz::unixcall::rename(const char* oldpath, const char* newpath)
{
GVX_TRACE("rutz::unixcall::rename");

  ErrnoSaver saver;

  if ( ::rename(oldpath, newpath) != 0 )
    throwErrno("rename", SRC_POS);
}

void rutz::unixcall::remove(const char* pathname)
{
GVX_TRACE("rutz::unixcall::remove");

  errno = 0;

  if ( ::remove(pathname) != 0 )
    throwErrno("rutz::unixcall::remove", SRC_POS);
}

rutz::fstring rutz::unixcall::getcwd()
{
GVX_TRACE("rutz::unixcall::getcwd");

  ErrnoSaver saver;

  const int INIT_SIZE = 256;
  rutz::dynamic_block<char> buf(INIT_SIZE);

  while ( ::getcwd(&buf[0], buf.size()) == 0 )
    {
      if (errno == ERANGE)
        {
          errno = 0;
          buf.resize(buf.size() * 2);
        }
      else
        {
          throwErrno("getcwd", SRC_POS);
        }
    }

  return rutz::fstring(&buf[0]);
}

pid_t rutz::unixcall::get_file_user_pid(const char* fname)
{
GVX_TRACE("rutz::unixcall::get_file_user_pid");
  DIR* const proc_dir = opendir("/proc");

  if (proc_dir == 0)
    return 0;

  struct stat target_statbuf;
  if (stat(fname, &target_statbuf) != 0)
    return 0;

  const pid_t my_pid = getpid();

  struct dirent* proc_dent = 0;

  while ((proc_dent = readdir(proc_dir)) != 0)
    {
      if (proc_dent == 0)
        break;

      if (!isdigit(proc_dent->d_name[0]))
        continue;

      const pid_t pid = atoi(proc_dent->d_name);

      if (pid == my_pid)
        continue;

      const rutz::fstring fd_dirname =
        rutz::sfmt("/proc/%s/fd", proc_dent->d_name);

      DIR* const fd_dir = opendir(fd_dirname.c_str());

      if (fd_dir == 0)
        continue;

      struct dirent* fd_dent = 0;

      while ((fd_dent = readdir(fd_dir)) != 0)
        {
          if (!isdigit(fd_dent->d_name[0]))
            continue;

          const rutz::fstring fd_fname =
            rutz::sfmt("%s/%s", fd_dirname.c_str(), fd_dent->d_name);

          struct stat fd_statbuf;
          if (stat(fd_fname.c_str(), &fd_statbuf) != 0)
            continue;

          if (fd_statbuf.st_dev == target_statbuf.st_dev &&
              fd_statbuf.st_ino == target_statbuf.st_ino)
            return pid;
        }

      closedir(fd_dir);
    }

  closedir(proc_dir);

  return 0;
}

static const char __attribute__((used)) vcid_groovx_rutz_unixcall_cc_utc20050626084019[] = "$Id: unixcall.cc 14376 2011-01-11 02:44:34Z pez $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/unixcall.cc $";
#endif // !GROOVX_RUTZ_UNIXCALL_CC_UTC20050626084019_DEFINED
