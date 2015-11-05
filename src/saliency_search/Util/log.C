/*!@file Util/log.C Interface to syslog */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/log.C $
// $Id: log.C 11908 2009-11-01 19:34:27Z lior $
//

#include "Util/log.H"

#include "rutz/compat_snprintf.h"
#include "rutz/demangle.h"
#include "rutz/error_context.h"

#include <cerrno>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <stdexcept> // for std::runtime_error
#include <sys/types.h>
#include <unistd.h>
#include <cstring>

//! total max size of a log entry
#define LBS  1024

int LOG_FLAGS = (LFATAL_THROWS
                 | LFATAL_PRINTS_ABORT
                 | LFATAL_XTRA_NOISY
                 | LOG_FULLTRACE);

//int LOG_FLAGS = (LFATAL_THROWS
//                 | LFATAL_PRINTS_ABORT
//                 | LFATAL_XTRA_NOISY);

int MYLOGVERB = LOG_INFO;

lfatal_exception::lfatal_exception() throw() {}

lfatal_exception::~lfatal_exception() throw() {}

const char* MYLOGPREFIX = 0;

namespace
{
  /// Concrete type thrown from LFATAL
  class real_lfatal_exception : public lfatal_exception
  {
  public:
    real_lfatal_exception(const std::string& msg)
      :
      lfatal_exception(),
      m_msg(msg)
    {}

    virtual ~real_lfatal_exception() throw() {}

    virtual const char* what() const throw()
    {
      return m_msg.c_str();
    }

    std::string m_msg;
  };

  /// A little helper function for sendlog().
  /** This avoids duplicating the flag-parsing logic between our two
      versions of sendlog(). */
  void sendlog_aux(const char* raw_msg, const int level, const int flags)
  {
    const char* msg = raw_msg;

    std::string full_msg;

    if (level == LOG_CRIT)
      {
        // if we have an error context message, prepend it to the
        // actual error message:

        full_msg = raw_msg;
        rutz::error_context::current().prepend_to(full_msg);
        msg = full_msg.c_str();
      }

    if (flags & LOG_FULLTRACE)
      {
        // if this was an LFATAL and we're going to throw an exception
        // object containing the logmsg, then don't print it to
        // stderr/syslog unless we also have LFATAL_XTRA_NOISY:
        const bool skip_printing =
          (level == LOG_CRIT
           && (flags & LFATAL_THROWS)
           && !(flags & LFATAL_XTRA_NOISY));

        if (!skip_printing)
          {
            if (MYLOGPREFIX != 0)
              fprintf(stderr, "[%s] %s\n", MYLOGPREFIX, msg);
            else
              fprintf(stderr, "%s\n", msg);
            fflush(stderr);
          }
      }
    else
      syslog(level, "%s", msg);

    // special handling if this was an LFATAL:
    if (level == LOG_CRIT)
      {
        if (flags & LFATAL_PRINTS_ABORT)
          fprintf(stderr, "-- ABORT.\n");

        if (flags & LFATAL_THROWS)
          throw real_lfatal_exception(full_msg);

        abort();
      }
  }
}

// ######################################################################
// for snprintf and vsnprintf, you need to define _GNU_SOURCE in your CFLAGS.
// define fake snprintf and vsnprintf if they do not exist in the system:
#ifndef _GNU_SOURCE
int snprintf(char *str, size_t n, const char *format, ...)
{ va_list a; va_start(a, format); vsprintf(str, format, a); va_end(a);
return strlen(str); }
int vsnprintf(char *str, size_t n, const char *format, va_list ap)
{ vsprintf(str, format, ap); return strlen(str); }
#endif

#ifdef __GNUC__
// ######################################################################
//! Send a log. GNUC, thread-safe version
void sendlog(const int lev, const int line, const char *file,
             const char *func, const int idval, const bool useid,
             const bool usep, const char *msg, ...)
{
  if (lev > MYLOGVERB) return; // ignore if insuffic priority

  va_list args; int len, len2; va_start(args, msg); char logbuf[LBS];

  // remove filename extension, if any, to get class name:
  char fil[strlen(file) + 1]; strcpy(fil, file);
  for (int i = strlen(fil)-1; i >= 0; i--)
    if (fil[i] == '.') { fil[i] = '\0'; break; }

  char* fil2 = fil;

  // also skip any subdirectories that may be in the file name, but if
  // the first character is a '[' then it's not a filename but a
  // descriptiveName() from e.g. CLINFO():
  if (fil2[0] != '[')
    {
      fil2 = strrchr(fil, '/');
      if (fil2) ++fil2; // get to the next char after the last /
      else fil2 = fil;  // there was no /, use whole file name
    }

  // print first chunk: file and function
  len = snprintf(logbuf, LBS, "%s::%s", fil2, func);
  if (len == -1 || len >= LBS-1) goto done;  // buffer is already full...

#ifdef DEBUG
  // if debug mode, add line number:
  len2 = snprintf(&(logbuf[len]), LBS-len, "(%d)", line);
  if (len2 == -1 || len2+len >= LBS-1) goto done; else len += len2;
#endif
  logbuf[len++] = ':'; if (len >= LBS-1) goto done;

  // if we use ID-logging, print the ID now:
  if (useid) {
    len2 = snprintf(&(logbuf[len]), LBS-len, "(%d)", idval);
    if (len2 == -1 || len2+len >= LBS-1) goto done; else len += len2;
  }

  // if msg starts with opening bracket, do not add a space:
  if (msg[0] != '(' && msg[0] != '[' && msg[0] != '{')
    { logbuf[len++] = ' '; if (len >= LBS-1) goto done; }

  // print the actual message:
  len2 = vsnprintf(&(logbuf[len]), LBS-len, msg, args);
  if (len2 == -1 || len2+len >= LBS-1) goto done; else len += len2;

  // if doing P-logging, add errno in clear:
  if (usep)
    snprintf( &(logbuf[len]), LBS-len, " (errno=%d, %s)",
              errno, strerror(errno));

  // ready to send to syslog:
 done:
  va_end(args);

  logbuf[LBS - 1] = '\0';

  sendlog_aux(logbuf, lev, LOG_FLAGS);
}

// ######################################################################
void showmemory(const int lev, const int line, const char *file,
                const char *func, const int idval, const bool useid,
                const bool usep, const char *msg, ...)
{
  if (lev > MYLOGVERB) return; // ignore if insufficient priority

  // crunch the var args:
  va_list args; va_start(args, msg); char logbuf[LBS + 100];
  vsnprintf(logbuf, LBS, msg, args); va_end(args);

  // get the memory usage:
  int pid = int(getpid());
  char xx[100]; snprintf(xx, 100, "/proc/%d/status", pid);
  FILE *f = fopen(xx, "r");
  if (f == NULL) { LERROR("Cannot open %s -- IGNORED", xx); return; }
  while(fgets(xx, 100, f))
    if (strncasecmp(xx, "VmSize", 6) == 0)
      {
        xx[strlen(xx) - 1] = '\0'; // eliminate trailing LF
        xx[7] = ' '; // replace TAB by a space
        strcat(logbuf, " [");
        strncat(logbuf, xx, 97);
        strcat(logbuf, "]");
        break;
      }
  fclose(f);
  sendlog(lev, line, file, func, idval, useid, usep, "%s", logbuf);
}

#else
// ######################################################################
//! Send a log. NON-GNUC, thread-unsafe version
void sendlog(const char *msg, ...)
{
  if (MYLOGLEV > MYLOGVERB) return; // ignore if insuffic priority

  va_list args; int len, len2; va_start(args, msg); char logbuf[LBS];

  // make sure strncpy did not leave us with unterminated strings:
  MYLOGFUN[LBS2-1] = '\0'; MYLOGFIL[LBS2-1] = '\0';

  // remove filename extension, if any, to get class name:
  for (int i = strlen(MYLOGFIL)-1; i >= 0; i--)
    if (MYLOGFIL[i] == '.') { MYLOGFIL[i] = '\0'; break; }

  // print first chunk: file and function
  len = snprintf(logbuf, LBS, "%s::%s", MYLOGFIL, MYLOGFUN);
  if (len == -1 || len >= LBS-1) goto done;  // buffer is already full...

#ifdef DEBUG
  // if debug mode, add line number:
  len2 = snprintf(&(logbuf[len]), LBS-len, "(%d)", MYLOGLIN);
  if (len2 == -1 || len2+len >= LBS-1) goto done; else len += len2;
#endif
  logbuf[len++] = ':'; if (len >= LBS-1) goto done;

  // if we use ID-logging, print the ID now:
  if (MYLOG_USEID) {
    len2 = snprintf(&(logbuf[len]), LBS-len, "(%d)", MYLOGIDVAL);
    if (len2 == -1 || len2+len >= LBS-1) goto done; else len += len2;
  }

  // if msg starts with opening bracket, do not add a space:
  if (msg[0] != '(' && msg[0] != '[' && msg[0] != '{')
    { logbuf[len++] = ' '; if (len >= LBS-1) goto done; }

  // print the actual message:
  len2 = vsnprintf(&(logbuf[len]), LBS-len, msg, args);
  if (len2 == -1 || len2+len >= LBS-1) goto done; else len += len2;

  // if doing P-logging, add errno in clear:
  if (MYLOG_USEP)
    snprintf( &(logbuf[len]), LBS-len, " (errno=%d, %s)",
              errno, strerror(errno));

  // ready to send to syslog:
 done:
  va_end(args);

  logbuf[LBS - 1] = '\0';

  sendlog_aux(logbuf, MYLOGLEV, LOG_FLAGS);
}


char MYLOGFUN[LBS2], MYLOGFIL[LBS2];
int MYLOGLIN, MYLOGLEV, MYLOGIDVAL;
bool MYLOG_USEP, MYLOG_USEID;

#endif

void report_current_exception(const int line, const char* file) throw()
{
  // rethrow the exception so we can recatch it and see if it is of a
  // known type:
  try { throw; }
  catch (std::exception& e)
    {
      fprintf(stderr, "%s:%d: exception caught (%s) with what():\n%s\n",
              file, line, rutz::demangled_name(typeid(e)), e.what());
      fflush(stderr);
    }
  catch (...)
    {
      fprintf(stderr, "%s:%d: caught an exception of unknown type\n",
              file, line);
      fflush(stderr);
    }
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
