/*!@file Util/csignals.C trivial wrapper for catching ANSI C signals */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/csignals.C $
// $Id: csignals.C 7457 2006-11-20 17:43:13Z rjpeters $
//

#ifndef UTIL_CSIGNALS_C_DEFINED
#define UTIL_CSIGNALS_C_DEFINED

#include "Util/csignals.H"

#include <cstdio>
#include <signal.h>

#include "Util/log.H"

namespace
{
  volatile int* signum_return = 0;

  void handlesignal(int s) throw()
  {
    if (MYLOGPREFIX != 0)
      fprintf(stderr, "[%s] ****** %s ******\n", MYLOGPREFIX, signame(s));
    else
      fprintf(stderr, "****** %s ******\n", signame(s));

    if (signum_return != 0)
      *signum_return = s;
  }
}

void catchsignals(volatile int* dest) throw()
{
  // see comments in header file for why we only catch these
  // "non-fatal" signals and don't catch fatal signals such as
  // SIGQUIT, SIGABRT, SIGBUS, SIGFPE, SIGKILL, or SIGSEGV

  // before we install our signal handler, check first to see if the
  // signal already has a non-default handler, in which case we want
  // to leave that one as is -- e.g. if the user runs a program with
  // 'nohup', then SIGHUP will be set to SIG_IGN instead of SIG_DFL,
  // and in that case we want to leave it set to SIG_IGN rather than
  // overriding it to our signal handler which would defeat the
  // purpose of running with 'nohup' in the first place

#ifndef HAVE_SIGHANDLER_T
  typedef void (*handler)(int);
#else
  typedef sighandler_t handler;
#endif

#define INSTALL_SIG_HANDLER(s, h)               \
  do {                                          \
    handler old = signal(s, 0);                 \
    if (old == SIG_DFL) signal(s, h);           \
    else                signal(s, old);         \
  } while (0)

  INSTALL_SIG_HANDLER(SIGHUP,  &handlesignal);
  INSTALL_SIG_HANDLER(SIGINT,  &handlesignal);
  INSTALL_SIG_HANDLER(SIGTERM, &handlesignal);
  INSTALL_SIG_HANDLER(SIGALRM, &handlesignal);
  INSTALL_SIG_HANDLER(SIGPIPE, &handlesignal);

#undef INSTALL_SIG_HANDLER

  signum_return = dest;
}

const char* signame(int num) throw()
{
  switch (num)
    {
#define CASE_SIGNAL_NAME(x) case x: return #x

      CASE_SIGNAL_NAME(SIGHUP); break;
      CASE_SIGNAL_NAME(SIGINT); break;
      CASE_SIGNAL_NAME(SIGQUIT); break;
      CASE_SIGNAL_NAME(SIGILL); break;
      CASE_SIGNAL_NAME(SIGTRAP); break;
      CASE_SIGNAL_NAME(SIGABRT); break;
      CASE_SIGNAL_NAME(SIGBUS); break;
      CASE_SIGNAL_NAME(SIGFPE); break;
      CASE_SIGNAL_NAME(SIGKILL); break;
      CASE_SIGNAL_NAME(SIGUSR1); break;
      CASE_SIGNAL_NAME(SIGSEGV); break;
      CASE_SIGNAL_NAME(SIGUSR2); break;
      CASE_SIGNAL_NAME(SIGPIPE); break;
      CASE_SIGNAL_NAME(SIGALRM); break;
      CASE_SIGNAL_NAME(SIGTERM); break;
      CASE_SIGNAL_NAME(SIGCHLD); break;
      CASE_SIGNAL_NAME(SIGCONT); break;
      CASE_SIGNAL_NAME(SIGSTOP); break;
      CASE_SIGNAL_NAME(SIGTSTP); break;
      CASE_SIGNAL_NAME(SIGTTIN); break;
      CASE_SIGNAL_NAME(SIGTTOU); break;
      CASE_SIGNAL_NAME(SIGURG); break;
      CASE_SIGNAL_NAME(SIGXCPU); break;
      CASE_SIGNAL_NAME(SIGXFSZ); break;
      CASE_SIGNAL_NAME(SIGVTALRM); break;
      CASE_SIGNAL_NAME(SIGPROF); break;
      CASE_SIGNAL_NAME(SIGWINCH); break;
      CASE_SIGNAL_NAME(SIGIO); break;
      CASE_SIGNAL_NAME(SIGSYS); break;

#undef CASE_SIGNAL_NAME
    }

  return "unknown";
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_CSIGNALS_C_DEFINED
