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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/log.H $
// $Id: log.H 11019 2009-03-11 20:57:11Z itti $
//

#ifndef LOG_H_DEFINED
#define LOG_H_DEFINED

#include <exception>
#include <syslog.h>
#include <string.h>

/*! \file Util/log.H
    \brief Logging facility

    This is a comprehensive logging facility.  Use the functions
    provided below to display messages to the user and/or syslog.
    Never use printf() or std::cout<< in your code, use the functions
    provided here instead.

    Basic function call is [ID | C][P]L<LOGLEV>("format", args, ...)

    - Log levels are: DEBUG, INFO, ERROR, FATAL.
    - Prepend 'L' to call the function for a given level.
    - Prepend 'P' to have strerror(errno) appended to msg.
    - Prepend 'ID' to have integer value MYLOGID inserted at start of
      msg (see examples), OR, prepend 'C' to have the ModelComponent
      descriptive name used instead of the file name (see below), if
      you are inside a ModelComponent. The 'C' version will not work
      in any place where a descriptiveName() function is not
      available. To minimize confusion, the 'C' version will put the
      descriptive name in square brackets, so that it may be easily
      distinguished from a file name.
    - '#define DEBUG' in 'log.C' to get line numbers with function name.

    Although macros, all functions are atomic, so 'if (bad)
    LERROR(bad);' is ok; also arguments that span several lines of
    source code are ok too. Using these functions in macros however is
    probably bogus: MACRO(LERROR("xx")). CAUTION: message size is
    limited to LBS chars (including final '\0'); longer messages will
    be cleanly truncated.

    All log functions act like printf, so be careful when passing
    strings from outside directly as the format string for these
    functions, as outside strings may contain '%' signs which could
    segfault your program!  so, use LERROR("%s", unknown_string)
    rather than LERROR(unknown_string).

    DEBUG, INFO and ERROR simply report an error. FATAL also throws an
    exception of class "bug" (or exits; see implementation).

Log entries will look like:

\verbatim

>>>>>>>>>>>>>>>>>>>> LERROR("success! %d %s", 10, "bozo")
Jan 11 07:21:26 jukebox dntwebcfgd: SockServ::check: success! 10 bozo
^^^^^^^^^^^^^^^ ^^^^^^^ ^^^^^^^^^^  ^^^^^^^^  ^^^^^  ^^^^^^^^^^^^^^^^
   date/time    machine   program    class    funct     message

>>>>>>>>>>>>>>>>>>>> PLERROR("Cannot open");
Jan 11 07:21:26 jukebox dntwebcfg: Cool::check: Cannot open (Bad descriptor)
^^^^^^^^^^^^^^^ ^^^^^^^ ^^^^^^^^^  ^^^^  ^^^^^  ^^^^^^^^^^^ ^^^^^^^^^^^^^^^^
  date/time    machine   program   class funct    message    errno in clear

>>>>>>>>>>>>>>>>>>>> IDLINFO("Connected");
with '#define MYLOGID fd' and current value of fd being 10
Jan 11 07:21:26 jukebox dntwebcfgd: Cool::check:(10) Connected
^^^^^^^^^^^^^^^ ^^^^^^^ ^^^^^^^^^^  ^^^^  ^^^^^  ^^  ^^^^^^^^^
   date/time    machine   program   class funct  ID   message

>>>>>>>>>>>>>>>>>>>> IDPLFATAL("Abort");
with '#define MYLOGID fd' and current value of fd being 10
Jan 11 07:21:26 jukebox dntwebcfgd: Cool::check:(10) Abort (Out of memory)
^^^^^^^^^^^^^^^ ^^^^^^^ ^^^^^^^^^^  ^^^^  ^^^^^  ^^  ^^^^^  ^^^^^^^^^^^^^
   date/time    machine   program   class funct  ID   mes   errno in clear

>>>>>>>>>>>>>>>>>>>> CLERROR("success! %d %s", 10, "bozo")
Jan 11 07:21:26 jukebox dntwebcfgd: [Gabor(45)]::check: success! 10 bozo
^^^^^^^^^^^^^^^ ^^^^^^^ ^^^^^^^^^^  ^^^^^^^^^^^  ^^^^^  ^^^^^^^^^^^^^^^^
   date/time    machine   program ModelComponent funct     message

\endverbatim

If the message starts with a '(', '[' or '{' char, no space is written
between the colon following function name and the message

A last macro is provided, SHOWMEMORY(format, ...) which will display
your message via LERROR, and append the total memory usage for the
current process (as per the VmSize field in /proc/<pid>/status; this
is useful to check for possible memory leaks). It only works under
Linux and hence we have only defined it in the GNU C section here.
*/


/// Make LFATAL() throw a std::exception rather than abort()ing.
/** This flag is on by default. */
const int LFATAL_THROWS       = (1 << 0);
/// Make LFATAL() print " -- ABORT" after each message.
/** This flag is on by default. */
const int LFATAL_PRINTS_ABORT = (1 << 1);
/// Make LFATAL() print its msg to stderr/syslog AND put it in the std::exception.
/** This flag is on by default. Without it, LFATAL will only print its
    message to stderr or syslog if it will NOT be throwing a
    std::exception with that same message in
    std::exception::what(). */
const int LFATAL_XTRA_NOISY   = (1 << 2);
/// Whether to send log messages to stderr or to syslog.
/*! Messages are sent to stderr if the LOG_FULLTRACE flag is set in
    LOG_FLAGS (this is the default), or are sent to syslog if the
    LOG_FULLTRACE flag is not set. Executables don't need to do
    anything special unless they need a non-default value. */
const int LOG_FULLTRACE       = (1 << 3);

/// Bitwise-or'ed combination of above flags.
extern int LOG_FLAGS;


/*! Log verbosity (see <syslog.h> for possible values). Default value is
  LOG_DEBUG. Executables don't need to do anything special unless they need
  a non-default value. */
extern int MYLOGVERB;

/// Null by default, but if set to non-null this string will be prefixed to every log message.
/*! This can be useful if you have multiple threads or multiple
    processes all writing to stdout, and you want to have some way to
    distinguish their log messages. */
extern const char* MYLOGPREFIX;

/// Type of exception that is thrown from LFATAL if (LOG_FLAGS&LFATAL_THROWS)
/** The real type of the exception is a derived type that is hidden
    from this header file -- that's so that we can hide potentially
    expensive things in the real implementation, like using
    std::string or doing some other fancy message formatting. */
class lfatal_exception : public std::exception
{
protected:
  lfatal_exception() throw();
  virtual ~lfatal_exception() throw();

public:
  virtual const char* what() const throw() = 0;
};

// ######################################################################
// If using GNU C, these vararg macros provide a thread-safe facility
// ######################################################################
#ifdef __GNUC__

#define LDEBUG(f, x...) sendlog(LOG_DEBUG,__LINE__,__FILE__,__FUNCTION__,\
                        0,false,false, f , ## x)
#define LINFO(f, x...)  sendlog(LOG_INFO,__LINE__,__FILE__,__FUNCTION__,\
                        0,false,false, f , ## x)
#define LERROR(f, x...) sendlog(LOG_ERR,__LINE__,__FILE__,__FUNCTION__,\
                        0,false,false,f ,## x)
#define LFATAL(f, x...) sendlog(LOG_CRIT,__LINE__,__FILE__,__FUNCTION__,\
                        0,false,false, f , ## x)

#define PLDEBUG(f, x...) sendlog(LOG_DEBUG,__LINE__,__FILE__,__FUNCTION__,\
                         0,false,true, f , ## x)
#define PLINFO(f, x...)  sendlog(LOG_INFO,__LINE__,__FILE__,__FUNCTION__,\
                         0,false,true, f , ## x)
#define PLERROR(f, x...) sendlog(LOG_ERR,__LINE__,__FILE__,__FUNCTION__,\
                         0,false,true, f , ## x)
#define PLFATAL(f, x...) sendlog(LOG_CRIT,__LINE__,__FILE__,__FUNCTION__,\
                         0,false,true, f , ## x)

#define IDLDEBUG(f, x...) sendlog(LOG_DEBUG,__LINE__,__FILE__,__FUNCTION__,\
                          MYLOGID,true,false, f , ## x)
#define IDLINFO(f, x...)  sendlog(LOG_INFO,__LINE__,__FILE__,__FUNCTION__,\
                          MYLOGID,true,false, f , ## x)
#define IDLERROR(f, x...) sendlog(LOG_ERR,__LINE__,__FILE__,__FUNCTION__,\
                          MYLOGID,true,false, f , ## x)
#define IDLFATAL(f, x...) sendlog(LOG_CRIT,__LINE__,__FILE__,__FUNCTION__,\
                          MYLOGID,true,false, f , ## x)

#define IDPLDEBUG(f, x...) sendlog(LOG_DEBUG,__LINE__,__FILE__,__FUNCTION__,\
                           MYLOGID,true,true, f , ## x)
#define IDPLINFO(f, x...)  sendlog(LOG_INFO,__LINE__,__FILE__,__FUNCTION__,\
                           MYLOGID,true,true, f , ## x)
#define IDPLERROR(f, x...) sendlog(LOG_ERR,__LINE__,__FILE__,__FUNCTION__,\
                           MYLOGID,true,true, f , ## x)
#define IDPLFATAL(f, x...) sendlog(LOG_CRIT,__LINE__,__FILE__,__FUNCTION__,\
                           MYLOGID,true,true, f , ## x)

#define CLOGFILE ("[" + realm() + ":" + descriptiveName() + "]").c_str()

#define CLDEBUG(f, x...) sendlog(LOG_DEBUG,__LINE__,CLOGFILE,__FUNCTION__,\
                        0,false,false, f , ## x)
#define CLINFO(f, x...)  sendlog(LOG_INFO,__LINE__,CLOGFILE,__FUNCTION__,\
                        0,false,false, f , ## x)
#define CLERROR(f, x...) sendlog(LOG_ERR,__LINE__,CLOGFILE,__FUNCTION__,\
                        0,false,false,f ,## x)
#define CLFATAL(f, x...) sendlog(LOG_CRIT,__LINE__,CLOGFILE,__FUNCTION__,\
                        0,false,false, f , ## x)

#define CPLDEBUG(f, x...) sendlog(LOG_DEBUG,__LINE__,CLOGFILE,__FUNCTION__,\
                         0,false,true, f , ## x)
#define CPLINFO(f, x...)  sendlog(LOG_INFO,__LINE__,CLOGFILE,__FUNCTION__,\
                         0,false,true, f , ## x)
#define CPLERROR(f, x...) sendlog(LOG_ERR,__LINE__,CLOGFILE,__FUNCTION__,\
                         0,false,true, f , ## x)
#define CPLFATAL(f, x...) sendlog(LOG_CRIT,__LINE__,CLOGFILE,__FUNCTION__,\
                         0,false,true, f , ## x)

#define SHOWMEMORY(f, x...) showmemory(LOG_ERR,__LINE__,__FILE__,__FUNCTION__,\
                        0,false,false,f ,## x)

// NOTE: the __attribute__ tells gcc that it should issue printf-style
// warnings when compiling calls to sendlog(), treating the 8th
// argument (msg) as the format string, and the 9th and subsequent
// arguments as the printf-style parameters

void sendlog(const int lev, const int line, const char *file,
             const char *func, const int idval, const bool useid,
             const bool usep, const char *msg, ...)
  __attribute__((format(__printf__,8,9)));

// NOTE: the __attribute__ tells gcc that it should issue printf-style
// warnings when compiling calls to sendlog(), treating the 8th
// argument (msg) as the format string, and the 9th and subsequent
// arguments as the printf-style parameters

void showmemory(const int lev, const int line, const char *file,
                const char *func, const int idval, const bool useid,
                const bool usep, const char *msg, ...)
  __attribute__((format(__printf__,8,9)));

#else
// ######################################################################
// If not using GNU C, these functions provide a thread-unsafe facility
// ######################################################################

//! max size of function or file name
#define LBS2 64

// log functions
#define SENDLOG      MYLOG_USEID=false, MYLOG_USEP=false, sendlog
#define SENDPLOG     MYLOG_USEID=false, MYLOG_USEP=true,  sendlog
#define SENDIDLOG    MYLOG_USEID=true,  MYLOG_USEP=false, sendlog
#define SENDIDPLOG   MYLOG_USEID=true,  MYLOG_USEP=true,  sendlog

#define LDEBUG MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_DEBUG,SENDLOG
#define LINFO MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_INFO,SENDLOG
#define LERROR MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_ERR,SENDLOG
#define LFATAL MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_CRIT,SENDLOG

#define PLDEBUG MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_DEBUG,SENDPLOG
#define PLINFO MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_INFO,SENDPLOG
#define PLERROR MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_ERR,SENDPLOG
#define PLFATAL MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_CRIT,SENDPLOG

#define IDLDEBUG MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_DEBUG,\
SENDIDLOG
#define IDLINFO MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_INFO,\
SENDIDLOG
#define IDLERROR MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_ERR,\
SENDIDLOG
#define IDLFATAL MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_CRIT,\
SENDIDLOG

#define IDPLDEBUG MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_DEBUG,\
SENDIDPLOG
#define IDPLINFO MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_INFO,\
SENDIDPLOG
#define IDPLERROR MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_ERR,\
SENDIDPLOG
#define IDPLFATAL MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
MYLOGIDVAL=MYLOGID,strncpy(MYLOGFIL, __FILE__, LBS2-1),MYLOGLEV=LOG_CRIT,\
SENDIDPLOG

#define CLOGFILE ("[" + descriptiveName() + "]").c_str()

#define CLDEBUG MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_DEBUG,SENDLOG
#define CLINFO MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_INFO,SENDLOG
#define CLERROR MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_ERR,SENDLOG
#define CLFATAL MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_CRIT,SENDLOG

#define CPLDEBUG MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_DEBUG,SENDPLOG
#define CPLINFO MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_INFO,SENDPLOG
#define CPLERROR MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_ERR,SENDPLOG
#define CPLFATAL MYLOGLIN=__LINE__,strncpy(MYLOGFUN, __FUNCTION__, LBS2-1),\
strncpy(MYLOGFIL, CLOGFILE, LBS2-1),MYLOGLEV=LOG_CRIT,SENDPLOG


// NOTE: the __attribute__ tells gcc that it should issue printf-style
// warnings when compiling calls to sendlog(), treating the 1st
// argument (msg) as the format string, and the 2nd and subsequent
// arguments as the printf-style parameters.
void sendlog(const char *msg, ...)
  __attribute__((format(__printf__, 1, 2)));

// defined in log.C:
extern char MYLOGFUN[], MYLOGFIL[];
extern int MYLOGLIN, MYLOGLEV, MYLOGIDVAL;
extern bool MYLOG_USEID, MYLOG_USEP;

#endif

/// Helper function for REPORT_CURRENT_EXCEPTION macro.
void report_current_exception(const int line, const char* file) throw();

/// Use this inside of a catch(...) to report info about the caught exception
/** There MUST be a current exception object in order for this to work
    (i.e., you must be inside of a catch block when you call
    REPORT_CURRENT_EXCEPTION). */
#define REPORT_CURRENT_EXCEPTION \
  report_current_exception(__LINE__, __FILE__)

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
