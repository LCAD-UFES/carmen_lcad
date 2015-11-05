/** @file rutz/pipe.h wrap posix inter-process pipes in a c++
    iostreams interface */

///////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2000-2004 California Institute of Technology
// Copyright (c) 2004-2007 University of Southern California
// Rob Peters <rjpeters at usc dot edu>
//
// created: Fri Jan 14 17:33:24 2000
// commit: $Id: pipe.h 11238 2009-05-28 01:05:46Z jshen $
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/pipe.h $
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

#ifndef GROOVX_RUTZ_PIPE_H_UTC20050626084019_DEFINED
#define GROOVX_RUTZ_PIPE_H_UTC20050626084019_DEFINED

#include "rutz/stdiobuf.h"
#include "nub/object.h"
#include <cstdio>
#include <istream>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

namespace rutz
{
  class shell_pipe;
  class pipe_fds;
  class child_process;
  class exec_pipe;
  class bidir_pipe;
}

/// Adapts UNIX-style process pipes to a std::iostream interface.
class rutz::shell_pipe
{
public:
  shell_pipe(const char* command, const char* mode);

  ~shell_pipe();

  std::iostream& stream() { return m_stream; }

  int close();

  bool is_closed() const { return (m_file == 0) || !m_stream.is_open(); }

  int exit_status() const { return m_exit_status; }

private:
  shell_pipe(const shell_pipe&);
  shell_pipe& operator=(const shell_pipe&);

  FILE*             m_file;
  rutz::stdiostream m_stream;
  int               m_exit_status;
};

/// An exception-safe wrapper around a pair of file descriptors from pipe().
class rutz::pipe_fds
{
public:
  /// Create a pipe pair of file descriptors.
  /** Throws an exception if the pipe() call fails. */
  pipe_fds();

  /// Destructor closes both file descriptors.
  ~pipe_fds() throw();

  int reader() const throw() { return m_fds[0]; }
  int writer() const throw() { return m_fds[1]; }

  void close_reader() throw() { if (m_fds[0] >= 0) close(m_fds[0]); m_fds[0] = -1; }
  void close_writer() throw() { if (m_fds[1] >= 0) close(m_fds[1]); m_fds[1] = -1; }

private:
  pipe_fds(const pipe_fds&);
  pipe_fds& operator=(const pipe_fds&);

  int m_fds[2]; // reading == m_fds[0], writing == m_fds[1]
};

/// An exception-safe wrapper around a child process from fork().
class rutz::child_process
{
public:
  /// Fork a child process.
  /** Throws an exception if the fork() call fails. */
  child_process();

  /// Destructor waits for child process to complete.
  ~child_process() throw();

  /// Check if we're in the parent or child process after the fork().
  bool in_parent() const throw() { return m_pid != 0; }

  /// Wait for child process to complete, and return its status code.
  int wait() throw();

private:
  child_process(const child_process&);
  child_process& operator=(const child_process&);

  int m_child_status;
  pid_t m_pid;
};


/// An exception-safe wrapper around a pipe-fork-exec sequence.
class rutz::exec_pipe : public nub::object
{
private:
  void init(char* const* argv);

public:
  /// Set up a pipe to a child process with the given argv array.
  /** The mode should be "r" if the parent is reading, and "w" if the
      parent is writing. NOTE that the argv array MUST be
      NULL-terminated! */
  exec_pipe(const char* m, char* const* argv);

  /// Set up a pipe to a child process with the given list of args.
  /** The mode should be "r" if the parent is reading, and "w" if the
      parent is writing. NOTE that the variable length argument list
      MUST NULL-terminated! */
  exec_pipe(const char* m, const char* argv0, ...);

  /// Destructor cleans up child process and the pipe's file descriptors.
  ~exec_pipe() throw();

  /// Get the stream that is linked to the child process.
  std::iostream& stream() throw();

  /// Close the underlying stream AND the underlying file descriptor.
  void close();

  /// Wait for child process to complete, return 0 if all is OK, -1 if error.
  int exit_status() throw();

private:
  bool               m_parent_is_reader;
  pipe_fds           m_fds;
  child_process      m_child;
  rutz::stdiostream* m_stream;
};


/// An exception-safe wrapper around a pipe-fork-exec sequence.
class rutz::bidir_pipe
{
public:
  /// Default construct; you MUST call init() before using any of the streams!
  bidir_pipe();

  /// Set up a pipe to a child process with the given argv array.
  /** BE SURE the argv array is NULL-terminated! */
  bidir_pipe(char* const* argv);

  /// Set up a pipe to a child process with the given list of args.
  /** BE SURE the variable-length argument list is NULL-terminated!

      A private copy will be made of all the arguments, so it is safe
      to pass const strings here (such as the result of str.c_str()
      for a std::string object).
  */
  bidir_pipe(const char* argv0, ...);

  /// Destructor cleans up child process and the pipe's file descriptors.
  ~bidir_pipe() throw();

  /// Request that SIGINT be ignored in the child process.
  /** You must request this BEFORE calling init(); so the proper
      sequence is:

      \code
      rutz::bidir_pipe prog;
      prog.block_child_signals();
      prog.init("progname", "arg1", "arg2", NULL);
      \endcode

      This functionality is useful if the parent process is already
      handling SIGINT specially; if you don't block SIGINT in the
      child process, then when the user presses Ctrl-C, the following
      happens: (1) the main parent handles the SIGINT, as desired, but
      (2) the child process also receives a SIGINT, thus it dies, and
      (3) the parent process then gets a SIGPIPE and dies itself
      without getting a chance for a clean exit.
  */
  void block_child_sigint();

  /// Core code for starting the child process.
  /** NOTE the argv array must be NULL-terminated! */
  void init(char* const* argv);

  /// Core code for starting the child process.
  /** BE SURE the variable-length argument list is NULL-terminated! */
  void init(const char* argv0, ...);

  /// Get the stream that is receiving input from the child process.
  std::iostream& in_stream() throw();

  /// Get the stream that is sending output to the child process.
  std::iostream& out_stream() throw();

  /// Close the underlying input stream file descriptor.
  void close_in();

  /// Close the underlying output stream file descriptor.
  void close_out();

  /// Wait for child process to complete, return 0 if all is OK, -1 if error.
  int exit_status() throw();

private:
  bidir_pipe(const bidir_pipe&); // not implemented
  bidir_pipe& operator=(const bidir_pipe&); // not implemented

  pipe_fds           m_in_pipe;
  pipe_fds           m_out_pipe;
  child_process      m_child;
  rutz::stdiostream* m_in_stream;
  rutz::stdiostream* m_out_stream;
  bool               m_block_child_sigint;
};

static const char __attribute__((used)) vcid_groovx_rutz_pipe_h_utc20050626084019[] = "$Id: pipe.h 11238 2009-05-28 01:05:46Z jshen $ $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/rutz/pipe.h $";
#endif // !GROOVX_RUTZ_PIPE_H_UTC20050626084019_DEFINED
