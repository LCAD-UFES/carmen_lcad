/*!@file Util/csignals.H trivial wrapper for catching ANSI C signals */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/csignals.H $
// $Id: csignals.H 5685 2005-10-07 23:46:44Z rjpeters $
//

#ifndef UTIL_CSIGNALS_H_DEFINED
#define UTIL_CSIGNALS_H_DEFINED

/// Install signal handlers to catch non-fatal signals
/** @param dest Location where the number of a caught signal will be
    stored. This is marked as 'volatile' because it will be modified
    by signal handlers. Typically, applications will pass a pointer to
    a local int variable in to catchsignals(), and then check the
    value of that variable in each iteration through the main loop of
    the application, and taking steps to exit the application if the
    value becomes non-zero.

    Note that catchsignals() only installs handlers for non-fatal
    signals (such as SIGHUP, SIGINT [control-c], SIGTERM [kill] and
    SIGALRM). The idea is that the user can interrupt an application
    that would otherwise be in an perpetual loop, but still give the
    application a chance to shut down cleanly (flushing open files,
    computing final stats, etc.). However, we don't trap more serious
    signals such as SIGQUIT, SIGABRT, SIGBUS, SIGFPE, SIGKILL, or
    SIGSEGV -- these usually represent a programming bug and/or the
    desire to generate a core dump, so catching the signal would avoid
    a core dump and hinder the ability to debug the context in which
    the signal was generated.
*/
void catchsignals(volatile int* dest) throw();

/// Get a string name for a given signal number (e.g., "SIGINT", "SIGTERM")
const char* signame(int signum) throw();

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_CSIGNALS_H_DEFINED
