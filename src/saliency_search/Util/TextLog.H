/*!@file Util/TextLog.H Functions for logging model/simulation events */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Util/TextLog.H $
// $Id: TextLog.H 6463 2006-04-13 17:26:30Z rjpeters $
//

#ifndef UTIL_TEXTLOG_H_DEFINED
#define UTIL_TEXTLOG_H_DEFINED

#include <string>

class SimTime;

/*! @name Logging to a text file

    These functions may safely be called from multiple threads.
 */
//@{

//! Log a message to a textlog file
/*! This version is for C++ formatted message. If the option was not
  specified in the command line, this is a no-op. To get a
  std::string from printf-style formatting, use sformat() from
  "Util/sformat.H".

  @param event short one-word name for event that triggered log entry
  @param details additional details about the event
*/
void textLog(const std::string& fname,
             const std::string& event,
             const std::string& details);

//! Like before, but give a specific timestamp
void textLog(const std::string& fname,
             const std::string& event,
             const std::string& details,
             const SimTime& t);

//! Specify the timestamp to be used for subsequent logfile entries
void setLogTime(const std::string& fname,
                const SimTime& t);

//@}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // UTIL_TEXTLOG_H_DEFINED
