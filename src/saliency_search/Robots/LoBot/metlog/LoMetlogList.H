/**
   \file  Robots/LoBot/misc/LoMetlogList.H
   \brief An object to store the list of metrics logs to be analyzed and
   provide thread-safe access to this list.

   This file defines a class that implements a simple object for storing
   the list of metrics logs to be analyzed and providing thread-safe
   access to this list.

   The Robolocust metrics log analyzer program enumerates the list of log
   files in the individual directories specified as command line
   arguments to it. Then, it creates multiple threads to load and parse
   all these log files in parallel. These loader threads need to know the
   next log file to load. To ensure that different threads pick different
   log files, we use the lobot::MetlogList object to keep track of the
   next one in the list that should be loaded.

   Obviously, since multiple threads can access this list and update the
   next field, we need to protect simultaneous accesses. The
   lobot::MetlogList object encapsulates all of that functionality,
   providing a straightforward interface to the loader threads.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoMetlogList.H $
// $Id: LoMetlogList.H 13923 2010-09-11 09:27:03Z mviswana $
//

#ifndef LOBOT_METLOG_LIST_DOT_H
#define LOBOT_METLOG_LIST_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoMutex.H"

// Standard C++ headers
#include <string>
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

/**
   \class lobot::MetlogList
   \brief An object to help keep track of which metrics log should be
   loaded next.

   This class implements a thread-safe interface for lobot::LoaderThread
   objects so they can figure out which metrics log should be loaded and
   parsed next. When all the metlogs have been processed, further
   attempts at retrieving the name of the next one for loading will
   result in an exception. The loader threads can use the exception as a
   signal to wind up their work.
*/
class MetlogList {
   // Prevent copy and assignment
   MetlogList(const MetlogList&) ;
   MetlogList& operator=(const MetlogList&) ;

   /// The whole idea behind this class is to keep track of a list of
   /// metrics log file names that need to be loaded and parsed. These
   /// data members take care of the details.
   //@{
   typedef std::vector<std::string> List ;
   typedef List::iterator Iter ;
   List m_list ;
   mutable Iter m_next ;
   //@}

   /// When a loader thread is done processing one metlog, it will
   /// request the next one in the queue from this object. Since multiple
   /// loader threads can use this object, we need a mutex to ensure that
   /// they don't step on each others' toes.
   Mutex m_mutex ;

public:
   /// When this object is created, it should be passed the list of names
   /// of metrics logs that have to be loaded.
   ///
   /// NOTE: It is important that the loader threads not yet be active
   /// when this class is instantiated.
   MetlogList(const std::vector<std::string>& metlog_list) ;

   /// This method retrieves the next name on the list. When all the
   /// names have been retrieved, it will throw an end-of-line exception
   /// to indicate to the loader threads that it's time for them to wind
   /// up their business.
   std::string next() const ;

   /// This is the exception object thrown when we reach the end of the
   /// list.
   struct eol {} ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
