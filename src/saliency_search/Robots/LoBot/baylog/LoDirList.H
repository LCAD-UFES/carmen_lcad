/**
   \file  Robots/LoBot/misc/LoDirList.H
   \brief A thread-safe object for storing the list of directories
   containing metrics logs for the Bayesian TTI prediction experiments.

   This file defines a class that implements a simple object for storing
   the list of directories containing metrics logs for the Bayesian
   time-to-impact experiments and providing thread-safe access to this
   directory list.

   The lobay program enumerates the list of subdirectories under the
   Bayesian TTI experiments' root data directory and creates multiple
   threads to process each of these datasets in parallel. The analyzer
   threads need to know the next directory to process. To ensure that
   different threads pick different datasets, we use the lobot::DirList
   object to keep track of the next one in the list that should be
   analyzed.

   Obviously, since multiple threads can access this list and update the
   next field, we need to protect simultaneous accesses. The
   lobot::DirList object encapsulates all of that functionality,
   providing a straightforward interface to the analyzer threads.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/baylog/LoDirList.H $
// $Id: LoDirList.H 14083 2010-09-30 13:59:37Z mviswana $
//

#ifndef LOBOT_BAYLOG_DIR_LIST_DOT_H
#define LOBOT_BAYLOG_DIR_LIST_DOT_H

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
   \class lobot::DirList
   \brief An object to help keep track of which dataset should be
   analyzed next.

   This class implements a thread-safe interface for the
   lobot::BaylogAnalyzer threads so they can figure out which Bayesian
   TTI prediction experiment's dataset should be loaded and parsed next.
   When all the datasets have been processed, further attempts at
   retrieving the name of the next one for analysis will result in an
   exception. The analyzer threads can use the exception as a signal to
   wind up their work.
*/
class DirList {
   // Prevent copy and assignment
   DirList(const DirList&) ;
   DirList& operator=(const DirList&) ;

   /// The whole idea behind this class is to keep track of a list of
   /// directories each of which contains metrics logs collected as part
   /// of the Robolocust Bayesian time-to-impact prediction experiments.
   /// These data members take care of the details.
   //@{
   typedef std::vector<std::string> List ;
   typedef List::iterator Iter ;
   List m_list ;
   mutable Iter m_next ;
   //@}

   /// When an analyzer thread is done processing one log, it will
   /// request the next one in the queue from this object. Since multiple
   /// threads can use this object, we need a mutex to ensure that they
   /// don't step on each others' toes.
   Mutex m_mutex ;

public:
   /// When this object is created, it should be passed the list of names
   /// of directories containing the relevant logs.
   ///
   /// NOTE: It is important that the analyzer threads not yet be active
   /// when this class is instantiated.
   DirList(const std::vector<std::string>& dirs) ;

   /// This method returns the number of directories held by this list
   /// object.
   int size() const {return m_list.size() ;}

   /// This method retrieves the next name on the list. When all the
   /// names have been retrieved, it will throw an end-of-list exception
   /// to indicate to the analyzer threads that it's time for them to
   /// wind up their business.
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
