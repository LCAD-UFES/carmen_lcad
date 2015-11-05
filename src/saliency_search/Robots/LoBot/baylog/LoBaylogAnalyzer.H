/**
   \file  Robots/LoBot/control/LoBaylogAnalyzer.H
   \brief A class for analyzing the metrics logs for the Bayesian
   time-to-impact prediction experiments.

   This file defines a class that loads and parses all the Robolocust
   metrics logs for a dataset containing the logs for a Bayesian
   time-to-impact prediction experiment. The logs are analyzed and then
   the results for that dataset written out to a file.

   Each analyzer runs in its own thread. The lobay program uses this
   facility to process multiple datasets in parallel.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/baylog/LoBaylogAnalyzer.H $
// $Id: LoBaylogAnalyzer.H 14083 2010-09-30 13:59:37Z mviswana $
//

#ifndef LOBOT_BAYLOG_ANALYZER_DOT_H
#define LOBOT_BAYLOG_ANALYZER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoThread.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class DirList ;

/**
   \class lobot::BaylogAnalyzer
   \brief A class for loading and parsing the metrics logs for a Bayesian
   TTI prediction dataset in a separate thread.

   This class implements an interface for loading and parsing all the
   metrics logs for an entire Bayesian time-to-impact prediction dataset.
   Analyzing one such dataset is completely unrelated to analyzing
   another. Thus, multiple datasets can be processed in parallel.
   Consequently, this analyzer object does its thing in a separate
   thread.
*/
class BaylogAnalyzer : private Thread {
public:
   /// This class uses the "named constructor" idiom to ensure that
   /// clients not create instances of it on the stack. This method is
   /// the named constructor (aka factory method).
   ///
   /// When clients create these analyzers, they should supply a list of
   /// directories containing the datasets that need to be analyzed. Upon
   /// creation, analyzers will automatically launch a new thread to
   /// perform the dataset analysis and will use the client-supplied
   /// directory list to figure out which dataset is next in line for
   /// processing.
   ///
   /// NOTE: This named constructor is the only publically accessible
   /// thing in this class. Once an analyzer is created, the rest of its
   /// work proceeds automatically. Clients can either continue on with
   /// their business or choose to wait for all the analyzer threads to
   /// complete.
   static BaylogAnalyzer* create(const DirList&) ;

private:
   /// Private constructor because only the factory method or named
   /// constructor can be used to instantiate this class.
   BaylogAnalyzer(const DirList&) ;

   /// This member keeps track of the list of datasets to be loaded and
   /// which one is next in line. We can think of this list as the input
   /// to this analyzer class.
   const DirList& m_dirs ;

   // Since we're using private inheritance, Thread::start() won't be
   // visible to subclasses without this explicit say-so.
   using Thread::start ;

   /// This method implements the dataset analysis functionality. It
   /// executes in a separate thread.
   void run() ;

   // Prevent copy and assignment
   BaylogAnalyzer(const BaylogAnalyzer&) ;
   BaylogAnalyzer& operator=(const BaylogAnalyzer&) ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
