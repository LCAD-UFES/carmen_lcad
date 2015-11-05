/**
   \file  Robots/LoBot/control/LoMetlogLoader.H
   \brief A class for loading and parsing Robolocust metrics logs.

   This file defines a class that loads and parses a Robolocust metrics
   log. Since a trajectory experiment's dataset consists of 25 (or more)
   individual log files, this loader is designed to run in a separate
   thread. The lomet program's main thread can use this facility to
   create multiple loader objects and have them all load and parse the
   dataset's individual logs in parallel.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoMetlogLoader.H $
// $Id: LoMetlogLoader.H 13930 2010-09-13 23:53:53Z mviswana $
//

#ifndef LOBOT_METLOG_LOADER_DOT_H
#define LOBOT_METLOG_LOADER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/thread/LoThread.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class MetlogList ;
class Dataset ;

/**
   \class lobot::MetlogLoader
   \brief A class for loading and parsing Robolocust metrics logs in a
   separate thread.

   This class implements an interface for loading and parsing Robolocust
   metrics logs. Since 25 (or more) logs comprise one trajectory-related
   dataset for measuring the performance of an LGMD-based obstacle
   avoidance algorithm, this class is designed to load logs in a separate
   thread. Thus, the lomet program's main thread can launch multiple
   threads to load all the datasets in parallel.
*/
class MetlogLoader : private Thread {
public:
   /// This class uses the "named constructor" idiom to ensure that
   /// clients not create instances of it on the stack. This method is
   /// the named constructor (aka factory method).
   ///
   /// When clients create metlog loaders, they should supply a list of
   /// logs that need to be loaded. Upon creation, loaders will
   /// automatically launch a new thread to perform the loading and
   /// parsing operations and will use this list to figure out which log
   /// is next in line for processing.
   ///
   /// Additionally, the loader object also requires a Dataset instance
   /// to which it will add each parsed metlog.
   ///
   /// NOTE: This named constructor is the only publically accessible
   /// thing in this class. Once a loader is created, the rest of its
   /// work proceeds automatically. Clients can either continue on with
   /// their business or choose to wait for all the loader threads to
   /// complete.
   static MetlogLoader* create(const MetlogList&, Dataset*) ;

private:
   /// Private constructor because only the factory method or named
   /// constructor can be used to instantiate this class.
   MetlogLoader(const MetlogList&, Dataset*) ;

   /// This member keeps track of the list of logs to be loaded and which
   /// one is next in line. We can think of this list as the input to
   /// this loader class.
   const MetlogList& m_logs ;

   /// This member references the dataset object used to collect all the
   /// parsed logs. We can think of this object as the output of this
   /// class.
   Dataset* m_dataset ;

   // Since we're using private inheritance, Thread::start() won't be
   // visible to subclasses without this explicit say-so.
   using Thread::start ;

   /// This method implements the metlog loading and parsing
   /// functionality. It executes in a separate thread.
   void run() ;

   // Prevent copy and assignment
   MetlogLoader(const MetlogLoader&) ;
   MetlogLoader& operator=(const MetlogLoader&) ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
