/**
   \file  Robots/LoBot/misc/LoDataset.H
   \brief An object to store the list of parsed metrics logs, or
   "experiments," and provide thread-safe access to this list along with
   a suitable API for analyzing the metrics.

   This file defines a class that implements an interface for storing the
   list of parsed metrics logs and providing thread-safe access to this
   list.

   The Robolocust metrics log dataset program enumerates the list of log
   files in the individual directories specified as command line
   arguments to it. Then, it creates multiple threads to load and parse
   all these log files in parallel. Once an individual log has been
   parsed, the loader thread that loaded it will need to know where to
   store the resulting lobot::Experiment object that encapsulates the
   parsed log.

   The lobot::Dataset class provides a data structure for storing the
   parsed experiments. Since the metlogs making up a dataset are all
   loaded in parallel, lobot::Dataset provides thread-safe API's to
   collect the parsing results as they become available.

   In addition to the above-mentioned collection API, the lobot::Dataset
   class also provides various functions that support the necessary data
   analysis on the metrics collected from the trajectory recording
   experiments.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoDataset.H $
// $Id: LoDataset.H 13934 2010-09-14 23:17:01Z mviswana $
//

#ifndef LOBOT_METRICS_DATASET_DOT_H
#define LOBOT_METRICS_DATASET_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoPointTypes.H"
#include "Robots/LoBot/thread/LoMutex.H"

// Standard C++ headers
#include <vector>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class Experiment ;

/**
   \class lobot::Dataset
   \brief A thread-safe container for storing parsed metlogs and
   performing various operations on them.

   Multiple instances of the lobot::MetlogLoader class are used to load
   and parse an LGMD trajectory dataset's individual metlogs in parallel.
   Each parsed metlog is stored in a lobot::Experiment object. The
   lobot::Dataset class is used to collect all the parsed metlogs, i.e.,
   experiments, and put them in a thread-safe container.

   Additionally, this class also provides various API's to support the
   necessary number crunching operations on the experiments to aid in
   their analysis.

   NOTE: The lobot::MetlogLoader class is responsible for creating the
   individual lobot::Experiment objects corresponding to each of the
   metlogs in a dataset. However, once it is done parsing the metlog and
   filling in the data in the Experiment object, it will hand the object
   over to the Dataset, which will then take over "ownership" of the
   Experiment. That is, modules that create Experiment objects should not
   delete those objects if they hand them over to a Dataset.
*/
class Dataset {
   // Prevent copy and assignment
   Dataset(const Dataset&) ;
   Dataset& operator=(const Dataset&) ;

   /// The whole idea behind this class is to keep track of a list of
   /// parsed metrics logs, i.e., "experiments."
   //@{
   typedef std::vector<Experiment*> List ;
   mutable List m_list ;
   mutable List::const_iterator m_next ;
   //@}

   /// When a loader thread is done processing one metlog, it will add
   /// the resulting Experiment object to an instance of this class.
   /// Since multiple loader threads can use this object, we need a mutex
   /// to ensure that they don't step on each others' toes.
   ///
   /// We use the same mutex when we're dealing with correspondence
   /// finder threads, which need to retrieve the experiments one-by-one.
   Mutex m_mutex ;

public:
   /// Constructor.
   Dataset() ;

   /// This method adds a parsed metlog, i.e., an Experiment, to its
   /// internal data structure. It is thread-safe and is meant to be used
   /// by lobot::MetlogLoader.
   void add(Experiment*) ;

   /// This function returns the number of experiments currently part of
   /// the dataset.
   int size() const {return m_list.size() ;}

   /// Check if the dataset is empty.
   bool empty() const {return m_list.empty() ;}

   /// This method returns the "reference experiment" for different
   /// criteria.
   ///
   /// Each dataset consists of several experiments. Each experiment
   /// contains a variety of point lists. One of these point lists
   /// records the robot's trajectory from start location to goal;
   /// another records the points at which the robot's LGMD-based
   /// extrication behaviour kicked in; another marks the locations where
   /// the robot's avoidance algorithms failed and made it bump into an
   /// obstacle; so on and so forth.
   ///
   /// To make this discussion a little more concrete and clear, let us
   /// say we are interested in computing the average trajectory from
   /// start to goal across all the experiments in a dataset. Each
   /// experiment will have a different number of points recorded in its
   /// trajectory list. To find the average path, we will have to somehow
   /// "normalize" all the recorded trajectories so that they all have
   /// the same number of points.
   ///
   /// To effect this "normalization," we find the trajectory containing
   /// the number of points that is the median of all the trajectories.
   /// The corresponding experiment is regarded as the "reference" and
   /// all average computations are made w.r.t. to this reference.
   ///
   /// This method returns the reference experiment for the specified
   /// point list. The return value is a const pointer to an Experiment
   /// object contained in the dataset. Clients should consider this
   /// object read-only, i.e., they should refrain from casting away the
   /// constness of the returned pointer and changing the returned object
   /// in any way. Clients should also not delete the pointer returned by
   /// this function.
   ///
   /// An invocation of this function prior to the addition of any parsed
   /// experiment objects to the dataset will result in a
   /// lobot::misc_error exception with the LOGIC_ERROR code.
   ///
   /// WARNING: This method alters the dataset object's internal state.
   /// It is most definitely not thread-safe. It is meant to be called by
   /// the lomet program's main thread and not from one of its helper
   /// threads.
   const Experiment* find_refexp(PointListName) const ;

   /// This method returns the next experiment that needs to be
   /// processed. It is meant to be used by the correspondence finder
   /// threads.
   const Experiment* next() const ;

   /// Since an experiment contains several different point lists, full
   /// analysis of a dataset requires invocation of the point
   /// correspondence finder procedure several times. Before we start the
   /// multithreaded correspondence finding procedure, we should first
   /// rewind the dataset so that the correspondence finder threads start
   /// off processing from the first experiment in the dataset. This
   /// method implements the dataset rewinding described above.
   ///
   /// This method is meant to be used by the lomet main program right
   /// before it launches the multiple correspondence finder threads.
   /// Note that it is not thread-safe and should not be called from any
   /// of lomet's helper threads.
   ///
   /// WARNING: If this function is called before any experiments have
   /// been added to the dataset, it will throw a lobot::misc_error with
   /// LOGIC_ERROR as the code.
   void rewind() ;

   /// This is the exception object thrown when we reach the end of the
   /// experiment list. This exception serves as the signal to the
   /// correspondence finder threads that all the experiments have been
   /// processed.
   struct eol {} ;

   /// Clean-up.
   ~Dataset() ;

   /// Debug support: dump each of the experiments to see what data they
   /// contain and allow users to verify whether that matches the
   /// original log files properly.
   void dump() const ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
