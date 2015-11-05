/**
   \file  Robots/LoBot/control/LoCorrFinder.H
   \brief A class for finding point correspondences between the point
   lists of a reference experiment and another experiment.

   An Experiment contains several different lists of points. The point
   lists in Experiments across a Dataset will have different
   cardinalities. To compute average-case behaviour for the robot, we
   will have to first "normalize" the point list sizes so that they all
   have the same number of points as some reference Experiment. Then, for
   each point in the reference Experiment's point list, we will have to
   find the corresponding points in the other Experiments making up the
   Dataset.

   The point correspondences for one Experiment are independent of those
   of another Experiment. Therefore, we can compute the point
   correspondences for all the Experiments in a Dataset in parallel by
   running multiple threads.

   This file defines a class that encapsulates the multithreaded point
   correspondence finding procedure.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoCorrFinder.H $
// $Id: LoCorrFinder.H 13934 2010-09-14 23:17:01Z mviswana $
//

#ifndef LOBOT_CORRESPONDENCE_FINDER_DOT_H
#define LOBOT_CORRESPONDENCE_FINDER_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoPointTypes.H"
#include "Robots/LoBot/thread/LoThread.H"

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class Experiment ;
class Dataset ;
class PointMatrix ;

/**
   \class lobot::CorrFinder
   \brief A class for finding point correspondences between a
   lobot::Experiment and a refernce Experiment in a separate thread.

   This class implements an interface for launching a point
   correspondence finding algorithm in a separate thread. This
   correspondence finder's job is to look at a reference Experiment's
   point list of interest and find the closest points in another
   Experiment. The resulting list of points is added to a PointMatrix.
*/
class CorrFinder : private Thread {
public:
   /// This class uses the "named constructor" idiom to ensure that
   /// clients not create instances of it on the stack. This method is
   /// the named constructor (aka factory method).
   ///
   /// When clients create correspondence finders, they should supply a
   /// reference experiment and the dataset containing all the
   /// experiments. Additionally, the correspondence finder also needs to
   /// know the point list for which the client is interested in finding
   /// point correspondences.
   ///
   /// Upon instantiation, this class will automatically launch a new
   /// thread to perform the correspondence finding operations. Once it
   /// has found the desired correspondences, it will send its results to
   /// the point matrix supplied by the client.
   ///
   /// NOTE: This named constructor is the only publically accessible
   /// thing in this class. Once a correspondence finder object is
   /// created, the rest of its work proceeds automatically. Clients can
   /// either continue on with their business or choose to wait for all
   /// the correspondence finder threads to complete.
   static CorrFinder* create(const Experiment*, const Dataset&,
                             PointListName, PointMatrix*) ;

private:
   /// Private constructor because only the factory method or named
   /// constructor can be used to instantiate this class.
   CorrFinder(const Experiment*, const Dataset&, PointListName, PointMatrix*) ;

   /// These data members are the inputs to the correspondence finder.
   //@{
   const Experiment* m_refexp ;
   const Dataset&    m_dataset ;
   PointListName     m_point_list ;
   //@}

   /// This data member references the output object for the
   /// correspondence finder.
   PointMatrix* m_point_matrix ;

   // Since we're using private inheritance, Thread::start() won't be
   // visible to subclasses without this explicit say-so.
   using Thread::start ;

   /// This method implements the correspondence finding functionality.
   /// It executes in a separate thread.
   void run() ;

   // Prevent copy and assignment
   CorrFinder(const CorrFinder&) ;
   CorrFinder& operator=(const CorrFinder&) ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
