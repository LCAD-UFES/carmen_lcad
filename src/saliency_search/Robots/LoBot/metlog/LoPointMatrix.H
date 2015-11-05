/**
   \file  Robots/LoBot/misc/LoPointMatrix.H
   \brief A matrix for storing points and then computing an average point
   list.

   When we conduct an experiment to gauge lobot's performance, we collect
   several different lists of points: one to record the robot's
   trajectory from start to finish, another to record the locations where
   the robot's emergency stop behaviour was activated, another for the
   locations where the LGMD avoidance algorithm averted the robot away
   from an approaching obstacle; so on and so forth.

   Each experiment will produce point lists of different sizes. To
   compute the robot's average case behaviour, we will have to
   "normalize" these lists so that they all contain the same number of
   elements as some reference experiment. After we have discarded extra
   points or added missing points w.r.t. the reference experiment, we can
   find point correspondences across experiments and, finally, average
   these transformed lists to get the desired result.

   This file defines a class that is used to store the intermediate
   "normalized" point lists in a matrix. The columns of this matrix
   correspond to the individual experiments. The rows store corresponding
   points across experiments. For example, if we have 25 experiments in a
   dataset and the reference experiment is found to have 300 points in
   its trajectory point list, then this point matrix will have 25 columns
   and 300 rows.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoPointMatrix.H $
// $Id: LoPointMatrix.H 13934 2010-09-14 23:17:01Z mviswana $
//

#ifndef LOBOT_POINT_MATRIX_DOT_H
#define LOBOT_POINT_MATRIX_DOT_H

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoPointTypes.H"
#include "Robots/LoBot/thread/LoMutex.H"

// Boost headers
#include <boost/numeric/ublas/matrix.hpp>

// Standard C++ headers

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//------------------------- CLASS DEFINITION ----------------------------

// Forward declarations
class PointList ;

/**
   \class lobot::PointMatrix
   \brief A thread-safe container for storing "normalized" point lists.

   When we conduct an experiment to gauge lobot's performance, we collect
   several different lists of points: one to record the robot's
   trajectory from start to finish, another to record the locations where
   the robot's emergency stop behaviour was activated, another for the
   locations where the LGMD avoidance algorithm averted the robot away
   from an approaching obstacle; so on and so forth.

   Each experiment will produce point lists of different sizes. To
   compute the robot's average case behaviour, we will have to
   "normalize" these lists so that they all contain the same number of
   elements as some reference experiment. After we have discarded extra
   points or added missing points w.r.t. the reference experiment, we can
   find point correspondences across experiments and, finally, average
   these transformed lists to get the desired result.

   This class stores the intermediate "normalized" point lists in a
   matrix. The columns of this matrix correspond to the individual
   experiments. The rows store corresponding points across experiments.
   For example, if we have 25 experiments in a dataset and the reference
   experiment is found to have 300 points in its trajectory point list,
   then this point matrix will have 25 columns and 300 rows.

   To find the point correspondences between each experiment's point
   lists and that of the reference experiment, we find the nearest point
   to each point in the reference experiment. The resulting list of
   points is stored as a column vector in the point matrix.

   The correspondences for one experiment are completely independent
   of the correspondences for another. Therefore, we can parallelize the
   correspondence finding procedure by launching multiple threads to
   handle the different experiments.

   Consequently, this class implements a thread-safe API for adding
   column vectors. The main thread should instantiate this class and then
   pass its address to each of the correspondence finding threads.
*/
class PointMatrix {
   // Prevent copy and assignment
   PointMatrix(const PointMatrix&) ;
   PointMatrix& operator=(const PointMatrix&) ;

   /// This class's raison d'etre: to store a bunch of points in a
   /// matrix.
   //@{
   typedef boost::numeric::ublas::matrix<mPoint> Matrix ;
   Matrix m_matrix ;
   //@}

   /// This data member keeps track of the column index into which new
   /// point lists should be copied.
   unsigned int m_next_col ;

   /// When a correspondence finder thread is done processing one
   /// experiment, it will add the resulting "normalized" point list to
   /// an instance of this class. Since multiple correspondence finder
   /// threads can use this object, we need a mutex to ensure that they
   /// don't step on each others' toes.
   Mutex m_mutex ;

public:
   /// Initalization: creates a point matrix of the specified size.
   PointMatrix(int rows, int cols) ;

   /// This method adds the supplied point list to the point matrix by
   /// copying the point list to one of its columns. Since this method is
   /// called by the correspondence finder threads, it ensures
   /// thread-safety.
   ///
   /// WARNING: Attempting to add more point lists, i.e., column vectors,
   /// to the matrix than the number of columns (dataset size) will
   /// result in a lobot::misc_error(LOGIC_ERROR).
   void add(const PointList&) ;

   /// Computes the component-wise average of the matrix's column
   /// vectors.
   PointList average() const ;
} ;

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

#endif

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
