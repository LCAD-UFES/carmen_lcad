/**
   \file  Robots/LoBot/misc/LoPointMatrix.C
   \brief This file defines the non-inline member functions of the
   lobot::PointMatrix class.
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Robots/LoBot/metlog/LoPointMatrix.C $
// $Id: LoPointMatrix.C 13935 2010-09-15 03:44:13Z mviswana $
//

//------------------------------ HEADERS --------------------------------

// lobot headers
#include "Robots/LoBot/metlog/LoPointMatrix.H"
#include "Robots/LoBot/metlog/LoPointList.H"
#include "Robots/LoBot/util/LoMath.H"
#include "Robots/LoBot/misc/LoExcept.H"

// Boost headers
#include <boost/numeric/ublas/matrix_proxy.hpp>

// Standard C++ headers
#include <numeric>
#include <algorithm>

//----------------------------- NAMESPACE -------------------------------

namespace lobot {

//-------------------------- INITIALIZATION -----------------------------

PointMatrix::PointMatrix(int rows, int cols)
   : m_matrix(rows, cols), m_next_col(0)
{
   std::fill(m_matrix.begin1(), m_matrix.end1(), mPoint(0, 0)) ;
}

//---------------------------- LIST ACCESS ------------------------------

void PointMatrix::add(const PointList& L)
{
   AutoMutex am(m_mutex) ;
   if (m_next_col >= m_matrix.size2())
      throw misc_error(LOGIC_ERROR) ;
   boost::numeric::ublas::matrix_column<Matrix> col(m_matrix, m_next_col++) ;
   std::copy(L.begin(), L.end(), col.begin()) ;
}

//----------------------------- AVERAGING -------------------------------

static mPoint add_points(const mPoint& a, const mPoint& b)
{
   return mPoint(a.first + b.first, a.second + b.second) ;
}

PointList PointMatrix::average() const
{
   if (m_next_col <= 0)
      return PointList() ;

   Matrix& matrix = const_cast<Matrix&>(m_matrix) ;
   const int    N = m_matrix.size1() ;
   const float  D = m_next_col ;

   PointList p(N) ;
   for (int i = 0; i < N; ++i)
   {
      boost::numeric::ublas::matrix_row<Matrix> row(matrix, i) ;
      mPoint sum =
         std::accumulate(row.begin(), row.end(), mPoint(0, 0), add_points) ;
      p.add(round(sum.first/D), round(sum.second/D)) ;
   }
   return p ;
}

//-----------------------------------------------------------------------

} // end of namespace encapsulating this file's definitions

/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
