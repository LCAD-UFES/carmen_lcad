/*!@file Image/Point2DT.H a retinotopic visual fixation */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2000-2003   //
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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Image/Point2DT.H $
// $Id: Point2DT.H 9412 2008-03-10 23:10:15Z farhan $
//

#ifndef POINT2DT_H_DEFINED
#define POINT2DT_H_DEFINED

#include "Image/Point2D.H"
#include "Util/SimTime.H"

//! This is a fairly trivial (i, j, t) set representing a timestamped Point2D<int>.
/*! This class is an open (all members public) container for a Point2D<int>
and a time stamp. Like Point2D<int>, this class is fully inlined, so there
is no Point2DT.C file. */

class Point2DT
{
public:
  //! Default contructor, initializes to zeros
  inline Point2DT() : p(0, 0), t(SimTime::ZERO()) { };

  //! Constructor given a Point2D<int> and time
  inline Point2DT(const Point2D<int>& pp, const SimTime& tt) : p(pp), t(tt) { };

  //! Constructor given (i, j) and time
  inline Point2DT(const int i, const int j,
                  const SimTime& tt) : p(i, j), t(tt) { };

  //! returns true if coordinates are not (-1, -1)
  inline bool isValid() const;


  Point2D<int> p;  //!< The spatial coordinates
  SimTime t;  //!< The time, in seconds
};

// ######################################################################
// #################### INLINED METHODS:
// ######################################################################
inline bool Point2DT::isValid() const
{
  return p.i != -1 && p.j != -1;
}


// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
#endif
