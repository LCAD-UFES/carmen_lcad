/*!@file SIFT/VisualObjectMatchAlgo.H Header for visual object match algos */

// //////////////////////////////////////////////////////////////////// //
// The iLab Neuromorphic Vision C++ Toolkit - Copyright (C) 2001 by the //
// University of Southern California (USC) and the iLab at USC.         //
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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/SIFT/VisualObjectMatchAlgo.H $
// $Id: VisualObjectMatchAlgo.H 5755 2005-10-19 20:57:27Z rjpeters $
//

#ifndef VISUALOBJECTMATCHALGO_H_DEFINED
#define VISUALOBJECTMATCHALGO_H_DEFINED

#include <string>

//! A simple enum to decide which matching algo to use
/*! This is used by the SIFT code to decide on which matching
  algorithm to use. VOMA_SIMPLE is exhausive but slow, VOMA_KDTREE is
  based on using a KD-Tree but is even slower, and VOMA_KDTREEBBF is
  fast but approximate.  See SIFT/VisualObjectMatch.H for functions
  using these algos. If you add new algos here, be sure to update the
  number of algos and the string descriptions in the
  visualObjectMatchAlgoName() function below. */
enum VisualObjectMatchAlgo {
  VOMA_SIMPLE    =  0,
  VOMA_KDTREE    =  1,
  VOMA_KDTREEBBF =  2
};

//! Number of matching algos:
#define NBVISUALOBJECTMATCHALGOS 3

//! Text descriptions of the various known algos:
inline const char *visualObjectMatchAlgoName(const VisualObjectMatchAlgo val)
{
  static const char *txt[] = { "Simple", "KDtree", "KDtreeBBF" };
  return txt[int(val)];
}

//! Conversion to string
std::string convertToString(const VisualObjectMatchAlgo val);

//! Conversion from string
void convertFromString(const std::string& str, VisualObjectMatchAlgo& val);

#endif

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* indent-tabs-mode: nil */
/* End: */
