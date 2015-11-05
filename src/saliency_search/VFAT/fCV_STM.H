/*!@file VFAT/fCV_STM.H Short Term Memory for Feature Cluster Vision
 */

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
// Primary maintainer for this file: T Nathan Mundhenk <mundhenk@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/VFAT/fCV_STM.H $
// $Id: fCV_STM.H 4663 2005-06-23 17:47:28Z rjpeters $
//

#ifndef FCV_STM_H_DEFINED
#define FCV_STM_H_DEFINED

#include "VFAT/covHolder.H"

#include <string>
#include <vector>

class fCV_STM
{
public:
  //! default constructor
  fCV_STM();
  //! default destructor
  ~fCV_STM();
  //! write memory contents to hard drive, true if success
  bool STMwriteMemory();
  //! get memory contents from hard drive, true if success
  bool STMreadMemory(std::string memFile);
  //! add an item to memory, return result
  int STMaddMemory(covHolder<double> &cov);
private:
  std::vector<covHolder<double> > STM_covMemory;
};

#endif // !FCV_STM_H_DEFINED
