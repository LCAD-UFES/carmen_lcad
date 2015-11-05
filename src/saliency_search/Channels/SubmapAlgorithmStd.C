/*!@file Channels/SubmapAlgorithmStd.C Standard strategy for computing submaps in SingleChannel objects */

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
// Primary maintainer for this file: Rob Peters <rjpeters at usc dot edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SubmapAlgorithmStd.C $
// $Id: SubmapAlgorithmStd.C 6628 2006-05-20 04:30:30Z rjpeters $
//

#ifndef CHANNELS_SUBMAPALGORITHMSTD_C_DEFINED
#define CHANNELS_SUBMAPALGORITHMSTD_C_DEFINED

#include "Channels/SubmapAlgorithmStd.H"

#include "Channels/SingleChannel.H"
#include "Image/Image.H"

// ######################################################################
SubmapAlgorithmStd::SubmapAlgorithmStd(OptionManager& mgr,
                                       const std::string& descrName,
                                       const std::string& tagName)
  :
  SubmapAlgorithm(mgr, descrName, tagName)
{}

// ######################################################################
SubmapAlgorithmStd::~SubmapAlgorithmStd()
{}

// ######################################################################
Image<float> SubmapAlgorithmStd::compute(const SingleChannel& chan,
                                         const uint i)
{
  return chan.postProcessMap(chan.getRawCSmap(i), i);
}

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SUBMAPALGORITHMSTD_C_DEFINED
