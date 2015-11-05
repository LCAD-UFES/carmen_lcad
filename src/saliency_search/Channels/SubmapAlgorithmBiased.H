/*!@file Channels/SubmapAlgorithmBiased.H Compute SingleChannel submaps with Bayesian biasing */

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
// Primary maintainer for this file: Lior Elazary <elazary@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/SubmapAlgorithmBiased.H $
// $Id: SubmapAlgorithmBiased.H 7036 2006-08-24 23:26:15Z rjpeters $
//

#ifndef CHANNELS_SUBMAPALGORITHMBIASED_H_DEFINED
#define CHANNELS_SUBMAPALGORITHMBIASED_H_DEFINED

#include "Channels/SubmapAlgorithm.H"

class ChannelBase;

/// Compute SingleChannel submaps with Bayesian biasing
class SubmapAlgorithmBiased : public SubmapAlgorithm
{
public:
  /// Constructor
  SubmapAlgorithmBiased(OptionManager& mgr,
                        const std::string& descrName = "Biased Submap Algorithm",
                        const std::string& tagName = "SubmapAlgorithmBiased");

  /// Destructor
  virtual ~SubmapAlgorithmBiased();

  /// Compute the i'th submap for the given channel
  virtual Image<float> compute(const SingleChannel& chan,
                               const uint i);
};

/// Install SubmapAlgorithmBiased in all SingleChannel objects underneath chan
void setSubmapAlgorithmBiased(ChannelBase& chan);

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_SUBMAPALGORITHMBIASED_H_DEFINED
