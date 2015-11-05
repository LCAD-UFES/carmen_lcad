/*!@file Channels/FeatureBiaser.H */

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
// Primary maintainer for this file:
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/FeatureBiaser.H $
// $Id: FeatureBiaser.H 7071 2006-08-30 00:05:15Z rjpeters $
//

#ifndef CHANNELS_FEATUREBIASER_H_DEFINED
#define CHANNELS_FEATUREBIASER_H_DEFINED

#include "Channels/ChannelVisitor.H"

#include <vector>

/// Bias the features at submap idx with mean, intra-class std. dev and inter-sub-class std. dev
class FeatureBiaser : public ChannelVisitor
{
public:
  /// Construct with settings for the algorithm
  FeatureBiaser(const double* mean, const double* sigma);

  /// Virtual destructor
  virtual ~FeatureBiaser();

  /// Visit the given ChannelBase node.
  virtual void visitChannelBase(ChannelBase& chan);

  /// Visit the given SingleChannel node.
  virtual void visitSingleChannel(SingleChannel& chan);

  /// Visit the given ComplexChannel node.
  virtual void visitComplexChannel(ComplexChannel& chan);

private:
  const double* itsMean;
  const double* itsSigma;
  int itsIndex;
};


//! Weight of the channel is the maximum weight of its subchannels
/*! used for biasing to find targets */
class WeightFinder : public ChannelVisitor
{
public:
  /// Construct with settings for the algorithm
  WeightFinder();

  /// Virtual destructor
  virtual ~WeightFinder();

  /// Visit the given ChannelBase node.
  virtual void visitChannelBase(ChannelBase& chan);

  /// Visit the given SingleChannel node.
  virtual void visitSingleChannel(SingleChannel& chan);

  /// Visit the given ComplexChannel node.
  virtual void visitComplexChannel(ComplexChannel& chan);

private:
  std::vector<double> itsMax;
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_FEATUREBIASER_H_DEFINED
