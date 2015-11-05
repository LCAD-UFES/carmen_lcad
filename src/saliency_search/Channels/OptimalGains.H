/*!@file Channels/OptimalGains.H Compute the optimal gains that maximize SNR */

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
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/OptimalGains.H $
// $Id: OptimalGains.H 10028 2008-08-01 03:12:32Z itti $
//

#ifndef CHANNELS_OPTIMALGAINS_H_DEFINED
#define CHANNELS_OPTIMALGAINS_H_DEFINED

#include "Channels/ChannelVisitor.H"
#include "Image/Image.H"
#include "rutz/shared_ptr.h"

class ParamMap;

/// Compute the optimal gains that maximize SNR
/** This ChannelVisitor computes the salience of the target and
    distractors using the masks as cues for their locations. Based on
    the salience, it finds SNR and the gains; however, SNR and gains
    are only used internally (they are needed to compute the salience
    at the next level up the channel hierarchy) but are not saved
    here. Rather, the salienceT / salienceD info is saved. One can
    then use app-combineOptimalGains to compute the gains, and a
    PrefrontalCortexGS to use them.  */
class OptimalGainsFinder : public ChannelVisitor
{
public:
  /// Construct with settings for the algorithm
  OptimalGainsFinder(const Image<byte>& targetMask,
                     const Image<byte>& distractorMask,
                     rutz::shared_ptr<ParamMap> pmap,
                     const bool doMax = true);

  /// Virtual destructor
  virtual ~OptimalGainsFinder();

  /// Visit the given ChannelBase node.
  virtual void visitChannelBase(ChannelBase& chan);

  /// Visit the given SingleChannel node.
  virtual void visitSingleChannel(SingleChannel& chan);

  /// Visit the given ComplexChannel node.
  virtual void visitComplexChannel(ComplexChannel& chan);

  /// Get our pmap, it will contain saliencsT and salienceD values after visit
  virtual rutz::shared_ptr<ParamMap> pmap() const;

private:
  Image<byte> itsTargetMask;
  Image<byte> itsDistractorMask;
  rutz::shared_ptr<ParamMap> itsPmap;
  bool itsDoMax;
};

//! Background firing rate to use when computing SNR
#define OPTIGAIN_BG_FIRING 0.1

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif // CHANNELS_OPTIMALGAINS_H_DEFINED
