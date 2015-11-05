/*!@file Channels/ChannelVisitorInstallFacet.H Install some ChannelFacet */

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
// Primary maintainer for this file: Laurent Itti <itti@usc.edu>
// $HeadURL: svn://isvn.usc.edu/software/invt/trunk/saliency/src/Channels/ChannelVisitorInstallFacet.H $
// $Id: ChannelVisitorInstallFacet.H 10794 2009-02-08 06:21:09Z itti $
//

#ifndef CHANNELS_CHANNELVISITORINSTALLFACET_H_DEFINED
#define CHANNELS_CHANNELVISITORINSTALLFACET_H_DEFINED

#include "Channels/ChannelVisitor.H"

/// ChannelVisitorInstallFacet install some ChannelFacet
/** This visitor installs some ChannelFacet. The two template
    arguments are for the ChannelFacetSingle and the
    ChannelFacetComplex to be installed. */
template <class CFS, class CFC>
class ChannelVisitorInstallFacet : public ChannelVisitor {
public:
  /// Constructor
  ChannelVisitorInstallFacet();

  /// Destructor
  virtual ~ChannelVisitorInstallFacet();

  /// Visit the given ChannelBase node.
  /** This will just LFATAL() for ChannelVisitorInstallFacet. */
  virtual void visitChannelBase(ChannelBase& chan);

  /// Visit the given SingleChannel node.
  /** Install appropriate ChannelFacetGainSingle and
      ChannelFacetGainComplex if those do not already exist. Then load
      up the gains from our ParamMap. */
  virtual void visitSingleChannel(SingleChannel& chan);

  /// Visit the given ComplexChannel node.
  /** NOTE: ComplexChannel does NOT provide the iteration over its
      subchannels; instead, each ChannelVisitor subclass must iterate
      over ComplexChannel's subchannels, calling
      subChan(i)->accept(*this) for each one. Although this requires
      slightly more code for each ChannelVisitor subclass, it is also
      much more flexible since it lets the visitor be aware of the
      hierarchical structres of the various channels that it
      visits. */
  virtual void visitComplexChannel(ComplexChannel& chan);
};

// ######################################################################
/* So things look consistent in everyone's emacs... */
/* Local Variables: */
/* mode: c++ */
/* indent-tabs-mode: nil */
/* End: */

#endif
